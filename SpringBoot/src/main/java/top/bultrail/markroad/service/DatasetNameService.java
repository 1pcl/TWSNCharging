package top.bultrail.markroad.service;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.ArrayList;
import java.util.List;

import top.bultrail.markroad.entity.QuickSave;
import top.bultrail.markroad.mapper.*;

import top.bultrail.markroad.entity.DatasetInfo;
import top.bultrail.markroad.entity.DatasetLocation;

@Service
public class DatasetNameService {

    @Autowired
    private PointMapper pointMapper;
    @Autowired
    private DatasetNameMapper datasetNameMapper;

    @Autowired
    PointsService pointsService;

    @Autowired
    private DistanceMapper distanceMapper;

    @Transactional
    public void saveDataset2(String setName, List<Double> currentLocation) {
        try {
            datasetNameMapper.insertDatasetInfo(setName, currentLocation.get(0), currentLocation.get(1));
            String[] keys = new String[]{"sensor", "gateWay", "station","distance"};
            for (String key : keys) {
                String newTableName = key + "_" + setName;
                pointMapper.createTableLike(newTableName, key);
                pointMapper.copyDataToNewTable(newTableName, key);
            }
            getAndClearActiveDatasets();
            datasetNameMapper.updateIsDatasetStatus(setName, 1);
        } catch (Exception e) {
            e.printStackTrace(); // 打印异常信息到控制台
            throw e; // 重新抛出异常，以便进行进一步处理
        }
    }

    //删除数据集
    @Transactional
    public void deleteDataset2(String setName) {
        String[] keys = new String[]{"sensor", "gateWay", "station","distance"};
        // 删除相关数据表
        for (String key : keys) {
            String tableName = key + "_" + setName;
            pointMapper.dropTable(tableName);
        }
        datasetNameMapper.deleteByPrimaryKey(setName);
    }


    public List<DatasetInfo> searchDatasetInfo2() {
        return datasetNameMapper.selectDatasetInfo();
    }

    @Transactional
    public void renameDataset2(String oldName, String newName) {
        if (datasetNameMapper.existsByName(newName)) {
            throw new IllegalArgumentException("Name already exists.");
        }
        // 调用 Mapper/DAO 层的方法来执行数据库更新（charging_,sensor_,gateway_,station_,road_,node_,road_node_）
        datasetNameMapper.updateDatasetName(oldName, newName);
        pointMapper.renameTable("gateWay_" + oldName, "gateWay_" + newName);
        pointMapper.renameTable("sensor_" + oldName, "sensor_" + newName);
        pointMapper.renameTable("station_" + oldName, "station_" + newName);
        pointMapper.renameTable("distance_"+oldName, "distance_" + newName);
    }


    @Transactional
    public List<Double> datasetLoad2(String setName) {
        String[] keys = new String[]{"sensor", "gateWay", "station","distance"};

        for (String key : keys) {
//            long startTime = System.currentTimeMillis();

            if ("distance".equals(key)) {
                // 大数据表使用分批
                batchTransfer(key, setName);
            } else {
                // 小表一次性插入
                pointMapper.transferData(key, setName);
            }

//            long endTime = System.currentTimeMillis();
//            System.out.println("表 " + key + " 加载耗时: " + (endTime - startTime) + "ms");
        }

        DatasetLocation location = datasetNameMapper.selectDatasetLocation(setName);
        List<Double> locationList = new ArrayList<>();
        if (location != null) {
            locationList.add(location.getLocationLng());
            locationList.add(location.getLocationLat());
        }
        getAndClearActiveDatasets();
        datasetNameMapper.updateIsDatasetStatus(setName, 1);
        return locationList;
    }

    // 新增分批插入方法
    private void batchTransfer(String tableName, String setName) {
        try {
            // 先获取总记录数，用于显示进度
            int totalRecords = pointMapper.getTableCount(tableName, setName);
            System.out.println(tableName + "_" + setName + " 总记录数: " + totalRecords);

            if (totalRecords == 0) {
                System.out.println("表为空，跳过");
                return;
            }

            int batchSize = 50000;
            int offset = 0;
            int totalInserted = 0;
            int batchNum = 0;

            while (true) {
                batchNum++;
                long startTime = System.currentTimeMillis();

                int inserted = pointMapper.transferDataBatch(tableName, setName, offset, batchSize);
                long endTime = System.currentTimeMillis();

                if (inserted == 0) {
                    System.out.println("批次" + batchNum + ": 没有更多数据");
                    break;
                }

                totalInserted += inserted;
                offset += batchSize;

                // 计算进度百分比
                int progress = (int) ((totalInserted * 100.0) / totalRecords);

                System.out.println(String.format(
                        "%s 批次%d: 插入%,d条 (累计:%,d/%,d, %d%%) 耗时:%dms",
                        tableName, batchNum, inserted, totalInserted, totalRecords,
                        progress, (endTime - startTime)
                ));

                // 判断是否最后一批
                if (inserted < batchSize) {
                    System.out.println(tableName + " 最后一批完成，共插入 " + totalInserted + " 条");
                    break;
                }

                // 大数据表控制频率
                if ("path".equals(tableName) && batchNum % 10 == 0) {
                    Thread.sleep(50);
                }
            }

            // 验证总数
            if (totalInserted != totalRecords) {
                System.err.println("警告: 插入总数(" + totalInserted +
                        ") 与总记录数(" + totalRecords + ") 不一致！");
            }

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException("分批插入被中断", e);
        } catch (Exception e) {
            System.err.println("分批插入失败: " + e.getMessage());
            throw e;
        }
    }

    /**
     * 获取所有活跃数据集名称并将其状态清零
     * @return 被清零的数据集名称列表
     */
    @Transactional // 开启事务，保证查询和多次更新的一致性
    public List<String> getAndClearActiveDatasets() {
        // 1. 获取所有 is_dataset = 1 的名称
        List<String> activeNames = datasetNameMapper.selectNamesByIsDataset();

        // 2. 如果列表不为空，遍历并清零
        if (activeNames != null && !activeNames.isEmpty()) {
            for (String name : activeNames) {
                // 将 is_dataset 改为 0
                datasetNameMapper.updateIsDatasetStatus(name, 0);
            }
        }

        // 3. 返回这些名称（以便前端或后续逻辑知道哪些被修改了）
        return activeNames;
    }


    public void quicksaveDB(QuickSave quickSave) {
        String[][][] gateWay_points = quickSave.getGateWay_points();
        String[][][] sensor_array = quickSave.getSensor_array();
        String[][][] station_array = quickSave.getStation_array();
        pointsService.clear2();
        pointsService.quick_write_new2(gateWay_points, sensor_array,station_array);
    }

    public void saveDataset(String datasetName, List<Double> currentLocation) {
        saveDataset2(datasetName, currentLocation);
    }

    public void deleteDataset(String datasetName) {
        deleteDataset2(datasetName);
    }


    public List<DatasetInfo> searchDatasetInfo() {
        List<DatasetInfo> datasetInfos;
        datasetInfos = searchDatasetInfo2();
        return datasetInfos;
    }

    public void renameDataset(String oldName, String newName) {
        // 调用数据库层进行重命名操作
        renameDataset2(oldName, newName);
    }


    //清空数据库 新增
    public void clearDB() {
        pointsService.clear2();
    }


    public List<Double> datasetLoad(String setName) {
        pointsService.clear2();
        List<Double> location = datasetLoad2(setName);
        return location;
    }

}
