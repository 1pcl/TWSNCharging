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
    private EdgesMapper distanceMapper;

    @Transactional
    public void saveDataset2(String setName, List<Double> currentLocation) {
        try {
            datasetNameMapper.insertDatasetInfo(setName, currentLocation.get(0), currentLocation.get(1));
            String[] keys = new String[]{"sensor", "relay", "station","intersection","edges"};
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
        String[] keys = new String[]{"sensor", "relay", "station","intersection","edges"};
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
        pointMapper.renameTable("relay_" + oldName, "relay_" + newName);
        pointMapper.renameTable("sensor_" + oldName, "sensor_" + newName);
        pointMapper.renameTable("station_" + oldName, "station_" + newName);
        pointMapper.renameTable("intersection_"+oldName, "intersection_" + newName);
        pointMapper.renameTable("edges_"+oldName, "edges_" + newName);
    }


    @Transactional
    public List<Double> datasetLoad2(String setName) {
        String[] keys = new String[]{"sensor", "relay", "station","intersection","edges"};

        for (String key : keys) {
            pointMapper.transferData(key, setName);
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
        String[][][] relay_array = quickSave.getRelay_points();
        String[][][] sensor_array = quickSave.getSensor_array();
        String[][][] station_array = quickSave.getStation_array();
        String[][][] road_array = quickSave.getRoad_array();
        pointsService.clear2();
        pointsService.quick_write_new2(relay_array, sensor_array,station_array,road_array);
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
