package top.bultrail.markroad.service;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import top.bultrail.markroad.entity.BatteryInfo;
import top.bultrail.markroad.entity.DistanceNode;
import top.bultrail.markroad.entity.PointInfo;
import top.bultrail.markroad.mapper.DatasetNameMapper;
import top.bultrail.markroad.mapper.DistanceMapper;
import java.util.*;
import java.util.stream.Collectors;

@Service
public class WSNSimulation {

    @Autowired
    PointsService pointsService;

    @Autowired
    private DatasetNameMapper datasetNameMapper;

    @Autowired
    private DistanceMapper distanceMapper;

    @Autowired
    private GraphHopperRoutingService graphHopperRoutingService;


    private final double sensorBatteryCapacity = 240000.0; // 对应 self.sensorBattery
    private final double gatewayBatteryCapacity = 800000.0; // 对应 self.gatewayBattery
    private final Random random = new Random(42); // 对应 Python 的 random

//    private Map<String, Map<String, Double>> graphG = new HashMap<>(); // 对应 self.graph_G
    private Map<Integer, Map<Integer, Double>> graphG = new HashMap<>();
    private List<PointInfo> pointInfoList = new ArrayList<>(); // 对应 self.point_info
//    private Map<String, Integer> originalLens = new HashMap<>();

    // 成员变量
    private Map<String, Map<String, Double>> maxMin = new HashMap<>();

    @Transactional
    public Map<String, Object> runWSNSimulation() {
        // 1. 获取原始数据
        List<List<String>> sensorList = pointsService.readSensor2();
        List<List<String>> gateWayList = pointsService.readGateWay2();
        List<List<String>> stationList = pointsService.readStation2();
        //读取distance数据作为边  List<DistanceNode> selectAllDistance();
        List<DistanceNode> graphEdges = distanceMapper.selectAllDistance();

        // 2. 转换数据格式 (对应 Python 的 _convert_data_format)
        _convertDataFormat(sensorList, gateWayList, stationList);

        _calculateGlobalMaxMin(graphEdges);

        // 3. 筛选低电量节点 (对应 Python 的 _select_low_battery_nodes)
        _selectLowBatteryNodes();

        // 4. 构建过滤后的图 (对应 Python 的 _build_filtered_graph)
        _buildFilteredGraph(graphEdges);

        // 5. 返回结果 (对应 Python 的 get_data)
        return getSimulationData();
    }
    /**
     * 对应 Python 的 _convert_data_format：将 List<List<String>> 转换为 PointInfo 列表
     * 假设 List<String> 格式为: [ID(String), Lng(String), Lat(String), ...]
     */
    private void _convertDataFormat(List<List<String>> sensorList, List<List<String>> gateWayList, List<List<String>> stationList) {
        pointInfoList.clear();

        int stationCount = stationList != null ? stationList.size() : 0;
        int gatewayCount = gateWayList != null ? gateWayList.size() : 0;

        // 辅助方法：将 List<List<String>> 转换为 PointInfo 列表
        // 假设索引 0: ID, 1: Lng, 2: Lat
        _addNodes(gateWayList, "gateWay",stationCount,gatewayCount);
        _addNodes(sensorList, "sensor",stationCount,gatewayCount);
        _addNodes(stationList, "station",stationCount,gatewayCount);
    }

    private void _calculateGlobalMaxMin(List<DistanceNode> graphEdges) {
        // 1. 初始化统计结构，显式指定类型避免推断错误
        Map<String, Double> distMap = new HashMap<>();
        distMap.put("max", 0.0);
        distMap.put("min", Double.MAX_VALUE);
        maxMin.put("distance", distMap);

        Map<String, Double> lngMap = new HashMap<>();
        lngMap.put("max", -180.0);
        lngMap.put("min", 180.0);
        maxMin.put("longitude", lngMap);

        Map<String, Double> latMap = new HashMap<>();
        latMap.put("max", -90.0);
        latMap.put("min", 90.0);
        maxMin.put("latitude", latMap);

        // 2. 计算距离的最大最小值
        if (graphEdges != null) {
            for (DistanceNode edge : graphEdges) {
                double dist = edge.getDistance();
                distMap.put("max", Math.max(distMap.get("max"), dist));
                distMap.put("min", Math.min(distMap.get("min"), dist));
            }
        }

        // 3. 计算经纬度的最大最小值 (基于原始 pointInfoList)
        for (PointInfo p : pointInfoList) {
            double lng = p.getLng();
            double lat = p.getLat();
            lngMap.put("max", Math.max(lngMap.get("max"), lng));
            lngMap.put("min", Math.min(lngMap.get("min"), lng));
            latMap.put("max", Math.max(latMap.get("max"), lat));
            latMap.put("min", Math.min(latMap.get("min"), lat));
        }
    }

    private void _addNodes(List<List<String>> rawList, String type, int stationCount, int gatewayCount ) {
        for (List<String> rawNode : rawList) {
            if (rawNode.size() >= 4) {
                PointInfo p = new PointInfo();
                try {
                    int originalId = Integer.parseInt(rawNode.get(0));
                    p.setOriginalId(originalId);
//                    p.setNodeId(type + "_" + rawNode.get(0));
                    p.setType(type);
                    p.setLng(Double.parseDouble(rawNode.get(2)));
                    p.setLat(Double.parseDouble(rawNode.get(3)));
                    // 计算nodeIdInt，基于generateAllNodes的逻辑
                    int nodeIdInt;
                    switch (type) {
                        case "station":
                            // station: nodeId = station.id - 1
                            nodeIdInt = originalId - 1;
                            break;
                        case "gateWay":
                            // gateWay: nodeId = gateWay.id - 1 + stationCount
                            nodeIdInt = originalId - 1 + stationCount;
                            break;
                        case "sensor":
                            // sensor: nodeId = sensor.id - 1 + stationCount + gatewayCount
                            nodeIdInt = originalId - 1 + stationCount + gatewayCount;
                            break;
                        default:
                            nodeIdInt = originalId; // 默认使用原始ID
                            break;
                    }
                    p.setNodeId(nodeIdInt);
                    pointInfoList.add(p);
                } catch (NumberFormatException e) {
                    System.err.println("Error parsing data for node type " + type + ": " + e.getMessage());
                }
            }
        }
    }
    /**
     * 根据数据集名称获取低电量节点配置
     * data1: 30传感器 + 3网关
     * data2: 60传感器 + 6网关
     * data3: 40传感器 + 4网关
     * data4: 50传感器 + 5网关
     * 默认: 30传感器 + 3网关
     */
    private Map<String, Integer> getLowBatteryConfig() {
        Map<String, Integer> config = new HashMap<>();

        try {
            List<String> names = datasetNameMapper.selectNamesByIsDataset();
            String datasetName = (names != null && !names.isEmpty()) ? names.get(0) : "data1";

            switch (datasetName.toLowerCase()) {
                case "data1":
                    config.put("sensor", 30);
                    config.put("gateway", 3);
                    break;
                case "data2":
                    config.put("sensor", 60);
                    config.put("gateway", 6);
                    break;
                case "data3":
                    config.put("sensor", 40);
                    config.put("gateway", 4);
                    break;
                case "data4":
                    config.put("sensor", 50);
                    config.put("gateway", 5);
                    break;
                default:
                    // 默认配置
                    config.put("sensor", 30);
                    config.put("gateway", 3);
                    System.err.println("Unknown dataset name: " + datasetName + ", using default config");
                    break;
            }
        } catch (Exception e) {
            // 异常时使用默认配置
            config.put("sensor", 30);
            config.put("gateway", 3);
            System.err.println("Error getting dataset name: " + e.getMessage() + ", using default config");
        }

        return config;
    }


    /**
     * 对应 Python 的 _select_low_battery_nodes：随机选择低电量节点并设置电量信息
     */
//    private void _selectLowBatteryNodes(Random random) {
    private void _selectLowBatteryNodes() {
        // 统计原始长度（此时 pointInfoList 包含所有读取到的节点）
//        originalLens.put("sensor", (int) pointInfoList.stream().filter(p -> "sensor".equals(p.getType())).count());
//        originalLens.put("gateWay", (int) pointInfoList.stream().filter(p -> "gateWay".equals(p.getType())).count());
//        originalLens.put("station", (int) pointInfoList.stream().filter(p -> "station".equals(p.getType())).count());

        Map<String, Integer> config = getLowBatteryConfig();
        int targetSensorCount = config.get("sensor");
        int targetGatewayCount = config.get("gateway");

        // 1. 筛选所有传感器和中继节点
        List<PointInfo> allSensors = pointInfoList.stream()
                .filter(p -> "sensor".equals(p.getType()))
                .collect(Collectors.toList());

        List<PointInfo> allGateWays = pointInfoList.stream()
                .filter(p -> "gateWay".equals(p.getType()))
                .collect(Collectors.toList());

//        // 2. 固定选择数量 (对应 Python 代码中的逻辑)
//        final int sensorLowBatteryCount = Math.min(30, allSensors.size());
//        final int gateWaysLowBatteryCount = Math.min(3, allGateWays.size());
        final int sensorLowBatteryCount = Math.min(targetSensorCount, allSensors.size());
        final int gateWaysLowBatteryCount = Math.min(targetGatewayCount, allGateWays.size());

        // 3. 随机选择
        List<PointInfo> selectedSensors = selectRandomElements(allSensors, sensorLowBatteryCount,random);
        List<PointInfo> selectedGateWays = selectRandomElements(allGateWays, gateWaysLowBatteryCount,random);

        // 4. 设置电量信息 (移植 Python 逻辑)
        setBatteryInfo(selectedSensors, sensorBatteryCapacity, 0.05, 0.5,random); // 传感器：0.05-0.5 J/s
        setBatteryInfo(selectedGateWays, gatewayBatteryCapacity, 0.5, 1.0,random); // 中继节点：0.5-1.0 J/s

        // 5. 过滤 pointInfoList (只保留 station, selected_sensors, selected_gateWays)
        Set<Integer> validNodeIds = new HashSet<>();
        selectedSensors.forEach(p -> validNodeIds.add(p.getNodeId()));
        selectedGateWays.forEach(p -> validNodeIds.add(p.getNodeId()));

        this.pointInfoList = this.pointInfoList.stream()
                .filter(p -> "station".equals(p.getType()) || validNodeIds.contains(p.getNodeId()))
                .collect(Collectors.toList());
    }

    // --- 辅助方法：设置电量信息 ---
    private void setBatteryInfo(List<PointInfo> nodes, double capacity, double minRate, double maxRate,Random random) {
        for (PointInfo node : nodes) {
            // currentEnergyRatio: 0.3-0.5
            double currentEnergyRatio = random.nextDouble() * 0.2 + 0.3;
            double currentEnergy = capacity * currentEnergyRatio;

            // 注意：根据 PointInfo 的字段定义获取，通常是 getLat() 和 getLng()
            double lat = node.getLat();
            double lon = node.getLng();

            double chargingRate = 0;

            if(sensorBatteryCapacity==capacity)
                chargingRate=calculateChargingRate(lat,lon,0.0,3.0);
//                chargingRate = random.nextDouble() * 0.05 + 0.9;
            else
                chargingRate=calculateChargingRate(lat,lon,3.0,1.8);
//                chargingRate = random.nextDouble() * 0.05 + 0.85;


            BatteryInfo info = new BatteryInfo();
            info.setCurrentEnergy(currentEnergy);
            info.setCurrentEnergyRatio(currentEnergyRatio);
            info.setChargingRate(chargingRate);

            node.setBatteryInfo(info);
        }
    }

    /**
     * 依照论文公式 (1) 和 (6) 计算充电效率
     * chargingRate = P_R / P_T
     */
    public double calculateChargingRate(double sensorLat, double sensorLon, double d_v, double n) {
        // 1. 获取水平距离 (传感器到最近道路边缘的距离)
        double d_h = graphHopperRoutingService.getHorizontalDistanceToRoad(sensorLat, sensorLon);

        // 2. 计算 3D 欧几里得距离
        double d_3d = Math.sqrt(Math.pow(d_h, 2) + Math.pow(d_v, 2));

        double d_ref = 1.0;
        double eta_ref = 0.98; // 1米处的基准效率
//        天线增益补偿因子
        double alpha = 6.0;

//        double chargingRate = alpha * eta_ref * Math.pow(d_ref / d_3d, n);
        // 1. 计算原始充电效率
        double rawRate = alpha * eta_ref * Math.pow(d_ref / d_3d, n);

        // 2. 限制范围在 [0.60, 0.98] 之间
        double chargingRate = Math.max(0.70, Math.min(0.98, rawRate));

        return chargingRate;
    }

    // --- 辅助方法：实现随机采样 (random.sample) ---
    private List<PointInfo> selectRandomElements(List<PointInfo> source, int count,Random random) {
        if (source.size() <= count) {
            return new ArrayList<>(source);
        }
        List<PointInfo> shuffledList = new ArrayList<>(source);
        Collections.shuffle(shuffledList, random);
        return shuffledList.subList(0, count);
    }

    /**
     * 对应 Python 的 _build_filtered_graph：构建过滤后的图结构
     */
    private void _buildFilteredGraph(List<DistanceNode> rawGraphEdges) {
    // 1. 对应 Python 的 valid_node_ids
        Set<Integer> validNodeIds = pointInfoList.stream()
                .map(PointInfo::getNodeId)
                .collect(Collectors.toSet());

        this.graphG.clear();

        if (rawGraphEdges == null) return;

        // 2. 循环体：使用 DistanceNode 类型
        for (DistanceNode edge : rawGraphEdges) {
            // 使用 Getter 方法获取字段 (对应 Python 的 edge["key"])
            int start = edge.getNodeStartId();
            int end = edge.getNodeEndId();
            Double dist = edge.getDistance();

            // 3. 过滤并构建图 (与 Python 逻辑一致)
            if (validNodeIds.contains(start) && validNodeIds.contains(end)) {
                // Java 8 语法：如果 start 不存在，则创建新的 HashMap
                // 对应 Python 的 if start not in self.graph_G: self.graph_G[start] = {}
                this.graphG.computeIfAbsent(start, k -> new HashMap<>()).put(end, dist);
            }
        }
    }

    /**
     * 对应 Python 的 get_data：返回 graph_G 和 point_info 数据
     * @return 包含 graph_G 和 point_info 的 Map
     */
    public Map<String, Object> getSimulationData() {
        Map<String, Object> result = new HashMap<>();

        result.put("graph_G", this.graphG);
        result.put("point_info", this.pointInfoList);
//        result.put("len", this.originalLens);
        result.put("max_min", this.maxMin);
        return result;
    }
}
