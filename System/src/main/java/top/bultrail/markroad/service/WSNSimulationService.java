// WSNSimulationService.java - 完整修复

package top.bultrail.markroad.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;
import top.bultrail.markroad.entity.Points;
import top.bultrail.markroad.entity.Edges;
import top.bultrail.markroad.entity.DatasetLocation;
import top.bultrail.markroad.mapper.PointMapper;
import top.bultrail.markroad.mapper.EdgesMapper;
import top.bultrail.markroad.mapper.DatasetNameMapper;

import java.util.*;

@Service
public class WSNSimulationService {

    @Value("${python.wsn.url:http://127.0.0.1:5000/api/runWSN}")
    private String wsnApiUrl;

    private final RestTemplate restTemplate;
    private final ObjectMapper pythonObjectMapper;

    @Autowired
    private PointMapper pointMapper;

    @Autowired
    private EdgesMapper edgesMapper;

    @Autowired
    private PointsService pointsService;

    @Autowired
    private DatasetNameMapper datasetNameMapper;

    // 参考点坐标（从当前激活的数据集加载）
    private double refLon = 0;
    private double refLat = 0;
    private static final double EARTH_RADIUS = 6371000;

    @Autowired
    public WSNSimulationService(RestTemplate restTemplate,
                                @Qualifier("pythonObjectMapper") ObjectMapper pythonObjectMapper) {
        this.restTemplate = restTemplate;
        this.pythonObjectMapper = pythonObjectMapper;
    }

    /**
     * 确保参考点已加载
     */
    private void ensureReferencePointLoaded() {
        if (refLon == 0 && refLat == 0) {
            loadReferencePointFromDatabase();
        }
    }

    /**
     * 从数据库加载当前激活数据集的参考点坐标
     */
    private void loadReferencePointFromDatabase() {
        try {
            List<String> activeDatasets = datasetNameMapper.selectNamesByIsDataset();
            if (activeDatasets != null && !activeDatasets.isEmpty()) {
                String activeDataset = activeDatasets.get(0);
                DatasetLocation location = datasetNameMapper.selectDatasetLocation(activeDataset);
                if (location != null) {
                    this.refLon = location.getLocationLng();
                    this.refLat = location.getLocationLat();
                    System.out.println("从数据集 [" + activeDataset + "] 加载参考点: (" + refLon + ", " + refLat + ")");
                } else {
                    System.err.println("警告: 数据集 [" + activeDataset + "] 没有有效的中心坐标，使用默认值");
                    setDefaultReferencePoint();
                }
            } else {
                System.err.println("警告: 没有激活的数据集，使用默认参考点");
                setDefaultReferencePoint();
            }
        } catch (Exception e) {
            System.err.println("加载参考点失败: " + e.getMessage());
            setDefaultReferencePoint();
        }
    }

    /**
     * 设置默认参考点
     */
    private void setDefaultReferencePoint() {
        this.refLon = 113.34823;
        this.refLat = 23.15829;
        System.out.println("使用默认参考点: (" + refLon + ", " + refLat + ")");
    }

    /**
     * 经纬度转米制坐标（给Python算法）
     */
    private double[] lonlatToMeters(double lon, double lat) {
        ensureReferencePointLoaded();
        double latRad = Math.toRadians(lat);
        double refLatRad = Math.toRadians(refLat);

        double dx = EARTH_RADIUS * Math.cos(refLatRad) * Math.toRadians(lon - refLon);
        double dy = EARTH_RADIUS * Math.toRadians(lat - refLat);

        return new double[]{dx, dy};
    }

    /**
     * 米制坐标转经纬度（给Vue前端）
     */
    private double[] metersToLonlat(double x, double y) {
        ensureReferencePointLoaded();
        double lon = refLon + (x / (EARTH_RADIUS * Math.cos(Math.toRadians(refLat)))) * (180 / Math.PI);
        double lat = refLat + (y / EARTH_RADIUS) * (180 / Math.PI);
        return new double[]{lon, lat};
    }

    /**
     * 从数据库加载完整的道路图数据，构建Python需要的JSON格式
     * 坐标转换为米制直角坐标系（给Python算法使用）
     */
    private Map<String, Object> loadRoadGraphFromDatabase() {
        Map<String, Object> roadGraphData = new HashMap<>();

        try {
            ensureReferencePointLoaded();

            List<Points> allNodes = pointsService.getAllNodes();
            List<Map<String, Object>> nodes = new ArrayList<>();

            System.out.println("加载道路网络数据，共 " + allNodes.size() + " 个节点");

            for (Points point : allNodes) {
                Map<String, Object> node = new HashMap<>();
                node.put("id", point.getId());

                // 转换为米制坐标（传给Python）
                double[] xy = lonlatToMeters(point.getLng(), point.getLat());
                double x = xy[0];
                double y = xy[1];

                node.put("x", x);
                node.put("y", y);

                // 根据ID前缀判断节点类型
                String nodeType;
                String id = point.getId();
                if (id.startsWith("ST")) {
                    nodeType = "station";
                } else if (id.startsWith("S")) {
                    nodeType = "sensor";
                } else if (id.startsWith("G")) {
                    nodeType = "relay";
                } else if (id.startsWith("I")) {
                    nodeType = "intersection";
                } else {
                    nodeType = "unknown";
                }
                node.put("node_type", nodeType);

                // 获取该节点的邻居和度数
                List<Edges> neighborEdges = edgesMapper.selectNeighborsByNode(point.getId());
                List<String> neighbors = new ArrayList<>();
                for (Edges edge : neighborEdges) {
                    if (edge.getFrom_node().equals(point.getId())) {
                        neighbors.add(edge.getTo_node());
                    } else {
                        neighbors.add(edge.getFrom_node());
                    }
                }
                node.put("neighbors", neighbors);
                node.put("degree", neighbors.size());

                nodes.add(node);
            }

            List<Edges> edges = edgesMapper.selectAllEdges();
            List<Map<String, Object>> edgeList = new ArrayList<>();
            for (Edges edge : edges) {
                Map<String, Object> edgeMap = new HashMap<>();
                edgeMap.put("from", edge.getFrom_node());
                edgeMap.put("to", edge.getTo_node());
                edgeMap.put("length_m", edge.getLength_m());
                edgeList.add(edgeMap);
            }

            roadGraphData.put("nodes", nodes);
            roadGraphData.put("edges", edgeList);
            roadGraphData.put("unit", "meters");
            roadGraphData.put("reference_point", new double[]{refLon, refLat});

            System.out.println("从数据库加载了 " + nodes.size() + " 个节点， " + edges.size() + " 条边");
            System.out.println("参考点: (" + refLon + ", " + refLat + ")");

        } catch (Exception e) {
            System.err.println("从数据库加载道路图数据失败: " + e.getMessage());
            e.printStackTrace();
            roadGraphData.put("nodes", new ArrayList<>());
            roadGraphData.put("edges", new ArrayList<>());
        }

        return roadGraphData;
    }

    /**
     * 处理WSN仿真返回结果 - 同时保留米制坐标和经纬度
     * - x/y: 米制坐标（Python返回，保持不变）
     * - lng/lat: 经纬度（自动从米制坐标转换，给Vue前端）
     */
    @SuppressWarnings("unchecked")
    private Map<String, Object> processWSNResponse(ResponseEntity<Map> response) {
        Map<String, Object> responseBody = response.getBody();

        if (responseBody == null) {
            throw new RuntimeException("WSN仿真返回空响应");
        }

        String status = (String) responseBody.get("status");
        if (!"success".equals(status)) {
            String message = (String) responseBody.get("message");
            throw new RuntimeException("WSN仿真失败: " + message);
        }

        Map<String, Object> data = (Map<String, Object>) responseBody.get("data");
        if (data == null) {
            throw new RuntimeException("WSN仿真返回数据为空");
        }

        // 转换 point_info：Python返回的是米制坐标，需要转换为经纬度给前端
        List<Map<String, Object>> pointInfo = (List<Map<String, Object>>) data.get("point_info");
        if (pointInfo != null) {
            System.out.println("处理 point_info，共 " + pointInfo.size() + " 个节点");

            for (Map<String, Object> point : pointInfo) {
                // Python返回的坐标是米制坐标（x, y）
                Object xObj = point.get("x");
                Object yObj = point.get("y");

                if (xObj instanceof Number && yObj instanceof Number) {
                    double x = ((Number) xObj).doubleValue();
                    double y = ((Number) yObj).doubleValue();

                    // 将米制坐标转换为经纬度（给Vue前端显示）
                    double[] lonlat = metersToLonlat(x, y);

                    // 添加经纬度字段（给Vue前端）
                    point.put("lng", lonlat[0]);
                    point.put("lat", lonlat[1]);

                    System.out.println("  节点 " + point.get("node_id") +
                            ": 米制(" + String.format("%.2f", x) + ", " + String.format("%.2f", y) +
                            ") -> 经纬度(" + String.format("%.6f", lonlat[0]) + ", " + String.format("%.6f", lonlat[1]) + ")");
                } else {
                    System.out.println("  警告: 节点 " + point.get("node_id") + " 没有有效的米制坐标");
                }
            }
        }

        return data;
    }

    /**
     * 运行WSN仿真
     */
    @SuppressWarnings("unchecked")
    public Map<String, Object> runWSNSimulation(Map<String, Object> params) {
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);

        try {
            System.out.println("========== 开始WSN仿真 ==========");

            // 获取当前活跃数据集名称，并根据名称获取对应的传感器和中继数量
            String activeDatasetName = getActiveDatasetName();
            Map<String, Integer> sensorRelayCount;
            if (activeDatasetName != null) {
                sensorRelayCount = getSensorAndRelayCountByDatasetName(activeDatasetName);
            } else {
                System.err.println("警告: 没有活跃的数据集，使用默认值");
                sensorRelayCount = new HashMap<>();
                sensorRelayCount.put("sensor_count", 40);
                sensorRelayCount.put("relay_count", 5);
            }

            int defaultSensorCount = sensorRelayCount.get("sensor_count");
            int defaultRelayCount = sensorRelayCount.get("relay_count");

            Map<String, Object> roadGraphData = loadRoadGraphFromDatabase();
            System.out.println("数据库加载完成，节点数: " + ((List<?>) roadGraphData.get("nodes")).size());
            System.out.println("数据库加载完成，边数: " + ((List<?>) roadGraphData.get("edges")).size());

            Map<String, Object> requestData = new HashMap<>();
            requestData.putAll(roadGraphData);

            if (params != null) {
                System.out.println("传入参数: " + params);
                requestData.putAll(params);
            }

            if (!requestData.containsKey("num_sensors")) {
                requestData.put("num_sensors", defaultSensorCount);
            }
            if (!requestData.containsKey("num_relays")) {
                requestData.put("num_relays", defaultRelayCount);
            }
            if (!requestData.containsKey("seed")) {
                requestData.put("seed", Math.abs(new Random().nextInt()));
            }
            if (!requestData.containsKey("sensor_battery")) {
                requestData.put("sensor_battery", 240000.0);
            }
            if (!requestData.containsKey("relay_battery")) {
                requestData.put("relay_battery", 800000.0);
            }
            if (!requestData.containsKey("station_battery")) {
                requestData.put("station_battery", 1000000.0);
            }

            System.out.println("请求URL: " + wsnApiUrl);
            System.out.println("请求参数: num_sensors=" + requestData.get("num_sensors") +
                    ", num_relays=" + requestData.get("num_relays") +
                    ", seed=" + requestData.get("seed"));

            String jsonPayload = pythonObjectMapper.writeValueAsString(requestData);
            System.out.println("请求JSON长度: " + jsonPayload.length() + " 字符");

            HttpEntity<String> entity = new HttpEntity<>(jsonPayload, headers);

            System.out.println("开始调用Python服务...");
            ResponseEntity<Map> response = restTemplate.postForEntity(wsnApiUrl, entity, Map.class);

            System.out.println("Python服务响应状态码: " + response.getStatusCode());

            Map<String, Object> result = processWSNResponse(response);

            System.out.println("WSN仿真完成，返回 " +
                    ((List<?>) result.get("point_info")).size() + " 个充电节点信息");
            System.out.println("========== WSN仿真结束 ==========");

            return result;

        } catch (Exception e) {
            System.err.println("WSN仿真执行失败: " + e.getMessage());
            e.printStackTrace();
            throw new RuntimeException("WSN仿真执行失败: " + e.getMessage(), e);
        }
    }

    /**
     * 获取完整的道路图数据（用于传递给Python算法）
     */
    public Map<String, Object> getRoadGraphData() {
        return loadRoadGraphFromDatabase();
    }

    @SuppressWarnings("unchecked")
    public Map<String, Object> processAlgorithmResponse(ResponseEntity<Map> response) {
        Map<String, Object> responseBody = response.getBody();

        if (responseBody == null) {
            throw new RuntimeException("算法服务返回空响应");
        }

        String status = (String) responseBody.get("status");
        if (!"success".equals(status)) {
            String message = (String) responseBody.get("message");
            throw new RuntimeException("算法执行失败: " + message);
        }

        Map<String, Object> data = (Map<String, Object>) responseBody.get("data");
        if (data == null) {
            throw new RuntimeException("算法服务返回数据为空");
        }

        // 转换 action_log 中的路径坐标（米制 -> 经纬度）
        List<Map<String, Object>> actionLog = (List<Map<String, Object>>) data.get("action_log");
        if (actionLog != null) {
            System.out.println("转换 action_log 路径坐标，共 " + actionLog.size() + " 步");

            for (Map<String, Object> logEntry : actionLog) {
                List<List<Double>> pathNodes = (List<List<Double>>) logEntry.get("path_nodes");
                if (pathNodes != null && !pathNodes.isEmpty()) {
                    List<List<Double>> convertedPath = new ArrayList<>();
                    for (List<Double> coord : pathNodes) {
                        if (coord.size() >= 2) {
                            double x = coord.get(0);
                            double y = coord.get(1);
                            // 米制坐标转经纬度
                            double[] lonlat = metersToLonlat(x, y);
                            List<Double> lonlatList = Arrays.asList(lonlat[0], lonlat[1]);
                            convertedPath.add(lonlatList);
                        }
                    }
                    logEntry.put("path_nodes", convertedPath);
                }
            }
        }

        return data;
    }
// 在类中添加一个方法，根据数据集名称获取传感器和中继数量
    /**
     * 根据数据集名称获取传感器和中继数量（硬编码配置）
     * @param datasetName 数据集名称
     * @return Map包含sensor_count和relay_count
     */
    private Map<String, Integer> getSensorAndRelayCountByDatasetName(String datasetName) {
        Map<String, Integer> result = new HashMap<>();

        // 根据数据集名称配置传感器和中继数量
        switch (datasetName) {
            case "RR335":
                result.put("sensor_count", 40);
                result.put("relay_count", 5);
                break;
            case "RR508":
                result.put("sensor_count", 50);
                result.put("relay_count", 6);
                break;
            case "RS327":
                result.put("sensor_count", 44);
                result.put("relay_count", 0);
                break;
            case "RS422":
                result.put("sensor_count", 55);
                result.put("relay_count", 0);
                break;
            default:
                // 默认值
                result.put("sensor_count", 40);
                result.put("relay_count", 5);
                System.out.println("未知数据集 [" + datasetName + "]，使用默认值: 传感器=40, 中继=5");
                break;
        }

        System.out.println("数据集 [" + datasetName + "] 配置: 传感器=" + result.get("sensor_count") +
                ", 中继=" + result.get("relay_count"));
        return result;
    }

    /**
     * 获取当前活跃数据集的名称
     * @return 活跃数据集名称，如果没有则返回null
     */
    public String getActiveDatasetName() {
        try {
            List<String> activeDatasets = datasetNameMapper.selectNamesByIsDataset();
            if (activeDatasets != null && !activeDatasets.isEmpty()) {
                return activeDatasets.get(0);
            }
        } catch (Exception e) {
            System.err.println("获取活跃数据集失败: " + e.getMessage());
        }
        return null;
    }
}