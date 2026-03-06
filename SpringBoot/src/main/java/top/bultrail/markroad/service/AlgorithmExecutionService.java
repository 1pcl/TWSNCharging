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
import top.bultrail.markroad.entity.*;
import top.bultrail.markroad.mapper.DatasetNameMapper;
import top.bultrail.markroad.mapper.PointMapper;

import java.util.*;

@Service
public class AlgorithmExecutionService {

    @Value("${python.greedy.url}")
    private String greedyApiUrl;

    @Value("${python.ppo.url}")
    private String ppoApiUrl;

    @Value("${python.aco.url}")
    private String acoApiUrl;

    @Value("${python.npso.url}")
    private String npsoApiUrl;

    private final RestTemplate restTemplate;
    private final ObjectMapper pythonObjectMapper;

    @Autowired
    private PointMapper pointMapper;

    @Autowired
    private DatasetNameMapper datasetNameMapper;

    @Autowired
    private GraphHopperRoutingService graphHopperRoutingService;

    @Autowired
    private PointsService pointsService;

    @Autowired
    public AlgorithmExecutionService(RestTemplate restTemplate,
                                     @Qualifier("pythonObjectMapper") ObjectMapper pythonObjectMapper) {
        this.restTemplate = restTemplate;
        this.pythonObjectMapper = pythonObjectMapper;
    }

    /**
     * 根据全局整数ID判断节点类型
     * 逻辑参考 WSNSimulation 的偏移量计算
     */
    private String determineNodeType(Integer nodeId) {
        int stationCount = pointsService.readStation2().size();
        int gatewayCount = pointsService.readGateWay2().size();

        if (nodeId < stationCount) {
            return "station";
        } else if (nodeId < (stationCount + gatewayCount)) {
            return "gateWay";
        } else {
            return "sensor";
        }
    }

    /**
     * 根据整数ID和类型获取坐标对象
     */
    private Points getPointEntity(Integer nodeId) {
        int stationCount = pointsService.readStation2().size();
        int gatewayCount = pointsService.readGateWay2().size();

        if (nodeId < stationCount) {
            // station: id = nodeId + 1
            return pointMapper.selectStationById(nodeId + 1);
        } else if (nodeId < (stationCount + gatewayCount)) {
            // gateway: id = nodeId - stationCount + 1
            return pointMapper.selectGatewayById(nodeId - stationCount + 1);
        } else {
            // sensor: id = nodeId - stationCount - gatewayCount + 1
            return pointMapper.selectSensorById(nodeId - stationCount - gatewayCount + 1);
        }
    }

    /**
     * 处理算法返回结果，丰富路径经纬度信息
     */
    private Map<String, Object> processAlgorithmResponse(ResponseEntity<Map> response) {
        if (response.getStatusCode().is2xxSuccessful() && response.getBody() != null) {
            if ("success".equals(response.getBody().get("status"))) {
                Map<String, Object> dataMap = (Map<String, Object>) response.getBody().get("data");
                List<Map<String, Object>> rawActionLog = (List<Map<String, Object>>) dataMap.get("action_log");
                Map<String, Object> metrics = (Map<String, Object>) dataMap.get("metrics");

                List<Map<String, Object>> finalActionLog = new ArrayList<>();

                for (Map<String, Object> logEntry : rawActionLog) {
                    // Python 返回的 ID 现在是 Integer
                    Integer fromId = (Integer) logEntry.get("uav_from");
                    Integer toId = (Integer) logEntry.get("uav_to");

                    // 1. 获取起止点的实体信息
                    Points startPoint = getPointEntity(fromId);
                    Points endPoint = getPointEntity(toId);

                    List<List<Double>> coordinateSequence = new ArrayList<>();

                    if (startPoint != null && endPoint != null) {
                        // 2. 调用 GraphHopper 服务获取真实的道路路径序列
                        ShortestPathRequest pathRequest = new ShortestPathRequest();
                        pathRequest.setStartLng(startPoint.getLng());
                        pathRequest.setStartLat(startPoint.getLat());
                        pathRequest.setEndLng(endPoint.getLng());
                        pathRequest.setEndLat(endPoint.getLat());

                        ShortestPathResponse pathRes = graphHopperRoutingService.findShortestPath(pathRequest);

                        if (pathRes.isSuccess()) {
                            coordinateSequence = pathRes.getPath(); // 获取 [ [lng, lat], ... ] 格式
                            // 总是在开头添加起点
                            coordinateSequence.add(0, Arrays.asList(startPoint.getLng(), startPoint.getLat()));

                            // 总是在结尾添加终点
                            coordinateSequence.add(Arrays.asList(endPoint.getLng(), endPoint.getLat()));
                        } else {
                            // 如果路由失败，降级为直线连接
                            coordinateSequence.add(Arrays.asList(startPoint.getLng(), startPoint.getLat()));
                            coordinateSequence.add(Arrays.asList(endPoint.getLng(), endPoint.getLat()));
                        }
                    }

                    // 3. 构造增强后的日志
                    Map<String, Object> enrichedLogEntry = new HashMap<>(logEntry);
                    enrichedLogEntry.put("path_nodes", coordinateSequence);
                    enrichedLogEntry.put("destination_type", determineNodeType(toId));

                    finalActionLog.add(enrichedLogEntry);
                }

                Map<String, Object> finalResult = new HashMap<>();
                finalResult.put("action_log", finalActionLog);
                finalResult.put("metrics", metrics);
                return finalResult;
            } else {
                throw new RuntimeException("Python API Error: " + response.getBody().get("message"));
            }
        }
        throw new RuntimeException("External service communication failed.");
    }

    public Map<String, Object> runGreedyAlgorithm(Map<String, Object> simulationData) {
        return executeAlgorithm(greedyApiUrl, simulationData);
    }

    public Map<String, Object> runPPOSimulation(Map<String, Object> simulationData) {
        // 添加数据集名称逻辑
        String datasetName = getDatasetName();
        simulationData.put("dataset_name", datasetName);
        return executeAlgorithm(ppoApiUrl, simulationData);
    }

    public Map<String, Object> runACOSimulation(Map<String, Object> simulationData) {
        return executeAlgorithm(acoApiUrl, simulationData);
    }

    public Map<String, Object> runNPSOSimulation(Map<String, Object> simulationData) {
        return executeAlgorithm(npsoApiUrl, simulationData);
    }

    /**
     * 通用的算法执行请求封装
     */
    private Map<String, Object> executeAlgorithm(String url, Map<String, Object> requestData) {
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);

        try {
            String jsonPayload = pythonObjectMapper.writeValueAsString(requestData);
            HttpEntity<String> entity = new HttpEntity<>(jsonPayload, headers);

            ResponseEntity<Map> response = restTemplate.postForEntity(url, entity, Map.class);
            return processAlgorithmResponse(response);

        } catch (Exception e) {
            throw new RuntimeException("Algorithm execution failed: " + e.getMessage(), e);
        }
    }

    private String getDatasetName() {
        try {
            List<String> names = datasetNameMapper.selectNamesByIsDataset();
            return (names != null && !names.isEmpty()) ? names.get(0) : "data1";
        } catch (Exception e) {
            return "data1";
        }
    }
}