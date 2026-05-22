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
import top.bultrail.markroad.mapper.DatasetNameMapper;
import top.bultrail.markroad.mapper.PointMapper;

import java.util.*;

@Service
public class AlgorithmExecutionService {

    // 贪心算法
    @Value("${python.greedy.url}")
    private String greedyApiUrl;

    // PPO算法
    @Value("${python.RAWCS.url}")
    private String RAWCSApiUrl;


    // ACO算法
    @Value("${python.aco.url}")
    private String acoApiUrl;

    // NPSO算法
    @Value("${python.nsga.url}")
    private String NSGAApiUrl;

    private final RestTemplate restTemplate;
    private final ObjectMapper pythonObjectMapper;

    @Autowired
    private PointMapper pointMapper;

    @Autowired
    private DatasetNameMapper datasetNameMapper;

    @Autowired
    private PointsService pointsService;

    @Autowired
    private WSNSimulationService wsnSimulationService;

    @Autowired
    public AlgorithmExecutionService(RestTemplate restTemplate,
                                     @Qualifier("pythonObjectMapper") ObjectMapper pythonObjectMapper) {
        this.restTemplate = restTemplate;
        this.pythonObjectMapper = pythonObjectMapper;
    }

    /**
     * 处理算法返回结果
     */
    @SuppressWarnings("unchecked")
    private Map<String, Object> processAlgorithmResponse(ResponseEntity<Map> response) {
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

        return data;
    }

    private Map<String, Object> executeAlgorithm(String url, Map<String, Object> requestData) {
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);

        try {
            String jsonPayload = pythonObjectMapper.writeValueAsString(requestData);
            System.out.println("请求URL: " + url);
            System.out.println("请求数据长度: " + jsonPayload.length());

            HttpEntity<String> entity = new HttpEntity<>(jsonPayload, headers);
            ResponseEntity<Map> response = restTemplate.postForEntity(url, entity, Map.class);
            System.out.println("算法服务响应状态码: " + response.getStatusCode());

            // 使用 WSNSimulationService 的方法处理响应（包含坐标转换）
            return wsnSimulationService.processAlgorithmResponse(response);

        } catch (Exception e) {
            throw new RuntimeException("Algorithm execution failed: " + e.getMessage(), e);
        }
    }

    // ==================== 贪心算法 ====================
    public Map<String, Object> runGreedyAlgorithm(Map<String, Object> simulationData) {
        String datasetName = getDatasetName();
        simulationData.put("dataset_name", datasetName);

        // 添加完整的道路网络数据
        Map<String, Object> roadGraphData = wsnSimulationService.getRoadGraphData();
        simulationData.put("road_graph_data", roadGraphData);

        return executeAlgorithm(greedyApiUrl, simulationData);
    }

    // ==================== RAWCS算法 ====================
    public Map<String, Object> runRAWCSAlgorithm(Map<String, Object> simulationData) {
        String datasetName = getDatasetName();
        simulationData.put("dataset_name", datasetName);
        // 添加完整的道路网络数据（与贪心算法一致）
        Map<String, Object> roadGraphData = wsnSimulationService.getRoadGraphData();
        simulationData.put("road_graph_data", roadGraphData);
        return executeAlgorithm(RAWCSApiUrl, simulationData);
    }


    // ==================== ACO算法 ====================
    public Map<String, Object> runACOAlgorithm(Map<String, Object> simulationData) {
        String datasetName = getDatasetName();
        simulationData.put("dataset_name", datasetName);

        // 添加完整的道路网络数据（与贪心算法一致）
        Map<String, Object> roadGraphData = wsnSimulationService.getRoadGraphData();
        simulationData.put("road_graph_data", roadGraphData);

        return executeAlgorithm(acoApiUrl, simulationData);
    }


    // ==================== NPSO算法 ====================
    public Map<String, Object> runNSGAAlgorithm(Map<String, Object> simulationData) {
        String datasetName = getDatasetName();
        simulationData.put("dataset_name", datasetName);
        // 添加完整的道路网络数据
        Map<String, Object> roadGraphData = wsnSimulationService.getRoadGraphData();
        simulationData.put("road_graph_data", roadGraphData);
        return executeAlgorithm(NSGAApiUrl, simulationData);
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