package top.bultrail.markroad.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;
import top.bultrail.markroad.entity.ResultEntity;
import top.bultrail.markroad.service.AlgorithmExecutionService;
import top.bultrail.markroad.service.WSNSimulationService;

import javax.servlet.http.HttpSession;
import java.util.HashMap;
import java.util.Map;

@Controller
public class AlgorithmController {

    @Autowired
    private AlgorithmExecutionService algorithmExecutionService;

    @Autowired
    private WSNSimulationService wsnSimulationService;

    /**
     * 获取WSN仿真数据（如果不存在则运行）
     */
    private Map<String, Object> getOrCreateWSNData(HttpSession session) {
        Map<String, Object> simulationData = (Map<String, Object>) session.getAttribute("LAST_WSN_DATA");
        if (simulationData == null) {
            simulationData = wsnSimulationService.runWSNSimulation(null);
            session.setAttribute("LAST_WSN_DATA", simulationData);
        }
        return simulationData;
    }

    /**
     * 合并结果
     */
    private Map<String, Object> combineResults(Map<String, Object> simulationData, Map<String, Object> algorithmResult) {
        Map<String, Object> combinedResult = new HashMap<>();
        combinedResult.putAll(simulationData);
        combinedResult.putAll(algorithmResult);
        return combinedResult;
    }

    // ==================== 贪心算法 API ====================

    @RequestMapping(value = {"/runGreedyAlgorithm", "/api/runGreedyAlgorithm"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runGreedyAlgorithm(HttpSession session) {
        Map<String, Object> simulationData = getOrCreateWSNData(session);
        Map<String, Object> algorithmResult = algorithmExecutionService.runGreedyAlgorithm(simulationData);
        return ResponseEntity.ok(ResultEntity.sucessWithData(combineResults(simulationData, algorithmResult)));
    }


    // ==================== RAWCS算法 API ====================
    @RequestMapping(value = {"/runRAWCSAlgorithm", "/api/runRAWCSAlgorithm"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runRAWCSAlgorithm(HttpSession session) {
        Map<String, Object> simulationData = getOrCreateWSNData(session);
        Map<String, Object> algorithmResult = algorithmExecutionService.runRAWCSAlgorithm(simulationData);
        return ResponseEntity.ok(ResultEntity.sucessWithData(combineResults(simulationData, algorithmResult)));
    }

    // ==================== ACO算法 API ====================
    @RequestMapping(value = {"/runACOAlgorithm", "/api/runACOAlgorithm"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runACOAlgorithm(HttpSession session) {
        Map<String, Object> simulationData = getOrCreateWSNData(session);
        Map<String, Object> algorithmResult = algorithmExecutionService.runACOAlgorithm(simulationData);
        return ResponseEntity.ok(ResultEntity.sucessWithData(combineResults(simulationData, algorithmResult)));
    }

    // ==================== NSGA算法 API ====================

    @RequestMapping(value = {"/runNSGAAlgorithm", "/api/runNSGAAlgorithm"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runNSGAAlgorithm(
            HttpSession session, @RequestBody(required = false) Map<String, Object> NSGAParams) {
        Map<String, Object> simulationData = getOrCreateWSNData(session);
        if (NSGAParams != null && !NSGAParams.isEmpty()) {
            simulationData.put("NSGAParams", NSGAParams);
        }
        Map<String, Object> algorithmResult = algorithmExecutionService.runNSGAAlgorithm(simulationData);
        return ResponseEntity.ok(ResultEntity.sucessWithData(combineResults(simulationData, algorithmResult)));
    }


}