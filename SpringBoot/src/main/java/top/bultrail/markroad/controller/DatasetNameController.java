package top.bultrail.markroad.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;
import top.bultrail.markroad.entity.*;

import top.bultrail.markroad.service.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.servlet.http.HttpSession;

@Controller
public class DatasetNameController {

    @Autowired
    DatasetNameService transformService;

    // 一键保存
    @ResponseBody
    @RequestMapping(value = {"/hhquicksaveNew", "/api/hhquicksaveNew"}, method = RequestMethod.POST, consumes = MediaType.APPLICATION_JSON_VALUE)
    public ResponseEntity<ResultEntity<String>> quicksave_new(@RequestBody QuickSave quick_save) {
        // 将数据保存在数据库
        try {
            transformService.quicksaveDB(quick_save);
            System.out.println("Data Save Success!");
            return ResponseEntity.ok(ResultEntity.successWithoutData());
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failed(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 保存数据集
    @ResponseBody
    @RequestMapping(value = {"/api/saveDataset"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<String>> saveDataset(@RequestBody Map<String, Object> requestBody) {
        String setName = (String) requestBody.get("datasetName");
        List<Double> currentLocation = (List<Double>) requestBody.get("current_location");
        try {
            transformService.saveDataset(setName, currentLocation);
            System.out.println("Dataset saved successfully");
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failed(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
        return ResponseEntity.ok(ResultEntity.successWithoutData());
    }

    // 删除数据集
    @ResponseBody
    @RequestMapping(value = {"/api/deleteDataset"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<String>> deleteDataset(@RequestParam(value = "datasetName") String setName) {
        try {
            transformService.deleteDataset(setName);
            System.out.println("Dataset deleted successfully");
            return ResponseEntity.ok(ResultEntity.successWithoutData());
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failedWithMessage(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 获取数据集信息
    @ResponseBody
    @RequestMapping(value = {"/api/searchDatasetInfo"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<List<DatasetInfo>>> searchDatasetInfo() {
        try {
            List<DatasetInfo> datasetInfos;
            datasetInfos = transformService.searchDatasetInfo();
            System.out.println("Search Datasets Information Successfully");
            return ResponseEntity.ok(ResultEntity.successWithDatasetInfo(datasetInfos, null));
        } catch (Exception e) {
            ResultEntity<List<DatasetInfo>> errorResponse = ResultEntity.failedWithMessage(e.getMessage());
            return new ResponseEntity<>(errorResponse, HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 对数据库表格进行重命名
    @ResponseBody
    @RequestMapping(value = {"/api/renameDataset"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<String>> renameDataset(@RequestBody RenameDataset request) {
        try {
            transformService.renameDataset(request.getOldName(), request.getNewName());
            System.out.println("Dataset renamed successfully");
            return ResponseEntity.ok(ResultEntity.successWithoutData());
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failedWithMessage(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }


    //数据集加载
    @ResponseBody
    @RequestMapping(value = {"/api/datasetLoad"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<List<Double>>> datasetLoad(@RequestParam(value = "datasetName") String setName) {
        try {
            List<Double> location = transformService.datasetLoad(setName);
            System.out.println("Dataset Load Success");
            return ResponseEntity.ok(ResultEntity.successWithDataDouble(location, null));
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failedWithMessage(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    /*------------------------------------计算部分--------------------------------------------------*/
    //https://docs.graphhopper.com/openapi

    @Autowired
    private GraphHopperRoutingService graphHopperRoutingService;


    /**
     * 处理前端 /shortestpath 的 POST 请求
     * 映射到 /api/map/shortestpath
     * 不用，留着参考的
     */
    @ResponseBody
    @RequestMapping(value = {"/api/shortestpath"}, method = RequestMethod.POST)
    public ShortestPathResponse getShortestPath(@RequestBody ShortestPathRequest request) {
        return graphHopperRoutingService.findShortestPath(request);
    }

    @Autowired
    private WSNSimulation wsnSimulation;

//    @Autowired
//    private FrontWSNSimulation frontWSNSimulation;

    /**
     * TWSN models 模拟调用接口
     * 触发服务层进行 WSN 模拟计算（如识别待充电节点和路径规划），
     * 并返回模拟结果（包括过滤后的点信息 point_info 和图结构 graph_G）。
     */
    @RequestMapping(value = {"/runTWSNSimulation", "/api/runTWSNSimulation"})
    @ResponseBody
    // 1. 将返回类型 ResultEntity<Map<String, Object>>
    public ResultEntity<Map<String, Object>> runTWSNSimulation(HttpSession session) {
        // 2. 直接调用 WSNSimulation 服务方法
//        Map<String, Object> simulationResult = frontWSNSimulation.runWSNSimulation();
        Map<String, Object> simulationResult = wsnSimulation.runWSNSimulation();
        // 将结果存入当前用户的 Session
        session.setAttribute("LAST_WSN_DATA", simulationResult);
        // 3. 使用 ResultEntity.sucessWithData 包装结果并返回
        return ResultEntity.sucessWithData(simulationResult);
    }

    @Autowired
    private AlgorithmExecutionService algorithmExecutionService;

    /**
     * 调用 Python 贪心算法服务
     * 映射到 /api/runGreedyAlgorithm
     */
    @RequestMapping(value = {"/runGreedyAlgorithm", "/api/runGreedyAlgorithm"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runGreedyAlgorithm(HttpSession session) {
        // 1. 调用 WSN Simulation 获取初始数据 (Graph G 和 Point Info)
//        Map<String, Object> simulationData = wsnSimulation.runWSNSimulation();
        // 1. 从 Session 中取出上一次的数据，强制转换为 Map
        Map<String, Object> simulationData = (Map<String, Object>) session.getAttribute("LAST_WSN_DATA");

        // 安全检查：如果 Session 过期或用户没点仿真就点算法
        if (simulationData == null) {
            simulationData = wsnSimulation.runWSNSimulation();
            session.setAttribute("LAST_WSN_DATA", simulationData);
        }
        // 2. 将数据传递给 Service，由 Service 发送给 Python API
        Map<String, Object> algorithmResult = algorithmExecutionService.runGreedyAlgorithm(simulationData);
        // 3. 合并两个结果，使前端能同时拿到节点状态和规划路径
        Map<String, Object> combinedResult = new HashMap<>();
        combinedResult.putAll(simulationData); // 放入仿真数据 (point_info 就在这里)
        combinedResult.putAll(algorithmResult); // 放入路径数据 (action_log 在这里)

        // 成功返回合并后的结果
        return ResponseEntity.ok(ResultEntity.sucessWithData(combinedResult));
    }

    @RequestMapping(value = {"/runPPOSimulation", "/api/runPPOSimulation"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runPPOSimulation(HttpSession session) {
        // 1. 调用 WSN Simulation 获取初始数据 (Graph G 和 Point Info)
//        Map<String, Object> simulationData = wsnSimulation.runWSNSimulation();
        // 1. 从 Session 中取出上一次的数据，强制转换为 Map
        Map<String, Object> simulationData = (Map<String, Object>) session.getAttribute("LAST_WSN_DATA");

        // 安全检查：如果 Session 过期或用户没点仿真就点算法
        if (simulationData == null) {
            simulationData = wsnSimulation.runWSNSimulation();
            session.setAttribute("LAST_WSN_DATA", simulationData);
        }
        // 2. 将数据传递给 Service，由 Service 发送给 Python API
        Map<String, Object> algorithmResult = algorithmExecutionService.runPPOSimulation(simulationData);
        // 3. 合并两个结果，使前端能同时拿到节点状态和规划路径
        Map<String, Object> combinedResult = new HashMap<>();
        combinedResult.putAll(simulationData); // 放入仿真数据 (point_info 就在这里)
        combinedResult.putAll(algorithmResult); // 放入路径数据 (action_log 在这里)

        // 成功返回合并后的结果
        return ResponseEntity.ok(ResultEntity.sucessWithData(combinedResult));
    }

    /**
     * 调用 Python ACO算法服务
     * 映射到 /api/runACOSimulation
     */
    @RequestMapping(value = {"/runACOSimulation", "/api/runACOSimulation"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runACOSimulation(HttpSession session,
                                                                              @RequestBody(required = false) Map<String, Object> acoParams) {
        // 1. 从 Session 中取出上一次的数据，强制转换为 Map
        Map<String, Object> simulationData = (Map<String, Object>) session.getAttribute("LAST_WSN_DATA");

        // 安全检查：如果 Session 过期或用户没点仿真就点算法
        if (simulationData == null) {
            simulationData = wsnSimulation.runWSNSimulation();
            session.setAttribute("LAST_WSN_DATA", simulationData);
        }

        // 如果有传入ACO参数，添加到数据中
        if (acoParams != null && !acoParams.isEmpty()) {
            simulationData.put("aco_params", acoParams);
        }

        // 2. 将数据传递给 Service，由 Service 发送给 Python API
        Map<String, Object> algorithmResult = algorithmExecutionService.runACOSimulation(simulationData);
        // 3. 合并两个结果，使前端能同时拿到节点状态和规划路径
        Map<String, Object> combinedResult = new HashMap<>();
        combinedResult.putAll(simulationData); // 放入仿真数据 (point_info 就在这里)
        combinedResult.putAll(algorithmResult); // 放入路径数据 (action_log 在这里)

        // 成功返回合并后的结果
        return ResponseEntity.ok(ResultEntity.sucessWithData(combinedResult));
    }

    /**
     * 调用 Python NPSO算法服务
     * 映射到 /api/runNPSOSimulation
     */
    @RequestMapping(value = {"/runNPSOSimulation", "/api/runNPSOSimulation"})
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runNPSOSimulation(HttpSession session,
                                                                               @RequestBody(required = false) Map<String, Object> npsoParams) {
        // 1. 从 Session 中取出上一次的数据，强制转换为 Map
        Map<String, Object> simulationData = (Map<String, Object>) session.getAttribute("LAST_WSN_DATA");

        // 安全检查：如果 Session 过期或用户没点仿真就点算法
        if (simulationData == null) {
            simulationData = wsnSimulation.runWSNSimulation();
            session.setAttribute("LAST_WSN_DATA", simulationData);
        }

        // 如果有传入NPSO参数，添加到数据中
        if (npsoParams != null && !npsoParams.isEmpty()) {
            simulationData.put("npso_params", npsoParams);
        }

        // 2. 将数据传递给 Service，由 Service 发送给 Python API
        Map<String, Object> algorithmResult = algorithmExecutionService.runNPSOSimulation(simulationData);
        // 3. 合并两个结果，使前端能同时拿到节点状态和规划路径
        Map<String, Object> combinedResult = new HashMap<>();
        combinedResult.putAll(simulationData); // 放入仿真数据 (point_info 就在这里)
        combinedResult.putAll(algorithmResult); // 放入路径数据 (action_log 在这里)

        // 成功返回合并后的结果
        return ResponseEntity.ok(ResultEntity.sucessWithData(combinedResult));
    }


}
