package top.bultrail.markroad.controller;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.ResponseBody;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;

import top.bultrail.markroad.entity.Edges;
import top.bultrail.markroad.entity.Points;
import top.bultrail.markroad.mapper.EdgesMapper;
import top.bultrail.markroad.mapper.PointMapper;
import top.bultrail.markroad.service.ChargingGraphService;
import top.bultrail.markroad.service.PointsService;
import top.bultrail.markroad.entity.ResultEntity;

import javax.servlet.http.HttpServletResponse;


@Controller
public class PointsController {
    @Autowired
    PointsService pointsService;

    @Autowired
    EdgesMapper edgesMapper;

    @Autowired
    PointMapper pointMapper;

    //上传传感器数据给前端
    @RequestMapping(value = {"/sup", "/api/sup"})
    @ResponseBody
    public ResultEntity<List<List<String>>> sensorUpload() {
        List<List<String>> sensorList = pointsService.readSensor2();
        return ResultEntity.sucessWithData(sensorList);
    }


    //上传网关数据给前端
    @RequestMapping(value = {"/cup", "/api/cup"})
    @ResponseBody
    public ResultEntity<List<List<String>>> gateWayUpload() {
        List<List<String>> gateWayList = pointsService.readRealy2();
        return ResultEntity.sucessWithData(gateWayList);
    }

    //上传服务站数据给前端
    @RequestMapping(value = {"/loadStation", "/api/loadStation"})
    @ResponseBody
    public ResultEntity<List<List<String>>> stationupload() {
        List<List<String>> stationList = pointsService.readStation2();
        return ResultEntity.sucessWithData(stationList);
    }

    // 加载道路线（边）数据
    @RequestMapping(value = {"/loadRoads", "/api/loadRoads"})
    @ResponseBody
    public ResultEntity<List<Map<String, Object>>> loadRoads() {
        try {
            List<Edges> edges = edgesMapper.selectAllEdges();
            List<Points> allNodes = pointsService.getAllNodes();

            // 创建节点ID到坐标的映射
            Map<String, Points> nodeMap = new HashMap<>();
            for (Points node : allNodes) {
                nodeMap.put(node.getId(), node);
            }

            // 构建道路线数据
            List<Map<String, Object>> roadList = new ArrayList<>();
            for (Edges edge : edges) {
                Points fromNode = nodeMap.get(edge.getFrom_node());
                Points toNode = nodeMap.get(edge.getTo_node());

                if (fromNode != null && toNode != null) {
                    Map<String, Object> road = new HashMap<>();
                    road.put("startPoint", new double[]{fromNode.getLng(), fromNode.getLat()});
                    road.put("endPoint", new double[]{toNode.getLng(), toNode.getLat()});
                    road.put("lengthM", edge.getLength_m());
                    roadList.add(road);
                }
            }

            System.out.println("加载道路线: " + roadList.size() + " 条");
            return ResultEntity.sucessWithData(roadList);
        } catch (Exception e) {
            e.printStackTrace();
            return ResultEntity.failedWithMessage(e.getMessage());
        }
    }

    // 加载路口节点数据
    @RequestMapping(value = {"/loadIntersections", "/api/loadIntersections"})
    @ResponseBody
    public ResultEntity<List<List<String>>> loadIntersections() {
        try {
            List<List<String>> intersections = pointsService.readIntersection2();
            System.out.println("加载路口节点: " + intersections.size() + " 个");
            return ResultEntity.sucessWithData(intersections);
        } catch (Exception e) {
            e.printStackTrace();
            return ResultEntity.failedWithMessage(e.getMessage());
        }
    }


    /**
     * 下载传感器数据为JSON文件
     */
    @GetMapping("/api/downloadSensors")
    public void downloadSensors(HttpServletResponse response) {
        try {
            List<List<String>> sensorData = pointsService.readSensor2();

            Map<String, Object> exportData = new LinkedHashMap<>();
            exportData.put("type", "sensors");
            exportData.put("exportTime", new Date());
            exportData.put("data", sensorData);

            setupJsonDownloadResponse(response, "sensors_data.json");
            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            mapper.writeValue(response.getWriter(), exportData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "传感器数据下载失败");
        }
    }

    /**
     * 下载中继数据为JSON文件
     */
    @GetMapping("/api/downloadrelays")
    public void downloadRelays(HttpServletResponse response) {
        try {
            List<List<String>> relayData = pointsService.readRealy2();

            Map<String, Object> exportData = new LinkedHashMap<>();
            exportData.put("type", "relay");
            exportData.put("exportTime", new Date());
            exportData.put("data", relayData);

            setupJsonDownloadResponse(response, "relays_data.json");
            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            mapper.writeValue(response.getWriter(), exportData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "中继数据下载失败");
        }
    }

    /**
     * 下载服务站数据为JSON文件
     */
    @GetMapping("/api/downloadStations")
    public void downloadStations(HttpServletResponse response) {
        try {
            List<List<String>> stationData = pointsService.readStation2();

            Map<String, Object> exportData = new LinkedHashMap<>();
            exportData.put("type", "stations");
            exportData.put("exportTime", new Date());
            exportData.put("data", stationData);

            setupJsonDownloadResponse(response, "stations_data.json");
            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            mapper.writeValue(response.getWriter(), exportData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "服务站数据下载失败");
        }
    }

    /**
     * 下载路口节点数据为JSON文件
     */
    @GetMapping("/api/downloadIntersections")
    public void downloadIntersections(HttpServletResponse response) {
        try {
            List<Points> intersections = pointMapper.selectAllIntersection();

            List<Map<String, Object>> nodes = new ArrayList<>();
            for (Points inter : intersections) {
                Map<String, Object> node = new LinkedHashMap<>();
                node.put("id", inter.getId());
                node.put("lng", inter.getLng());
                node.put("lat", inter.getLat());
                nodes.add(node);
            }

            Map<String, Object> exportData = new LinkedHashMap<>();
            exportData.put("type", "intersections");
            exportData.put("exportTime", new Date());
            exportData.put("count", nodes.size());
            exportData.put("data", nodes);

            setupJsonDownloadResponse(response, "intersections_data.json");
            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            mapper.writeValue(response.getWriter(), exportData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "路口节点下载失败");
        }
    }

    /**
     * 下载道路线数据为JSON文件（边数据，包含起点ID和终点ID）
     */
    @GetMapping("/api/downloadRoads")
    public void downloadRoads(HttpServletResponse response) {
        try {
            List<Edges> edges = edgesMapper.selectAllEdges();

            // 构建道路线数据（边格式）
            List<Map<String, Object>> roadList = new ArrayList<>();
            for (Edges edge : edges) {
                Map<String, Object> road = new LinkedHashMap<>();
                road.put("id", edge.getId());
                road.put("from_node", edge.getFrom_node());
                road.put("to_node", edge.getTo_node());
                road.put("length_m", edge.getLength_m());
                roadList.add(road);
            }

            Map<String, Object> exportData = new LinkedHashMap<>();
            exportData.put("type", "roads");
            exportData.put("exportTime", new Date());
            exportData.put("count", roadList.size());
            exportData.put("data", roadList);

            setupJsonDownloadResponse(response, "roads_data.json");
            ObjectMapper mapper = new ObjectMapper();
            mapper.enable(SerializationFeature.INDENT_OUTPUT);
            mapper.writeValue(response.getWriter(), exportData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "道路线下载失败");
        }
    }

    @Autowired
    private ChargingGraphService chargingGraphService;

    /**
     * 下载完整的充电调度规划数据（类似 road_graph_with_nodes.json 格式）
     */
    @GetMapping("/api/downloadChargingScheduling")
    public void downloadChargingScheduling(HttpServletResponse response) {
        try {
            // 使用流式写入完整数据
            chargingGraphService.streamInfrastructureDataToResponse(response);
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "完整数据下载失败: " + e.getMessage());
        }
    }



    /**
     * 设置JSON下载响应头
     */
    private void setupJsonDownloadResponse(HttpServletResponse response, String filename) {
        response.setContentType("application/json;charset=UTF-8");
        response.setCharacterEncoding("UTF-8");
        response.setHeader("Content-Disposition",
                "attachment; filename=\"" + filename + "\"");
        response.setHeader("Access-Control-Expose-Headers", "Content-Disposition");
    }

    /**
     * 处理下载错误
     */
    private void handleDownloadError(HttpServletResponse response, String message) {
        try {
            response.sendError(HttpServletResponse.SC_INTERNAL_SERVER_ERROR, message);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

}
