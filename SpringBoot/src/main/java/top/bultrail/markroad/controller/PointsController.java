package top.bultrail.markroad.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.ResponseBody;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

import top.bultrail.markroad.service.ChargingGraphService;
import top.bultrail.markroad.service.PointsService;
import top.bultrail.markroad.entity.ResultEntity;

import javax.servlet.http.HttpServletResponse;


@Controller
public class PointsController {
    @Autowired
    PointsService pointsService;

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
        List<List<String>> gateWayList = pointsService.readGateWay2();
        return ResultEntity.sucessWithData(gateWayList);
    }

    @RequestMapping(value = {"/loadStation", "/api/loadStation"})
    @ResponseBody
    public ResultEntity<List<List<String>>> stationupload() {
        List<List<String>> stationList = pointsService.readStation2();
        return ResultEntity.sucessWithData(stationList);
    }

    // ============ 新增下载接口 ============

    /**
     * 下载传感器数据为JSON文件
     */
    @GetMapping("/api/downloadSensors")
    public void downloadSensors(HttpServletResponse response) {
        try {
            String jsonData = pointsService.convertSensorsToJson();
            setupJsonDownloadResponse(response, "sensors_data.json");
            response.getWriter().write(jsonData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "传感器数据下载失败");
        }
    }


    /**
     * 下载充电点数据为JSON文件
     */
    @GetMapping("/api/downloadgateWays")
    public void downloadgateWay(HttpServletResponse response) {
        try {
            String jsonData = pointsService.convertGateWaysToJson();
            setupJsonDownloadResponse(response, "gateWays_data.json");
            response.getWriter().write(jsonData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "充电站数据下载失败");
        }
    }

    /**
     * 下载服务站数据为JSON文件
     */
    @GetMapping("/api/downloadStations")
    public void downloadStations(HttpServletResponse response) {
        try {
            String jsonData = pointsService.convertStationsToJson();
            setupJsonDownloadResponse(response, "stations_data.json");
            response.getWriter().write(jsonData);
            response.getWriter().flush();
        } catch (Exception e) {
            e.printStackTrace();
            handleDownloadError(response, "服务站数据下载失败");
        }
    }


    @Autowired
    private ChargingGraphService chargingGraphService;
    /**
     * 下载充电调度规划数据（流式版本）
     */
    @GetMapping("/api/downloadChargingScheduling")
    public void downloadChargingScheduling(HttpServletResponse response) {
        try {
            // 先设置响应头（非常重要：在开始写入之前设置）
            String fileName = "chargingScheduling_data.json";
            response.setContentType("application/json");
            response.setCharacterEncoding("UTF-8");
            response.setHeader("Content-Disposition", "attachment; filename=\"" + fileName + "\"");

            // 设置缓冲区大小
            response.setBufferSize(1024 * 32); // 32KB缓冲区

            // 使用流式写入
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
