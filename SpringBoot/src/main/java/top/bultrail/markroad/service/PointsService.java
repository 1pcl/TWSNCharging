package top.bultrail.markroad.service;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import top.bultrail.markroad.entity.*;
import top.bultrail.markroad.mapper.*;

import java.util.*;

import top.bultrail.markroad.mapper.PointMapper;

@Service
public class PointsService {

    @Autowired
    private PointMapper pointMapper;

    @Autowired
    private DistanceMapper distanceMapper;

    @Autowired
    private ChargingGraphService chargingGraphService;

    @Autowired
    private GraphHopperRoutingService graphHopperRoutingService;

    @Transactional
    public void quick_write_new2(String[][][] gateWay_points, String[][][] sensor_array, String[][][] station_array) {


        // 处理 sensor_array
        String tableName = "sensor";
        // 搜索半径（度），约50米范围
//        double searchRadius = 0.0005;
        for (int i = 0; i < sensor_array.length; i++) {
            for (int j = 0; j < sensor_array[i].length; j++) {
                double lng = Double.parseDouble(sensor_array[i][j][0]);
                double lat = Double.parseDouble(sensor_array[i][j][1]);
                double distanceInLoad = graphHopperRoutingService.getHorizontalDistanceToRoad(lat, lng);
                insertPoint(tableName,lng,lat,distanceInLoad);
            }
        }


        tableName = "gateWay";
        for (int i = 0; i < gateWay_points.length; i++) {
            for (int j = 0; j < gateWay_points[i].length; j++) {
                double lng = Double.parseDouble(gateWay_points[i][j][0]);
                double lat = Double.parseDouble(gateWay_points[i][j][1]);
                double distanceInLoad = graphHopperRoutingService.getHorizontalDistanceToRoad(lat, lng);
                insertPoint(tableName,lng,lat,distanceInLoad);
            }
        }

        tableName = "station";
        for (int i = 0; i < station_array.length; i++) {
            for (int j = 0; j < station_array[i].length; j++) {
                double lng = Double.parseDouble(station_array[i][j][0]);
                double lat = Double.parseDouble(station_array[i][j][1]);
                double distanceInLoad = graphHopperRoutingService.getHorizontalDistanceToRoad(lat, lng);
                insertPoint(tableName,lng,lat,distanceInLoad);
            }
        }

        chargingGraphService.calculateAndSaveInfrastructureData();


    }


    @Transactional
    public void clear2() {
        pointMapper.truncateSensor();
        pointMapper.truncateGateWays();
        pointMapper.truncateStation();
        distanceMapper.truncateDistance();
    }


    // 读取senor
    public List<List<String>> readSensor2() {
        List<Points> points = pointMapper.selectAllSensors();
        List<List<String>> bk = new ArrayList<>();
        for (Points p : points) {
            List<String> lis = new ArrayList<>();
            lis.add(Integer.toString(p.getId()));
            lis.add(Double.toString(p.getDistanceInRoad()));
            lis.add(Double.toString(p.getLng()));
            lis.add(Double.toString(p.getLat()));
            bk.add(lis);
        }
        return bk;
    }


    @Transactional
    public void insertPoint(String tableName, double lng, double lat, double distanceInLoad)
    {
        Points points=new Points();
//        RouteWay routeWay = osmService.getRoadAtPoint(lng, lat, searchRadius);
//        if (routeWay!=null)
//        {
            points.setLng(lng);
            points.setLat(lat);
            points.setDistanceInRoad(distanceInLoad);
//            points.setRoadId(routeWay.getId());
//            points.setRoadName(routeWay.getName());
            pointMapper.insertDynamic(tableName, points);
//        }
//        else
//        {
//            System.out.println("找不到合适的道路!");
//        }
    }


    public List<List<String>> readGateWay2() {
        List<Points> points = pointMapper.selectAllGateWays();
        List<List<String>> bk = new ArrayList<>();
        for (Points p : points) {
            List<String> lis = new ArrayList<>();
            lis.add(Integer.toString(p.getId()));
            lis.add(Double.toString(p.getDistanceInRoad()));
            lis.add(Double.toString(p.getLng()));
            lis.add(Double.toString(p.getLat()));
            bk.add(lis);
        }
        return bk;
    }

    public List<List<String>> readStation2() {
        List<Points> points = pointMapper.selectAllStations();
        List<List<String>> bk = new ArrayList<>();
        for (Points p : points) {
            List<String> lis = new ArrayList<>();
            lis.add(Integer.toString(p.getId()));
            lis.add(Double.toString(p.getDistanceInRoad()));
            lis.add(Double.toString(p.getLng()));
            lis.add(Double.toString(p.getLat()));
            bk.add(lis);
        }
        return bk;
    }

    /*下面是新增的关于下载的代码*/

    public String convertSensorsToJson() {
        try {
            List<List<String>> sensorData = readSensor2();
            return buildJsonResponse("sensors", sensorData);
        } catch (Exception e) {
            throw new RuntimeException("传感器数据转换失败", e);
        }
    }

    // public String convertGatewaysToJson() {
    //     try {
    //         List<List<String>> gatewayData = readGateway2();
    //         return buildJsonResponse("gateways", gatewayData);
    //     } catch (Exception e) {
    //         throw new RuntimeException("网关数据转换失败", e);
    //     }
    // }

    public String convertGateWaysToJson() {
        try {
            List<List<String>> gateWayData = readGateWay2();
            return buildJsonResponse("gateWay", gateWayData);
        } catch (Exception e) {
            throw new RuntimeException("充电点数据转换失败", e);
        }
    }

    public String convertStationsToJson() {
        try {
            List<List<String>> stationData = readStation2();
            return buildJsonResponse("stations", stationData);
        } catch (Exception e) {
            throw new RuntimeException("服务站数据转换失败", e);
        }
    }


    /**
     * 构建通用的JSON响应
     */
    private String buildJsonResponse(String dataType, List<?> data) throws JsonProcessingException {
        Map<String, Object> exportData = new HashMap<>();
        exportData.put("type", dataType);
        exportData.put("count", data.size());
        exportData.put("exportTime", new Date());
        exportData.put("data", data);

        ObjectMapper mapper = new ObjectMapper();
        mapper.enable(SerializationFeature.INDENT_OUTPUT);
        return mapper.writeValueAsString(exportData);
    }

}
