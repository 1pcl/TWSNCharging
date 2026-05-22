package top.bultrail.markroad.service;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import top.bultrail.markroad.entity.*;
import top.bultrail.markroad.mapper.EdgesMapper;
import top.bultrail.markroad.mapper.PointMapper;

import javax.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.util.*;

@Service
public class ChargingGraphService {

    @Autowired
    private PointMapper pointMapper;

    @Autowired
    private EdgesMapper edgesMapper;

    // 坐标转换参数（基于原始经纬度范围计算）
    private double minLng = Double.MAX_VALUE;
    private double maxLng = Double.MIN_VALUE;
    private double minLat = Double.MAX_VALUE;
    private double maxLat = Double.MIN_VALUE;
    private boolean boundsCalculated = false;

    /**
     * 计算经纬度范围（用于坐标转换）
     */
    private void calculateBounds(List<Points> allPoints) {
        if (boundsCalculated) return;

        for (Points point : allPoints) {
            minLng = Math.min(minLng, point.getLng());
            maxLng = Math.max(maxLng, point.getLng());
            minLat = Math.min(minLat, point.getLat());
            maxLat = Math.max(maxLat, point.getLat());
        }

        // 添加边距，避免边界点贴边
        double lngMargin = (maxLng - minLng) * 0.05;
        double latMargin = (maxLat - minLat) * 0.05;
        minLng -= lngMargin;
        maxLng += lngMargin;
        minLat -= latMargin;
        maxLat += latMargin;

        boundsCalculated = true;

        System.out.println("坐标转换范围 - 经度: [" + minLng + ", " + maxLng +
                "], 纬度: [" + minLat + ", " + maxLat + "]");
    }

    /**
     * 将经纬度转换为直角坐标（米制单位）
     * 使用墨卡托投影的简化版本，保持比例
     */
    private double[] convertToCartesian(double lng, double lat) {
        // 经度转换为X坐标（东西方向）
        // 纬度转换为Y坐标（南北方向）
        // 使用余弦修正纬度方向的比例（中纬度地区）
        double centerLat = (minLat + maxLat) / 2;
        double latCos = Math.cos(Math.toRadians(centerLat));

        // 经度范围对应的实际距离（米）- 约111公里每度
        double lngRangeKm = (maxLng - minLng) * 111.32 * latCos;
        double latRangeKm = (maxLat - minLat) * 110.574;

        // 取较大值作为缩放基准，保持正方形坐标系
        double maxRangeKm = Math.max(lngRangeKm, latRangeKm);

        // 计算归一化后的坐标（0-1000范围，便于处理）
        double normalizedX = (lng - minLng) / (maxLng - minLng);
        double normalizedY = (lat - minLat) / (maxLat - minLat);

        // 转换为米制坐标（以公里为单位，乘以1000转为米）
        double x_m = normalizedX * maxRangeKm * 1000;
        double y_m = normalizedY * maxRangeKm * 1000;

        return new double[]{x_m, y_m};
    }

    /**
     * 流式导出完整的基础设施数据（类似Python的save_to_json格式）
     * 包含: nodes, edges, adjacency, statistics
     * 坐标已转换为直角坐标系（米）
     */
    public void streamInfrastructureDataToResponse(HttpServletResponse response) throws IOException {
        JsonFactory jsonFactory = new JsonFactory();
        ObjectMapper mapper = new ObjectMapper();

        // 设置响应头
        response.setContentType("application/json");
        response.setCharacterEncoding("UTF-8");
        response.setHeader("Content-Disposition", "attachment; filename=\"road_graph_with_nodes.json\"");

        // 获取所有数据
        List<Points> sensors = pointMapper.selectAllSensors();
        List<Points> relays = pointMapper.selectAllRelays();
        List<Points> stations = pointMapper.selectAllStations();
        List<Points> intersections = pointMapper.selectAllIntersection();
        List<Edges> edges = edgesMapper.selectAllEdges();

        // 收集所有点用于计算坐标范围
        List<Points> allPoints = new ArrayList<>();
        allPoints.addAll(sensors);
        allPoints.addAll(relays);
        allPoints.addAll(stations);
        allPoints.addAll(intersections);

        // 计算坐标转换边界
        calculateBounds(allPoints);

        // 构建节点映射（坐标已转换）
        Map<String, NodeInfo> nodeMap = buildNodeMapWithConvertedCoords(sensors, relays, stations, intersections);

        // 构建邻接表并更新节点的neighbors
        Map<String, List<String>> adjacency = buildAdjacencyAndUpdateNeighbors(edges, nodeMap);

        // 计算统计信息
        Map<String, Object> statistics = calculateStatistics(nodeMap, edges);

        try (JsonGenerator generator = jsonFactory.createGenerator(response.getWriter())) {
            generator.setCodec(mapper);

            generator.writeStartObject();

            // 写入统计信息
            generator.writeFieldName("statistics");
            generator.writeObject(statistics);

            // 写入单位
            generator.writeStringField("unit", "meters");

            // 写入坐标参考信息
            generator.writeFieldName("coordinate_system");
            generator.writeStartObject();
            generator.writeStringField("type", "cartesian");
            generator.writeStringField("description", "Converted from WGS84 longitude/latitude to Cartesian coordinates (meters)");
            generator.writeEndObject();

            // 写入节点信息（使用转换后的直角坐标）
            generator.writeFieldName("nodes");
            generator.writeStartArray();
            for (NodeInfo node : nodeMap.values()) {
                generator.writeStartObject();
                generator.writeStringField("id", node.id);
                generator.writeNumberField("x_m", node.x);
                generator.writeNumberField("y_m", node.y);
                generator.writeStringField("node_type", node.nodeType);
                generator.writeNumberField("degree", node.degree);
                generator.writeFieldName("neighbors");
                List<String> neighbors = node.neighbors != null ? node.neighbors : Collections.emptyList();
                generator.writeObject(neighbors);
                generator.writeEndObject();
            }
            generator.writeEndArray();

            // 写入边信息
            generator.writeFieldName("edges");
            generator.writeStartArray();
            for (Edges edge : edges) {
                generator.writeStartObject();
                generator.writeStringField("id", edge.getId());
                generator.writeStringField("from", edge.getFrom_node());
                generator.writeStringField("to", edge.getTo_node());
                generator.writeNumberField("length_m", edge.getLength_m());
                generator.writeEndObject();
            }
            generator.writeEndArray();

            // 写入邻接表
            generator.writeFieldName("adjacency");
            generator.writeStartObject();
            for (Map.Entry<String, List<String>> entry : adjacency.entrySet()) {
                generator.writeFieldName(entry.getKey());
                generator.writeObject(entry.getValue());
            }
            generator.writeEndObject();

            generator.writeEndObject();
            generator.flush();
        }
    }

    /**
     * 构建节点映射（坐标已转换为直角坐标系）
     */
    private Map<String, NodeInfo> buildNodeMapWithConvertedCoords(List<Points> sensors, List<Points> relays,
                                                                  List<Points> stations, List<Points> intersections) {
        Map<String, NodeInfo> nodeMap = new LinkedHashMap<>();

        // 添加路口节点
        for (Points inter : intersections) {
            NodeInfo node = new NodeInfo();
            node.id = inter.getId();
            double[] coords = convertToCartesian(inter.getLng(), inter.getLat());
            node.x = coords[0];
            node.y = coords[1];
            node.nodeType = "intersection";
            node.neighbors = new ArrayList<>();
            nodeMap.put(node.id, node);
        }

        // 添加传感器节点
        for (Points sensor : sensors) {
            NodeInfo node = new NodeInfo();
            node.id = sensor.getId();
            double[] coords = convertToCartesian(sensor.getLng(), sensor.getLat());
            node.x = coords[0];
            node.y = coords[1];
            node.nodeType = "sensor";
            node.neighbors = new ArrayList<>();
            nodeMap.put(node.id, node);
        }

        // 添加中继节点
        for (Points relay : relays) {
            NodeInfo node = new NodeInfo();
            node.id = relay.getId();
            double[] coords = convertToCartesian(relay.getLng(), relay.getLat());
            node.x = coords[0];
            node.y = coords[1];
            node.nodeType = "relay";
            node.neighbors = new ArrayList<>();
            nodeMap.put(node.id, node);
        }

        // 添加服务站节点
        for (Points station : stations) {
            NodeInfo node = new NodeInfo();
            node.id = station.getId();
            double[] coords = convertToCartesian(station.getLng(), station.getLat());
            node.x = coords[0];
            node.y = coords[1];
            node.nodeType = "station";
            node.neighbors = new ArrayList<>();
            nodeMap.put(node.id, node);
        }

        return nodeMap;
    }

    /**
     * 构建邻接表并更新节点的neighbors
     */
    private Map<String, List<String>> buildAdjacencyAndUpdateNeighbors(List<Edges> edges, Map<String, NodeInfo> nodeMap) {
        Map<String, Set<String>> adjacencySet = new LinkedHashMap<>();

        for (Edges edge : edges) {
            String from = edge.getFrom_node();
            String to = edge.getTo_node();

            // 初始化邻居集合
            adjacencySet.computeIfAbsent(from, k -> new TreeSet<>()).add(to);
            adjacencySet.computeIfAbsent(to, k -> new TreeSet<>()).add(from);

            // 更新节点的度数
            NodeInfo fromNode = nodeMap.get(from);
            NodeInfo toNode = nodeMap.get(to);
            if (fromNode != null) {
                fromNode.degree++;
                fromNode.neighbors.add(to);
            }
            if (toNode != null) {
                toNode.degree++;
                toNode.neighbors.add(from);
            }
        }

        // 对每个节点的邻居进行排序（确保一致性）
        for (NodeInfo node : nodeMap.values()) {
            if (node.neighbors != null) {
                node.neighbors.sort(String::compareTo);
            }
        }

        // 转换为List<String>格式返回
        Map<String, List<String>> adjacency = new LinkedHashMap<>();
        for (Map.Entry<String, Set<String>> entry : adjacencySet.entrySet()) {
            adjacency.put(entry.getKey(), new ArrayList<>(entry.getValue()));
        }

        return adjacency;
    }

    /**
     * 计算统计信息
     */
    private Map<String, Object> calculateStatistics(Map<String, NodeInfo> nodeMap, List<Edges> edges) {
        Map<String, Object> stats = new LinkedHashMap<>();

        int totalNodes = nodeMap.size();
        int intersections = 0;
        int sensors = 0;
        int relays = 0;
        int stations = 0;

        for (NodeInfo node : nodeMap.values()) {
            switch (node.nodeType) {
                case "intersection":
                    intersections++;
                    break;
                case "sensor":
                    sensors++;
                    break;
                case "relay":
                    relays++;
                    break;
                case "station":
                    stations++;
                    break;
            }
        }

        double totalRoadLength = 0;
        for (Edges edge : edges) {
            totalRoadLength += edge.getLength_m();
        }

        stats.put("total_nodes", totalNodes);
        stats.put("intersections", intersections);
        stats.put("sensors", sensors);
        stats.put("relays", relays);
        stats.put("stations", stations);
        stats.put("total_edges", edges.size());
        stats.put("total_road_length_km", totalRoadLength / 1000);

        double avgSensorSpacing = sensors > 0 ? totalRoadLength / sensors : 0;
        stats.put("avg_sensor_spacing_km", avgSensorSpacing / 1000);

        double sensorDensity = totalRoadLength > 0 ? sensors / (totalRoadLength / 1000) : 0;
        stats.put("sensor_density_per_km", sensorDensity);

        return stats;
    }

    /**
     * 内部类：节点信息
     */
    private static class NodeInfo {
        String id;
        double x;
        double y;
        String nodeType;
        int degree = 0;
        List<String> neighbors;
    }
}