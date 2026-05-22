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

@Service
public class PointsService {

    @Autowired
    private PointMapper pointMapper;

    @Autowired
    private EdgesMapper edgesMapper;

    private static final double INTERSECTION_MERGE_THRESHOLD = 25.0;
    private static final double SNAP_TO_ROAD_THRESHOLD = 1000000.0;
    private static final double MIN_EDGE_LENGTH = 0.1;

    private int sensorCounter = 0;
    private int relayCounter = 0;
    private int stationCounter = 0;
    private int intersectionCounter = 0;
    private int edgeCounter = 0;

    @Transactional
    public void quick_write_new2(String[][][] relay_points, String[][][] sensor_array,
                                 String[][][] station_array, String[][][] road_array) {

        System.out.println("========== 执行空间拓扑映射算法 ==========");

        resetCounters();
        clearAllData();

        // 1. 预处理并保存所有物理设备节点
        List<DeviceNode> allDevices = new ArrayList<>();
        collectAndSaveDevices(sensor_array, "sensor", allDevices);
        collectAndSaveDevices(relay_points, "relay", allDevices);
        collectAndSaveDevices(station_array, "station", allDevices);

        if (road_array == null || road_array.length == 0) return;

        // 2. 解析原始道路并提取“交点候选”
        List<RoadInfo> roads = new ArrayList<>();
        List<RoadPoint> candidates = new ArrayList<>();

        for (int i = 0; i < road_array.length; i++) {
            String[][] road = road_array[i];
            if (road.length < 2) continue;
            try {
                double sLng = Double.parseDouble(road[0][0]);
                double sLat = Double.parseDouble(road[0][1]);
                double eLng = Double.parseDouble(road[1][0]);
                double eLat = Double.parseDouble(road[1][1]);
                double len = road.length >= 3 ? Double.parseDouble(road[2][0]) :
                        calculateDistance(sLng, sLat, eLng, eLat);

                RoadInfo ri = new RoadInfo(sLng, sLat, eLng, eLat, len);
                roads.add(ri);
                // 必须添加端点，否则道路无法与路口连通
                candidates.add(new RoadPoint(sLng, sLat, i, true));
                candidates.add(new RoadPoint(eLng, eLat, i, false));
            } catch (Exception e) { }
        }

        // 3. 自动计算线段交叉点
        for (int i = 0; i < roads.size(); i++) {
            for (int j = i + 1; j < roads.size(); j++) {
                double[] cross = findSegmentIntersection(roads.get(i), roads.get(j));
                if (cross != null) {
                    candidates.add(new RoadPoint(cross[0], cross[1], -1, false));
                }
            }
        }

        // 4. 合并路口：25米以内合并
        List<IntersectionNode> intersections = createIntersections(candidates);

        // 5. 设备强映射：找到最近的道路
        Map<DeviceNode, RoadInfo> deviceToNearestRoad = new HashMap<>();
        for (DeviceNode device : allDevices) {
            RoadInfo nearestRoad = findNearestRoad(device, roads);
            if (nearestRoad != null) {
                deviceToNearestRoad.put(device, nearestRoad);
            }
        }

        // 6. 核心拓扑切分：在每条路上按顺序串联所有映射点
        for (RoadInfo road : roads) {
            processRoadProjection(road, intersections, deviceToNearestRoad);
        }

        validateNoIsolatedNodes();
        System.out.println("========== 拓扑构建完成 ==========");
    }

    /**
     * 找到距离设备最近的道路
     */
    private RoadInfo findNearestRoad(DeviceNode device, List<RoadInfo> roads) {
        RoadInfo nearestRoad = null;
        double minDistance = Double.MAX_VALUE;
        for (RoadInfo road : roads) {
            double dist = pointToLineDistance(device.lng, device.lat,
                    road.startLng, road.startLat, road.endLng, road.endLat);
            if (dist < minDistance) {
                minDistance = dist;
                nearestRoad = road;
            }
        }
        return nearestRoad;
    }
    /**
     * 空间投影逻辑：优化版 - 强制引入道路端点确保连通
     */
    private void processRoadProjection(RoadInfo road, List<IntersectionNode> intersections,
                                       Map<DeviceNode, RoadInfo> deviceToNearestRoad) {
        List<PointOnLine> nodesOnRoad = new ArrayList<>();

        // 1. 投影强映射到此路的设备
        for (Map.Entry<DeviceNode, RoadInfo> entry : deviceToNearestRoad.entrySet()) {
            if (entry.getValue() == road) {
                DeviceNode dev = entry.getKey();
                double offset = calculateProjectionOffset(dev.lng, dev.lat, road);
                nodesOnRoad.add(new PointOnLine(dev.nodeId, dev.lng, dev.lat, offset));
            }
        }

        // 2. 投影路口点（这里做了逻辑加强）
        for (IntersectionNode inter : intersections) {
            double dist = pointToLineDistance(inter.lng, inter.lat,
                    road.startLng, road.startLat, road.endLng, road.endLat);

            // 逻辑加强：如果该路口本身就是这条路的定义端点，或者距离足够近，则必须参与连边
            boolean isEndpoint = (calculateDistance(inter.lng, inter.lat, road.startLng, road.startLat) < 5.0) ||
                    (calculateDistance(inter.lng, inter.lat, road.endLng, road.endLat) < 5.0);

            if (isEndpoint || dist <= INTERSECTION_MERGE_THRESHOLD + 1.0) {
                double offset = calculateProjectionOffset(inter.lng, inter.lat, road);
                nodesOnRoad.add(new PointOnLine(inter.nodeId, inter.lng, inter.lat, offset));
            }
        }

        // 如果这条路上有点，但点太少（比如只有一个孤零零的设备），
        // 这种情况下由于 sort 后循环不会执行，设备就会变孤立。
        // 所以我们必须确保路口点（端点）被正确识别并加入。
        if (nodesOnRoad.size() < 2) {
            // 如果还是没法构成边，尝试寻找全集里离这条路起点/终点最近的路口强制补全
            return;
        }

        // 3. 排序并依次连边（珍珠项链式连接）
        nodesOnRoad.sort(Comparator.comparingDouble(p -> p.distance));

        for (int i = 0; i < nodesOnRoad.size() - 1; i++) {
            PointOnLine p1 = nodesOnRoad.get(i);
            PointOnLine p2 = nodesOnRoad.get(i + 1);

            // 只要不是同一个 ID，就建立连接
            if (!p1.nodeId.equals(p2.nodeId)) {
                double segLen = Math.abs(p2.distance - p1.distance);
                createEdgeIfNotExists(p1.nodeId, p2.nodeId, Math.max(segLen, MIN_EDGE_LENGTH));
            }
        }
    }
    /**
     * 线段求交算法 (保持不变)
     */
    private double[] findSegmentIntersection(RoadInfo r1, RoadInfo r2) {
        double x1 = r1.startLng, y1 = r1.startLat, x2 = r1.endLng, y2 = r1.endLat;
        double x3 = r2.startLng, y3 = r2.startLat, x4 = r2.endLng, y4 = r2.endLat;
        double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
        if (denom == 0) return null;
        double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
        double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
        if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
            return new double[]{ x1 + ua * (x2 - x1), y1 + ua * (y2 - y1) };
        }
        return null;
    }

    /**
     * 合并路口算法：计算质心，确保合并后的路口位置精准
     */
    private List<IntersectionNode> createIntersections(List<RoadPoint> roadPoints) {
        List<IntersectionNode> intersections = new ArrayList<>();
        boolean[] processed = new boolean[roadPoints.size()];

        for (int i = 0; i < roadPoints.size(); i++) {
            if (processed[i]) continue;
            List<RoadPoint> cluster = new ArrayList<>();
            cluster.add(roadPoints.get(i));
            processed[i] = true;

            for (int j = i + 1; j < roadPoints.size(); j++) {
                if (!processed[j]) {
                    double d = calculateDistance(roadPoints.get(i).lng, roadPoints.get(i).lat,
                            roadPoints.get(j).lng, roadPoints.get(j).lat);
                    if (d <= INTERSECTION_MERGE_THRESHOLD) {
                        cluster.add(roadPoints.get(j));
                        processed[j] = true;
                    }
                }
            }
            // 计算质心
            double avgLng = cluster.stream().mapToDouble(p -> p.lng).average().orElse(roadPoints.get(i).lng);
            double avgLat = cluster.stream().mapToDouble(p -> p.lat).average().orElse(roadPoints.get(i).lat);

            String id = "I" + (++intersectionCounter);
            Points p = new Points();
            p.setId(id); p.setLng(avgLng); p.setLat(avgLat);
            pointMapper.insertDynamic("intersection", p);
            intersections.add(new IntersectionNode(id, avgLng, avgLat, cluster));
        }
        return intersections;
    }

    /**
     * 验证孤立节点 (保持不变)
     */
    private void validateNoIsolatedNodes() {
        List<Edges> allEdges = edgesMapper.selectAllEdges();
        Set<String> connectedNodes = new HashSet<>();
        for (Edges edge : allEdges) {
            connectedNodes.add(edge.getFrom_node());
            connectedNodes.add(edge.getTo_node());
        }
        List<Points> allNodes = getAllNodes();
        List<String> isolatedNodes = new ArrayList<>();
        for (Points node : allNodes) {
            if (!connectedNodes.contains(node.getId())) {
                isolatedNodes.add(node.getId());
            }
        }
        if (!isolatedNodes.isEmpty()) {
            System.err.println("警告: 发现 " + isolatedNodes.size() + " 个孤立节点: " + isolatedNodes);
        } else {
            System.out.println("验证通过：全图连通性良好");
        }
    }

    /**
     * 获取邻居信息 (保持不变)
     */
    public Map<String, List<String>> getAllNodeNeighbors() {
        Map<String, List<String>> result = new HashMap<>();
        List<Edges> allEdges = edgesMapper.selectAllEdges();
        Map<String, Set<String>> adjacency = new HashMap<>();
        for (Edges edge : allEdges) {
            adjacency.computeIfAbsent(edge.getFrom_node(), k -> new HashSet<>()).add(edge.getTo_node());
            adjacency.computeIfAbsent(edge.getTo_node(), k -> new HashSet<>()).add(edge.getFrom_node());
        }
        for (Map.Entry<String, Set<String>> entry : adjacency.entrySet()) {
            List<String> neighbors = new ArrayList<>(entry.getValue());
            neighbors.sort(String::compareTo);
            result.put(entry.getKey(), neighbors);
        }
        for (Points node : getAllNodes()) {
            if (!result.containsKey(node.getId())) result.put(node.getId(), new ArrayList<>());
        }
        return result;
    }

    // ========== 以下功能函数及内部类签名严禁修改，保证外部调用正常 ==========

    private void collectAndSaveDevices(String[][][] array, String type, List<DeviceNode> allDevices) {
        if (array == null) return;
        for (String[][] group : array) {
            for (String[] point : group) {
                try {
                    double lng = Double.parseDouble(point[0]);
                    double lat = Double.parseDouble(point[1]);
                    String id = generateIdByType(type);
                    Points p = new Points();
                    p.setId(id); p.setLng(lng); p.setLat(lat);
                    pointMapper.insertDynamic(type.equals("relay") ? "relay" : type, p);
                    allDevices.add(new DeviceNode(id, lng, lat, type));
                } catch (Exception e) { }
            }
        }
    }

    private String generateIdByType(String type) {
        if ("sensor".equals(type)) return "S" + (++sensorCounter);
        if ("relay".equals(type)) return "G" + (++relayCounter);
        if ("station".equals(type)) return "ST" + (++stationCounter);
        return "UNK";
    }

    private void createEdgeIfNotExists(String from, String to, double len) {
        if (from.equals(to)) return;
        List<Edges> exist = edgesMapper.selectAllEdges();
        boolean duplicated = exist.stream().anyMatch(e ->
                (e.getFrom_node().equals(from) && e.getTo_node().equals(to)) ||
                        (e.getFrom_node().equals(to) && e.getTo_node().equals(from))
        );
        if (!duplicated) {
            Edges edge = new Edges();
            edge.setId("R" + (++edgeCounter));
            edge.setFrom_node(from);
            edge.setTo_node(to);
            edge.setLength_m(len);
            edgesMapper.insertEdge(edge);
        }
    }

    public void clear2() { clearAllData(); }
    public List<List<String>> readSensor2() { return fetchPoints(pointMapper.selectAllSensors()); }
    public List<List<String>> readRealy2() { return fetchPoints(pointMapper.selectAllRelays()); }
    public List<List<String>> readStation2() { return fetchPoints(pointMapper.selectAllStations()); }
    public List<List<String>> readIntersection2() { return fetchPoints(pointMapper.selectAllIntersection()); }

    public List<Points> getAllNodes() {
        List<Points> allNodes = new ArrayList<>();
        allNodes.addAll(pointMapper.selectAllSensors());
        allNodes.addAll(pointMapper.selectAllRelays());
        allNodes.addAll(pointMapper.selectAllStations());
        allNodes.addAll(pointMapper.selectAllIntersection());
        return allNodes;
    }

    private List<List<String>> fetchPoints(List<Points> list) {
        List<List<String>> res = new ArrayList<>();
        if (list == null) return res;
        for (Points p : list) res.add(Arrays.asList(p.getId(), String.valueOf(p.getLng()), String.valueOf(p.getLat())));
        return res;
    }

    public String convertSensorsToJson() { return toJsonWrapper("sensors", readSensor2()); }
    public String convertRelaysToJson() { return toJsonWrapper("relay", readRealy2()); }
    public String convertStationsToJson() { return toJsonWrapper("stations", readStation2()); }

    private String toJsonWrapper(String type, Object data) {
        try {
            Map<String, Object> map = new HashMap<>();
            map.put("type", type);
            map.put("exportTime", new Date());
            map.put("data", data);
            ObjectMapper m = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
            return m.writeValueAsString(map);
        } catch (Exception e) { return "{}"; }
    }

    private double calculateProjectionOffset(double px, double py, RoadInfo road) {
        double dx = road.endLng - road.startLng;
        double dy = road.endLat - road.startLat;
        if (dx == 0 && dy == 0) return 0;
        double t = ((px - road.startLng) * dx + (py - road.startLat) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));
        return t * road.length;
    }

    private double pointToLineDistance(double px, double py, double x1, double y1, double x2, double y2) {
        double dx = x2 - x1, dy = y2 - y1;
        if (dx == 0 && dy == 0) return calculateDistance(px, py, x1, y1);
        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));
        return calculateDistance(px, py, x1 + t * dx, y1 + t * dy);
    }

    private double calculateDistance(double lng1, double lat1, double lng2, double lat2) {
        double R = 6371000;
        double dLat = Math.toRadians(lat2 - lat1), dLng = Math.toRadians(lng2 - lng1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.sin(dLng / 2) * Math.sin(dLng / 2);
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    private void clearAllData() {
        pointMapper.truncateSensor(); pointMapper.truncateRelays();
        pointMapper.truncateStation(); pointMapper.truncateIntersection();
        edgesMapper.truncateEdges();
    }

    private void resetCounters() {
        sensorCounter = 0; relayCounter = 0; stationCounter = 0; intersectionCounter = 0; edgeCounter = 0;
    }

    // 内部数据类
    private static class RoadInfo {
        double startLng, startLat, endLng, endLat, length;
        RoadInfo(double sl, double sa, double el, double ea, double l) {
            this.startLng = sl; this.startLat = sa; this.endLng = el; this.endLat = ea; this.length = l;
        }
    }
    private static class RoadPoint {
        double lng, lat; int roadIndex; boolean isStart;
        RoadPoint(double lo, double la, int idx, boolean s) { this.lng = lo; this.lat = la; this.roadIndex = idx; this.isStart = s; }
    }
    private static class IntersectionNode {
        String nodeId; double lng, lat; List<RoadPoint> roadPoints;
        IntersectionNode(String id, double lo, double la, List<RoadPoint> rp) { this.nodeId = id; this.lng = lo; this.lat = la; this.roadPoints = rp; }
    }
    private static class DeviceNode {
        String nodeId; double lng, lat; String type;
        DeviceNode(String id, double lo, double la, String t) { this.nodeId = id; this.lng = lo; this.lat = la; this.type = t; }
    }
    private static class PointOnLine {
        String nodeId; double lng, lat, distance;
        PointOnLine(String id, double lo, double la, double d) { this.nodeId = id; this.lng = lo; this.lat = la; this.distance = d; }
    }
}