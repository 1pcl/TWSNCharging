package top.bultrail.markroad.service;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import top.bultrail.markroad.entity.*;
import top.bultrail.markroad.mapper.DatasetNameMapper;
import top.bultrail.markroad.mapper.PointMapper;
import top.bultrail.markroad.mapper.DistanceMapper; // 假设用于存储 DistanceNode

import javax.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.*;

/**
 * 职责:
 * 1. 从数据库获取所有基础设施点数据。
 * 2. 计算所有基础设施点之间两两的最短距离和路径。
 * 3. 提供两个入口：一个用于计算和存储数据到数据库，另一个用于计算并返回数据。
 *
 * 【重要修改】: 实现了分批计算和分批存储，以解决数据库连接超时问题。
 */
@Service
public class ChargingGraphService {

    // 【新增】定义批次大小。该值应根据数据库性能和网络延迟进行调整。
    // BATCH_SIZE 表示每批次处理的点对数量。
    private static final int BATCH_SIZE = 100;

    @Autowired
    private PointMapper pointMapper;

    @Autowired
    private DistanceMapper distanceMapper;

    @Autowired
    private GraphHopperRoutingService graphHopperRoutingService;

    @Autowired
    private DatasetNameMapper datasetNameMapper;


    private Map<String, List<Points>> readInfrastructureData() {
        Map<String, List<Points>> data = new HashMap<>();
        try {
            data.put("sensors", pointMapper.selectAllSensors());
        } catch (Exception e) {
            System.err.println("WARN: 无法从 PointMapper 读取传感器数据，错误: " + e.getMessage());
        }
        try {
            data.put("gateWay", pointMapper.selectAllGateWays());
        } catch (Exception e) {
            System.err.println("WARN: 无法从 PointMapper 读取中继数据，错误: " + e.getMessage());
        }
        try {
            data.put("stations", pointMapper.selectAllStations());
        } catch (Exception e) {
            System.err.println("WARN: 无法从 PointMapper 读取服务站数据，错误: " + e.getMessage());
        }
        return data;
    }


    /**
     * 【公共方法 - 存储到数据库】
     * 负责【分批】计算所有节点、距离和路径，并批量存储到数据库。
     * 【原 computeAllInfrastructureData() 的逻辑已整合到此方法中】
     */
    public void calculateAndSaveInfrastructureData() {
        // 1. 读取数据库数据
        Map<String, List<Points>> data = readInfrastructureData();
        List<Points> sensors = data.getOrDefault("sensors", Collections.emptyList());
        List<Points> gateWay = data.getOrDefault("gateWay", Collections.emptyList());
        List<Points> stations = data.getOrDefault("stations", Collections.emptyList());

        // 2. 生成所有基础设施节点列表
        List<ChargingSchedleNode> pointInfo = generateAllNodes(sensors, gateWay, stations);

        if (pointInfo.isEmpty()) {
            System.out.println("没有基础设施点，无需计算和存储。");
            return;
        }

        System.out.println("开始计算 " + pointInfo.size() + " 个点之间的两两最短路径，并以 " + BATCH_SIZE + " 对/批进行分批存储...");


        // 3. 执行分批计算和存储
        processAndSaveInBatches(pointInfo);

    }

    /**
     * 【核心分批逻辑】
     * 循环计算所有点对，并在达到 BATCH_SIZE 时触发批量插入。
     */
    private void processAndSaveInBatches(List<ChargingSchedleNode> pointInfo) {
        // 用于收集路径节点 (分批完成后才插入)
        Set<Node> batchPathNodesSet = new HashSet<>();

        List<DistanceNode> currentDistanceBatch = new ArrayList<>();
        List<PathNode> currentPathBatch = new ArrayList<>();

        int totalPoints = pointInfo.size();
        int totalPairs = totalPoints * (totalPoints - 1);
        int pairCount = 0;
        int batchCounter = 1;

        for (int i = 0; i < totalPoints; i++) {
            ChargingSchedleNode node1 = pointInfo.get(i);
            int startId = node1.getNodeId();

            for (int j = 0; j < totalPoints; j++) {
                if (i == j) continue; // 不计算自己到自己的距离
                pairCount++;

                ChargingSchedleNode node2 = pointInfo.get(j);
                int endId = node2.getNodeId();

                // --- A. 调用路由服务并收集数据 ---
                ShortestPathRequest request = new ShortestPathRequest();
                request.setStartLng(node1.getLon());
                request.setStartLat(node1.getLat());
                request.setEndLng(node2.getLon());
                request.setEndLat(node2.getLat());

                ShortestPathResponse response = graphHopperRoutingService.findShortestPath(request);

                // 1. 记录结果到 DistanceNode
                float distanceMeters = response.isSuccess() ? (float) response.getDistance() : Float.MAX_VALUE;
                DistanceNode distNode = createDistanceNode(startId, endId, distanceMeters);
                currentDistanceBatch.add(distNode);


                // --- B. 达到批次大小，执行插入操作 ---
                if (currentDistanceBatch.size() >= BATCH_SIZE) {
                    System.out.println("--- 正在处理第 " + batchCounter + " 批次 (已完成 " + pairCount + "/" + totalPairs + " 对) ---");
                    // 执行批量存储
                    saveBatchData(currentDistanceBatch, currentPathBatch, batchPathNodesSet);

                    // 清空批次列表，准备下一批
                    currentDistanceBatch.clear();
                    batchCounter++;
                }
            }
        }

        // --- C. 处理最后一个（可能不完整的）批次 ---
        if (!currentDistanceBatch.isEmpty()) {
            System.out.println("--- 正在处理最后一个批次 (总对数: " + pairCount + "/" + totalPairs + " 对) ---");
            saveBatchData(currentDistanceBatch, currentPathBatch, batchPathNodesSet);
        }

        System.out.println("所有点对的最短路径计算和存储完成。总批次数: " + batchCounter);
    }

    /**
     * 【辅助方法】集中处理数据库插入
     */
    private void saveBatchData(List<DistanceNode> distanceList, List<PathNode> pathList, Set<Node> pathNodesSet) {

        // 批量插入距离数据 (DistanceNode)
        if (!distanceList.isEmpty()) {
            try {
                int insertedCount = distanceMapper.batchInsertDistance(distanceList);
                System.out.println("  > 成功批量插入 " + insertedCount + " 条距离数据 (DistanceNode)。");
            } catch (Exception e) {
                System.err.println("  > ❗ 批量插入距离数据失败 (连接超时或数据问题): " + e.getMessage());
            }
        }

    }


    private List<ChargingSchedleNode> generateAllNodes(
            List<Points> sensors,
            List<Points> gateWays,
            List<Points> stations) {

        List<ChargingSchedleNode> scheduleNodes = new ArrayList<>();

        int stationCount = stations.size();
        int gatewayCount = gateWays.size();

        // 1. 传感器
        for (Points sensor : sensors) {
            ChargingSchedleNode node = new ChargingSchedleNode();
            node.setNodeId(sensor.getId()-1+stationCount+gatewayCount);
            node.setType("sensor");
            node.setOriginalId(sensor.getId());
            node.setLon(sensor.getLng());
            node.setLat(sensor.getLat());
            scheduleNodes.add(node);
        }

        // 2. 中继节点
        for (Points gateWay : gateWays) {
            ChargingSchedleNode node = new ChargingSchedleNode();
            node.setNodeId(gateWay.getId()-1+stationCount);
            node.setType("gateWay");
            node.setOriginalId(gateWay.getId());
            node.setLon(gateWay.getLng());
            node.setLat(gateWay.getLat());
            scheduleNodes.add(node);
        }

        // 3. 服务站
        for (Points station : stations) {
            ChargingSchedleNode node = new ChargingSchedleNode();
            node.setNodeId(station.getId()-1);
            node.setType("station");
            node.setOriginalId(station.getId());
            node.setLon(station.getLng());
            node.setLat(station.getLat());
            scheduleNodes.add(node);
        }

        return scheduleNodes;
    }



    // 辅助方法，用于创建 DistanceNode (保持代码简洁)
    private DistanceNode createDistanceNode(int startId, int endId, float distance) {
        DistanceNode distNode = new DistanceNode();
        distNode.setNodeStartId(startId);
        distNode.setNodeEndId(endId);
        distNode.setDistance(distance);
        return distNode;
    }



    // 批次大小配置
    private static final int DISTANCE_BATCH_SIZE = 2000;

    public void streamInfrastructureDataToResponse(HttpServletResponse response) throws IOException {
        JsonFactory jsonFactory = new JsonFactory();
        ObjectMapper mapper = new ObjectMapper();

        // 设置响应头
        response.setContentType("application/json");
        response.setCharacterEncoding("UTF-8");

        try (JsonGenerator generator = jsonFactory.createGenerator(response.getWriter())) {
            // 关键：设置 ObjectCodec，这样 writeObject() 才能正常工作
            generator.setCodec(mapper);

            generator.writeStartObject(); // 开始整个JSON对象

            // 写入顺序可以根据需要调整
            // 1. 先写stations
            writeStationsToGenerator(generator);

            // 2. 再写gateWay
            writeGateWaysToGenerator(generator);

            // 3. 然后写sensors
            writeSensorsToGenerator(generator);

            // 4. 最后写graph_G（使用分批处理）
            writeDistanceToGenerator(generator);

            generator.writeEndObject(); // 结束整个JSON对象

        } catch (Exception e) {
            // 流式写入已经开始，不能调用 response.sendError()
            throw new IOException("流式导出失败: " + e.getMessage(), e);
        }
    }

    /**
     * 写入stations数据（一次性）
     */
    private void writeStationsToGenerator(JsonGenerator generator) throws IOException {
        generator.writeFieldName("stations");
        generator.writeStartArray();

        List<Points> stations = pointMapper.selectAllStations();
        for (Points station : stations) {
            generator.writeObject(station);
        }

        generator.writeEndArray();
    }

    /**
     * 写入gateWay数据（一次性）
     */
    private void writeGateWaysToGenerator(JsonGenerator generator) throws IOException {
        generator.writeFieldName("gateWay");
        generator.writeStartArray();

        List<Points> gateways = pointMapper.selectAllGateWays();
        for (Points gateway : gateways) {
            generator.writeObject(gateway);
        }

        generator.writeEndArray();
    }

    /**
     * 写入sensors数据（一次性）
     */
    private void writeSensorsToGenerator(JsonGenerator generator) throws IOException {
        generator.writeFieldName("sensors");
        generator.writeStartArray();

        List<Points> sensors = pointMapper.selectAllSensors();
        for (Points sensor : sensors) {
            generator.writeObject(sensor);
        }

        generator.writeEndArray();
    }

    /**
     * 写入graph_G数据（distance表 - 分批处理）
     */
    private void writeDistanceToGenerator(JsonGenerator generator) throws IOException {
        generator.writeFieldName("graph_G");
        generator.writeStartArray();

        int offset = 0;
        boolean hasMoreData = true;
        int totalCount = 0;

        System.out.println("开始分批查询distance数据...");

        while (hasMoreData) {
            // 使用分批查询
//            List<DistanceNode> batch = distanceMapper.selectDistanceBatch(offset, DISTANCE_BATCH_SIZE);
            List<DistanceNode> batch = distanceMapper.selectDistanceBatch(DISTANCE_BATCH_SIZE, offset);

            if (batch == null || batch.isEmpty()) {
                hasMoreData = false;
                System.out.println("distance数据查询完成，共处理" + totalCount + "条记录");
            } else {
                // 写入当前批次
                for (DistanceNode edge : batch) {
                    generator.writeObject(edge);
                }

                totalCount += batch.size();
                offset += batch.size();

                // 刷新缓冲区
                generator.flush();

                // 输出进度信息
                if (batch.size() > 0) {
                    System.out.println("已处理distance记录: " + totalCount + "条");
                }

                // 如果返回数量小于批次大小，说明是最后一批
                if (batch.size() < DISTANCE_BATCH_SIZE) {
                    hasMoreData = false;
                    System.out.println("distance数据查询完成，共处理" + totalCount + "条记录");
                }
            }
        }

        generator.writeEndArray();
    }
}