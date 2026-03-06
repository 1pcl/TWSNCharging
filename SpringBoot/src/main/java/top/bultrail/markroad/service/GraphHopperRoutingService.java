package top.bultrail.markroad.service;

import org.springframework.beans.factory.annotation.Autowired;
import top.bultrail.markroad.entity.ShortestPathRequest;
import top.bultrail.markroad.entity.ShortestPathResponse;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import java.util.ArrayList;
import java.util.List;

//:: 直接启动服务，不需要重新导入，每次需要启动一下
//java -Xmx4g -Xms2g -jar graphhopper-3.2.jar server config.yml

@Service
public class GraphHopperRoutingService {

    private final RestTemplate restTemplate;
    private final ObjectMapper objectMapper = new ObjectMapper();

    // GraphHopper 本地服务端点
    private final String GRAPHHOPPER_BASE_URL = "http://localhost:8989/route";

    public GraphHopperRoutingService(RestTemplate restTemplate) {
        this.restTemplate = restTemplate;
    }

    public ShortestPathResponse findShortestPath(ShortestPathRequest request) {
        // 1. 构建 GraphHopper 请求 URL
        String url = buildGraphHopperUrl(request);

        try {
            // 2. 发送同步 GET 请求到本地 GraphHopper
            String responseBody = restTemplate.getForObject(url, String.class);

            if (responseBody == null) {
                return new ShortestPathResponse(false, "GraphHopper 返回空响应", 0, new ArrayList<>());
            }

            // 3. 解析 GraphHopper 返回的 JSON 结果
            JsonNode responseJson = objectMapper.readTree(responseBody);

            // 检查 GraphHopper 返回的 info 字段
            JsonNode info = responseJson.path("info");
            if (info.has("errors") && info.path("errors").size() > 0) {
                String message = info.path("errors").get(0).path("message").asText("路径计算失败");
                System.err.println("GraphHopper 返回失败: " + message +
                        "，起点(WGS-84): " + request.getStartLng() + "," + request.getStartLat() +
                        "，终点(WGS-84): " + request.getEndLng() + "," + request.getEndLat());
                return new ShortestPathResponse(false, "GraphHopper 失败: " + message, 0, new ArrayList<>());
            }

            // 4. 提取路径数据
            JsonNode path = responseJson.path("paths").get(0);

            // 总距离 (米) - GraphHopper返回的就是米
            double distanceMeters = path.path("distance").asDouble();

            // 5. 提取路径点
            List<List<Double>> pathPoints = extractPathPoints(path);

            // 6. 返回前端所需的格式 (WGS-84 坐标)
            return new ShortestPathResponse(
                    true,
                    "路径计算成功 (GraphHopper本地服务，最短距离)",
                    distanceMeters,
                    pathPoints
            );

        } catch (Exception e) {
            System.err.println("GraphHopper API 调用异常: " + e.getMessage());
            return new ShortestPathResponse(
                    false,
                    "路径计算失败 (请确保GraphHopper服务已启动): " + e.getMessage(),
                    0.0,
                    new ArrayList<>()
            );
        }
    }

    /**
     * 构建 GraphHopper 请求 URL
     * 格式: /route?point=纬度,经度&point=纬度,经度&vehicle=car&weighting=shortest
     */
    private String buildGraphHopperUrl(ShortestPathRequest request) {
        // GraphHopper 坐标格式: 纬度,经度 (注意：与OSRM顺序相反！)
        String point1 = String.format("%s,%s", request.getStartLat(), request.getStartLng());
        String point2 = String.format("%s,%s", request.getEndLat(), request.getEndLng());

        // 参数说明:
        // point - 路径点（纬度,经度）
        // vehicle=car - 车辆类型为汽车
        // weighting=shortest - 使用最短距离权重
        // points_encoded=false - 返回未编码的坐标
        // type=json - 返回JSON格式
        return String.format("%s?point=%s&point=%s&vehicle=car&weighting=shortest&points_encoded=false&type=json",
                GRAPHHOPPER_BASE_URL, point1, point2);
    }

    /**
     * 从 GraphHopper 响应中提取路径点
     */
    private List<List<Double>> extractPathPoints(JsonNode path) {
        List<List<Double>> pathPoints = new ArrayList<>();

        try {
            // 从 points.coordinates 字段提取路径点
            JsonNode points = path.path("points");
            JsonNode coordinates = points.path("coordinates");

            for (JsonNode coord : coordinates) {
                if (coord.isArray() && coord.size() >= 2) {
                    double lat = coord.get(1).asDouble(); // 纬度 (注意：GeoJSON是[经度,纬度])
                    double lng = coord.get(0).asDouble(); // 经度

                    List<Double> point = new ArrayList<>();
                    point.add(lng); // 保持与OSRM一致的顺序：[经度, 纬度]
                    point.add(lat);
                    pathPoints.add(point);
                }
            }

//            System.out.println("GraphHopper 路径点提取成功，共 " + pathPoints.size() + " 个点");

        } catch (Exception e) {
            System.err.println("提取 GraphHopper 路径点失败: " + e.getMessage());
        }

        return pathPoints;
    }

    /**
     * 检查 GraphHopper 服务是否正常运行
     */
    public boolean isServiceHealthy() {
        try {
            String healthUrl = "http://localhost:8989/health";
            String response = restTemplate.getForObject(healthUrl, String.class);
            return response != null && response.contains("OK");
        } catch (Exception e) {
            System.err.println("GraphHopper 健康检查失败: " + e.getMessage());
            return false;
        }
    }

    public double getHorizontalDistanceToRoad(double lat, double lon) {
        // GraphHopper 的 nearest 接口通常返回最近的线段投影点
        String url = "http://localhost:8989/nearest?point=" + lat + "," + lon;

        try {
            JsonNode root = restTemplate.getForObject(url, JsonNode.class);

            // 1. 获取投影后的点坐标 (Snapped Point)
            // GraphHopper 3.2 默认返回的是一个 Point 对象的 JSON
            JsonNode coordinates = root.path("coordinates");
            if (coordinates.isMissingNode() || coordinates.size() < 2) {
                return 0.0;
            }

            double snappedLon = coordinates.get(0).asDouble();
            double snappedLat = coordinates.get(1).asDouble();

            // 2. 计算【传感器原点】到【路边投影点】的直线距离
            // 这才是公式中要求的水平距离 (d_horizontal)
            double distance = calculateHaversine(lat, lon, snappedLat, snappedLon);

            return distance;
        } catch (Exception e) {
            return 0.0;
        }
    }

    // 辅助计算球面距离
    private double calculateHaversine(double lat1, double lon1, double lat2, double lon2) {
        double R = 6371000; // 地球半径，单位米
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                        Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    }
}