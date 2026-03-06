package top.bultrail.markroad.entity;

import java.util.HashMap;
import java.util.Map;

public class ChargingSchedleNode {
    private int nodeId; // 统一的节点ID，例如 "sensor_1" 或 "charging_1"
    private String type; // 节点类型: "sensor", "charging", "station"
    private int originalId; // 原始ID: "1", "2", "3"
    private double lon; // 经度 (WGS-84)
    private double lat; // 纬度 (WGS-84)
    private Map<String, Object> battery_info; // 存储 battery_info

    public ChargingSchedleNode(int nodeId, String type, int originalId, double lon, double lat, Map<String, Object> battery_info) {
        this.nodeId = nodeId;
        this.type = type;
        this.originalId = originalId;
        this.lon = lon;
        this.lat = lat;
        this.battery_info = battery_info;
    }

    public ChargingSchedleNode() {}

    public int getNodeId() {
        return nodeId;
    }

    public void setNodeId(int nodeId) {
        this.nodeId = nodeId;
    }

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public int getOriginalId() {
        return originalId;
    }

    public void setOriginalId(int originalId) {
        this.originalId = originalId;
    }

    public double getLon() {
        return lon;
    }

    public void setLon(double lon) {
        this.lon = lon;
    }

    public double getLat() {
        return lat;
    }

    public void setLat(double lat) {
        this.lat = lat;
    }

    public Map<String, Object> getBattery_info() {
        return battery_info;
    }

    public void setBattery_info(Map<String, Object> battery_info) {
        this.battery_info = battery_info;
    }
}
