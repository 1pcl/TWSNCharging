package top.bultrail.markroad.entity;

import java.util.List;

public class ShortestPathResponse {
    private boolean success;
    private String message;
    private double distance; // 路径总距离 (米)
    private List<List<Double>> path; // 路径坐标列表 [[lng1, lat1], [lng2, lat2], ...]

    public ShortestPathResponse(boolean success, String message, double distance, List<List<Double>> path) {
        this.success = success;
        this.message = message;
        this.distance = distance;
        this.path = path;
    }

    public boolean isSuccess() {
        return success;
    }

    public void setSuccess(boolean success) {
        this.success = success;
    }

    public String getMessage() {
        return message;
    }

    public void setMessage(String message) {
        this.message = message;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public List<List<Double>> getPath() {
        return path;
    }

    public void setPath(List<List<Double>> path) {
        this.path = path;
    }
}
