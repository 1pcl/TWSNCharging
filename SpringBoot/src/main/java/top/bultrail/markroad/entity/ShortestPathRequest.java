package top.bultrail.markroad.entity;

public class ShortestPathRequest {
    private double startLng;
    private double startLat;
    private double endLng;
    private double endLat;

    // 必须包含 Getter/Setter 和无参构造函数
    public ShortestPathRequest() {}

    public double getStartLng() {
        return startLng;
    }

    public void setStartLng(double startLng) {
        this.startLng = startLng;
    }

    public double getStartLat() {
        return startLat;
    }

    public void setStartLat(double startLat) {
        this.startLat = startLat;
    }

    public double getEndLng() {
        return endLng;
    }

    public void setEndLng(double endLng) {
        this.endLng = endLng;
    }

    public double getEndLat() {
        return endLat;
    }

    public void setEndLat(double endLat) {
        this.endLat = endLat;
    }
}
