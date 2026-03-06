package top.bultrail.markroad.entity;

public class Node {
    private int nodeId;
    private double longitude;
    private double latitude;

    public Node(int nodeId, double longitude, double latitude) {
        this.nodeId = nodeId;
        this.longitude = longitude;
        this.latitude = latitude;
    }

    public Node() {

    }

    public int getNodeId() {
        return nodeId;
    }

    public void setNodeId(int nodeId) {
        this.nodeId = nodeId;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    @Override
    public String toString() {
        return nodeId + "(" + longitude + "," + latitude + ")";
    }
}
