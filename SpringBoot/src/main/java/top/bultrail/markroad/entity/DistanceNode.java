package top.bultrail.markroad.entity;

public class DistanceNode {
    private int nodeStartId;
    private int nodeEndId;
    private double distance;

    public int getNodeStartId() {
        return nodeStartId;
    }

    public void setNodeStartId(int nodeStartId) {
        this.nodeStartId = nodeStartId;
    }

    public int getNodeEndId() {
        return nodeEndId;
    }

    public void setNodeEndId(int nodeEndId) {
        this.nodeEndId = nodeEndId;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
