package top.bultrail.markroad.entity;

public class PathNode {
    private String nodeStartId;
    private String nodeEndId;
    private int order;
    private String nodeId;

    public String getNodeStartId() {
        return nodeStartId;
    }

    public void setNodeStartId(String nodeStartId) {
        this.nodeStartId = nodeStartId;
    }

    public String getNodeEndId() {
        return nodeEndId;
    }

    public void setNodeEndId(String nodeEndId) {
        this.nodeEndId = nodeEndId;
    }

    public int getOrder() {
        return order;
    }

    public void setOrder(int order) {
        this.order = order;
    }

    public String getNodeId() {
        return nodeId;
    }

    public void setNodeId(String nodeId) {
        this.nodeId = nodeId;
    }
}
