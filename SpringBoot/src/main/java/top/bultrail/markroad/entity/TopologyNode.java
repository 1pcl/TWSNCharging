package top.bultrail.markroad.entity;

import java.util.List;

public class TopologyNode {
    private String startNodeId;
    private String endNodeId;
    private List<String> nodes;

    public TopologyNode() {}

    public TopologyNode(String startNodeId, String endNodeId, List<String> nodes) {
        this.startNodeId = startNodeId;
        this.endNodeId = endNodeId;
        this.nodes = nodes;
    }

    public String getStartNodeId() {
        return startNodeId;
    }

    public void setStartNodeId(String startNodeId) {
        this.startNodeId = startNodeId;
    }

    public String getEndNodeId() {
        return endNodeId;
    }

    public void setEndNodeId(String endNodeId) {
        this.endNodeId = endNodeId;
    }

    public List<String> getNodes() {
        return nodes;
    }

    public void setNodes(List<String> nodes) {
        this.nodes = nodes;
    }
}
