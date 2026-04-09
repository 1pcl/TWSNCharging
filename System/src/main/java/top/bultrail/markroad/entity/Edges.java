package top.bultrail.markroad.entity;

public class Edges {
    private String id;
    private String from_node;
    private String to_node;
    private double length_m;

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public String getFrom_node() {
        return from_node;
    }

    public void setFrom_node(String from_node) {
        this.from_node = from_node;
    }

    public String getTo_node() {
        return to_node;
    }

    public void setTo_node(String to_node) {
        this.to_node = to_node;
    }

    public double getLength_m() {
        return length_m;
    }

    public void setLength_m(double length_m) {
        this.length_m = length_m;
    }
}
