package top.bultrail.markroad.entity;

public class DatasetInfo {
    private String name;
    private int sensorSize;
    private int relaySize;
    private int stationSize;


    public DatasetInfo(String name, int sensorSize, int gateWaySize, int stationSize) {
        this.name = name;
        this.sensorSize = sensorSize;
//        this.gatewaySize = gatewaySize;
        this.stationSize=stationSize;
    }

    public int getStationSize() {
        return stationSize;
    }

    public void setStationSize(int stationSize) {
        this.stationSize = stationSize;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getRelaySize() {
        return relaySize;
    }

    public void setRelaySize(int relaySize) {
        this.relaySize = relaySize;
    }

    public int getSensorSize() {
        return sensorSize;
    }

    public void setSensorSize(int sensorSize) {
        this.sensorSize = sensorSize;
    }


    @Override
    public String toString() {
        return "Dataset: " + name + ", Sensor Number: " + sensorSize + ", relay Number: " + relaySize+",Station Number:"+stationSize;
    }
}
