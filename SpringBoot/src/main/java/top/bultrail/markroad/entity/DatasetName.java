package top.bultrail.markroad.entity;

public class DatasetName {

    private String name;
    private int sensorSize;
    // private int gatewaySize;
    private int gateWaySize;
    private double locationLng;
    private double locationLag;
    private int roadSize;
    private int stationSize;
    private int isDataset;

    public int getIsDataset() {
        return isDataset;
    }

    public void setIsDataset(int isDataset) {
        this.isDataset = isDataset;
    }

    public int getRoadSize() {
        return roadSize;
    }

    public void setRoadSize(int roadSize) {
        this.roadSize = roadSize;
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

    public int getSensorSize() {
        return sensorSize;
    }

    public void setSensorSize(int sensorSize) {
        this.sensorSize = sensorSize;
    }

    // public int getGatewaySize() {
    //     return gatewaySize;
    // }

    // public void setGatewaySize(int gatewaySize) {
    //     this.gatewaySize = gatewaySize;
    // }


    public int getGateWaySize() {
        return gateWaySize;
    }

    public void setGateWaySize(int gateWaySize) {
        this.gateWaySize = gateWaySize;
    }

    public double getLocationLng() {
        return locationLng;
    }

    public void setLocationLng(double locationLng) {
        this.locationLng = locationLng;
    }

    public double getLocationLag() {
        return locationLag;
    }

    public void setLocationLag(double locationLag) {
        this.locationLag = locationLag;
    }
}
