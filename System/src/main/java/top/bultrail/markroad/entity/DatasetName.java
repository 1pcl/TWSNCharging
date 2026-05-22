package top.bultrail.markroad.entity;

public class DatasetName {

    private String name;
    private int sensorSize;
    private int relaySize;
    private int stationSize;
    private int intersection;
    private double locationLng;
    private double locationLag;
    private int isDataset;

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

    public int getRelaySize() {
        return relaySize;
    }

    public void setRelaySize(int relaySize) {
        this.relaySize = relaySize;
    }

    public int getStationSize() {
        return stationSize;
    }

    public void setStationSize(int stationSize) {
        this.stationSize = stationSize;
    }

    public int getIntersection() {
        return intersection;
    }

    public void setIntersection(int intersection) {
        this.intersection = intersection;
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

    public int getIsDataset() {
        return isDataset;
    }

    public void setIsDataset(int isDataset) {
        this.isDataset = isDataset;
    }
}
