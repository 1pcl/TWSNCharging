package top.bultrail.markroad.entity;

public class Points {

    private int id;
    private double distanceInRoad;
    private double lng;
    private double lat;

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }


    public double getDistanceInRoad() {
        return distanceInRoad;
    }

    public void setDistanceInRoad(double distanceInRoad) {
        this.distanceInRoad = distanceInRoad;
    }

    public double getLng() {
        return lng;
    }

    public void setLng(double lng) {
        this.lng = lng;
    }

    public double getLat() {
        return lat;
    }

    public void setLat(double lat) {
        this.lat = lat;
    }

}
