package top.bultrail.markroad.entity;

import lombok.Data;

@Data
public class DatasetLocation {
    private double locationLng;
    private double locationLat;

    public double getLocationLng() {
        return locationLng;
    }

    public void setLocationLng(double locationLng) {
        this.locationLng = locationLng;
    }

    public double getLocationLat() {
        return locationLat;
    }

    public void setLocationLat(double locationLat) {
        this.locationLat = locationLat;
    }
}
