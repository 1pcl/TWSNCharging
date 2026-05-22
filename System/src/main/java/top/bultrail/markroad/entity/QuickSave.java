package top.bultrail.markroad.entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.io.Serializable;


@Data
@AllArgsConstructor
@NoArgsConstructor
public class QuickSave implements Serializable {

    private String[][][] relay_points;
    private String[][][] sensor_array;
    private  String[][][] station_array;
    private String[][][] road_array;

    public String[][][] getStation_array() {
        return station_array;
    }

    public void setStation_array(String[][][] station_array) {
        this.station_array = station_array;
    }


    public String[][][] getRelay_points() {
        return relay_points;
    }

    public void setRelay_points(String[][][] relay_points) {
        this.relay_points = relay_points;
    }

    public String[][][] getSensor_array() {
        return sensor_array;
    }

    public void setSensor_array(String[][][] sensor_array) {
        this.sensor_array = sensor_array;
    }

    public String[][][] getRoad_array() {
        return road_array;
    }

    public void setRoad_array(String[][][] road_array) {
        this.road_array = road_array;
    }
}


