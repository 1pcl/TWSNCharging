package top.bultrail.markroad.entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.io.Serializable;


@Data
@AllArgsConstructor
@NoArgsConstructor
public class QuickSave implements Serializable {

    private String[][][] gate_way_points;
    private String[][][] sensor_array;
    private  String[][][] station_array;

    public String[][][] getStation_array() {
        return station_array;
    }

    public void setStation_array(String[][][] station_array) {
        this.station_array = station_array;
    }



    public String[][][] getGateWay_points() {
        return gate_way_points;
    }

    public void setGateWay_points(String[][][] gateWay_points) {
        this.gate_way_points = gateWay_points;
    }


    public String[][][] getSensor_array() {
        return sensor_array;
    }

    public void setSensor_array(String[][][] sensor_array) {
        this.sensor_array = sensor_array;
    }
}


