package top.bultrail.markroad.entity;

public class BatteryInfo {
    private double currentEnergy;
    private double currentEnergyRatio;
    private double chargingRate;

    public BatteryInfo() {}

    public double getChargingRate() {
        return chargingRate;
    }

    public void setChargingRate(double chargingRate) {
        this.chargingRate = chargingRate;
    }

    public double getCurrentEnergy() {
        return currentEnergy;
    }

    public void setCurrentEnergy(double currentEnergy) {
        this.currentEnergy = currentEnergy;
    }

    public double getCurrentEnergyRatio() {
        return currentEnergyRatio;
    }

    public void setCurrentEnergyRatio(double currentEnergyRatio) {
        this.currentEnergyRatio = currentEnergyRatio;
    }

}
