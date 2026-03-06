package top.bultrail.markroad.util;

/**
 * 中国大陆坐标系转换工具类
 * 实现了 WGS-84 (GPS/OSM) 和 GCJ-02 (高德/Google中国) 的互转。
 * GCJ-02 到 WGS-84 的转换使用反向纠偏 (迭代法)。
 */
public class CoordinateConverter {

    // 常量
    private static final double PI = 3.14159265358979324;
    private static final double A = 6378245.0; // WGS-84 椭球长半轴
    private static final double EE = 0.00669342162296594323; // WGS-84 椭球的第一偏心率平方

    // 允许的误差，用于GCJ-02到WGS-84的迭代 (调低至 1e-10，提高精度)
    private static final double THRESHOLD = 0.0000000001; // 1e-10

    /**
     * 判断坐标是否在中国大陆范围 (用于 GCJ-02 转换)
     * @param lng 经度 (WGS-84)
     * @param lat 纬度 (WGS-84)
     * @return 是否在中国大陆范围内
     */
    private static boolean outOfChina(double lng, double lat) {
        // 中国大陆地理范围 (排除南海、港澳台)
        if (lng < 72.004 || lng > 137.8347) {
            return true;
        }
        if (lat < 0.8293 || lat > 55.8271) {
            return true;
        }
        return false;
    }

    /**
     * 计算WGS-84到GCJ-02的偏移量 (核心偏移算法)
     * @param lng WGS-84 经度
     * @param lat WGS-84 纬度
     * @return [dLng, dLat] 偏移量数组
     */
    private static double[] delta(double lng, double lat) {
        double dLat = transformLat(lng - 105.0, lat - 35.0);
        double dLng = transformLng(lng - 105.0, lat - 35.0);

        // 关键的参数计算
        double radLat = lat / 180.0 * PI;
        double magic = Math.sin(radLat);
        magic = 1 - EE * magic * magic;
        double sqrtMagic = Math.sqrt(magic);

        // 分母
        double d = A / (sqrtMagic);
        double d2 = (A * (1 - EE)) / (magic * sqrtMagic);

        // 缩放修正
        dLat = (dLat * 180.0) / (d2 * PI);
        dLng = (dLng * 180.0) / (d * Math.cos(radLat) * PI);

        return new double[]{dLng, dLat};
    }

    private static double transformLat(double x, double y) {
        double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x));
        ret += (20.0 * Math.sin(6.0 * x * PI) + 20.0 * Math.sin(2.0 * x * PI)) * 2.0 / 3.0;
        ret += (20.0 * Math.sin(y * PI) + 40.0 * Math.sin(y / 3.0 * PI)) * 2.0 / 3.0;
        ret += (160.0 * Math.sin(y / 12.0 * PI) + 320 * Math.sin(y * PI / 30.0)) * 2.0 / 3.0;
        return ret;
    }

    private static double transformLng(double x, double y) {
        double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x));
        ret += (20.0 * Math.sin(6.0 * x * PI) + 20.0 * Math.sin(2.0 * x * PI)) * 2.0 / 3.0;
        ret += (20.0 * Math.sin(x * PI) + 40.0 * Math.sin(x / 3.0 * PI)) * 2.0 / 3.0;
        ret += (150.0 * Math.sin(x / 12.0 * PI) + 300.0 * Math.sin(x / 30.0 * PI)) * 2.0 / 3.0;
        return ret;
    }

    /**
     * WGS-84 (OSM) 转换为 GCJ-02 (高德)
     * @param wgsLng WGS-84 经度
     * @param wgsLat WGS-84 纬度
     * @return [gcjLng, gcjLat] GCJ-02 坐标数组
     */
    public static double[] wgs84ToGcj02(double wgsLng, double wgsLat) {
        if (outOfChina(wgsLng, wgsLat)) {
            return new double[]{wgsLng, wgsLat}; // 不在中国大陆，无需转换
        }

        double[] d = delta(wgsLng, wgsLat);
        double gcjLng = wgsLng + d[0];
        double gcjLat = wgsLat + d[1];

        return new double[]{gcjLng, gcjLat};
    }

    /**
     * GCJ-02 (高德) 转换为 WGS-84 (OSM) - 使用迭代法 (已优化)
     * @param gcjLng GCJ-02 经度
     * @param gcjLat GCJ-02 纬度
     * @return [wgsLng, wgsLat] WGS-84 坐标数组
     */
    public static double[] gcj02ToWgs84(double gcjLng, double gcjLat) {
        if (outOfChina(gcjLng, gcjLat)) {
            return new double[]{gcjLng, gcjLat}; // 不在中国大陆，无需转换
        }

        // 1. 设置迭代的初始估计值：使用平均偏移量作为更精确的初始近似值。
        // 这比直接从 wgsLng = gcjLng 开始迭代能更快地收敛到高精度结果。
        double wgsLng = gcjLng - 0.0065;
        double wgsLat = gcjLat - 0.006;

        // 2. 迭代逼近 WGS-84 坐标
        int maxIterations = 20; // 提高迭代次数，保证充分收敛
        for (int i = 0; i < maxIterations; i++) {
            // a. 将当前的 WGS-84 估计值，正向转换为 GCJ-02
            double[] d = delta(wgsLng, wgsLat);
            double currentGcjLng = wgsLng + d[0];
            double currentGcjLat = wgsLat + d[1];

            // b. 计算偏差：当前估算的 GCJ-02 与目标 GCJ-02 的差距
            double dLng = currentGcjLng - gcjLng;
            double dLat = currentGcjLat - gcjLat;

            // c. 检查是否达到精度要求
            if (Math.abs(dLng) < THRESHOLD && Math.abs(dLat) < THRESHOLD) {
                // 如果偏差小于阈值，认为已收敛
                break;
            }

            // d. 修正 WGS-84 估计值（反向减去偏差）
            wgsLng -= dLng;
            wgsLat -= dLat;
        }

        // 3. 返回经过迭代纠偏后的 WGS-84 坐标
        return new double[]{wgsLng, wgsLat};
    }
}