package top.bultrail.markroad.mapper;

import org.apache.ibatis.annotations.Param;
import org.springframework.stereotype.Repository;
import top.bultrail.markroad.entity.Points;

import java.util.List;

@Repository
public interface PointMapper {


    //根据gatewany表的主键id查询对应那一行的信息
    Points selectRelayById(String id);

    //根据station表的主键id查询对应那一行的信息
    Points selectStationById(String id);

    //根据sensor表的主键id查询对应那一行的信息
    Points selectSensorById(String id);

    Points selectIntersectionById(String id);

    //查询sensor表的所有列
    List<Points> selectAllSensors();

    //查询gateWay表的所有列
    List<Points> selectAllRelays();

    //查询所有station表的所有
    List<Points> selectAllStations();

    List<Points> selectAllIntersection();

    //动态表插入数据
    int insertDynamic(@Param("tableName") String tableName,
                      @Param("record") Points record);

    //清空Sensor表
    void truncateSensor();

    void truncateRelays();

    //清空服务站
    void truncateStation();

    void truncateIntersection();

    //创建一个与现有表结构完全相同的新表，但是不复制数据
    void createTableLike(String newTableName, String originalTableName);

    //将原表中的所有数据复制到新表中
    void copyDataToNewTable(String newTableName, String originalTableName);

    //删除数据库表 的DDL操作
    void dropTable(String tableName);


    //将整个表的数据转移到主表
    void transferData(String tableName, String name);

    //重命名表名
    void renameTable(@Param("oldTableName") String oldTableName, @Param("newTableName") String newTableName);

}