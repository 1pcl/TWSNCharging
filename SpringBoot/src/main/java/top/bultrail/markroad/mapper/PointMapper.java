package top.bultrail.markroad.mapper;

import org.apache.ibatis.annotations.Param;
import org.springframework.stereotype.Repository;
import top.bultrail.markroad.entity.Points;

import java.util.List;

@Repository
public interface PointMapper {

//    //根据gatewany表的主键id删除对应那一行的信息
//    int deleteGatewayById(Integer id);
//
//    //插入gatewany表所有值
//    int insertGateway(Points record);
//
//    //插入gatewany表不为空的值
//    int insertGatewaySelective(Points record);
//
    //根据gatewany表的主键id查询对应那一行的信息
    Points selectGatewayById(Integer id);

    //根据station表的主键id查询对应那一行的信息
    Points selectStationById(Integer id);

    //根据sensor表的主键id查询对应那一行的信息
    Points selectSensorById(Integer id);
//
//    //根据id更新gatewany表(动态)
//    int updateGatewayByIdSelective(Points record);
//
//    //根据id更新gatewany表
//    int updateGatewayById(Points record);

    //动态表插入数据
    int insertDynamic(@Param("tableName") String tableName,
                      @Param("record") Points record);

    //清空Sensor表
    void truncateSensor();

    //清空交叉路口表
    void truncateGateWays();

    //清空服务站
    void truncateStation();

    //创建一个与现有表结构完全相同的新表，但是不复制数据
    void createTableLike(String newTableName, String originalTableName);

    //将原表中的所有数据复制到新表中
    void copyDataToNewTable(String newTableName, String originalTableName);

    //删除数据库表 的DDL操作
    void dropTable(String tableName);

    // 新增的分批方法
    int transferDataBatch(@Param("tableName") String tableName,
                          @Param("name") String name,
                          @Param("offset") int offset,
                          @Param("batchSize") int batchSize);

    // 新增的获取记录数方法
    int getTableCount(@Param("tableName") String tableName,
                      @Param("name") String name);

    //一次性将整个表的数据转移到主表
    void transferData(String tableName, String name);

    //查询sensor表的所有列
    List<Points> selectAllSensors();

//    //查询gateway表的所有列
//    List<Points> selectAllGateways();

    //查询gateWay表的所有列
    List<Points> selectAllGateWays();

    //重命名表名
    void renameTable(@Param("oldTableName") String oldTableName, @Param("newTableName") String newTableName);

    //查询所有station表的所有
    List<Points> selectAllStations();

}