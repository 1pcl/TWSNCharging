package top.bultrail.markroad.mapper;

import org.apache.ibatis.annotations.Param;
import org.springframework.stereotype.Repository;

import org.springframework.stereotype.Repository;
import top.bultrail.markroad.entity.DatasetInfo;
import top.bultrail.markroad.entity.DatasetLocation;
import top.bultrail.markroad.entity.DatasetName;

import java.util.List;

@Repository
public interface DatasetNameMapper {

    //通过关键字删除改行
    int deleteByPrimaryKey(String name);

    //增加
    int insert(DatasetName record);

    int insertSelective(DatasetName record);

    void insertDatasetInfo(String setName, Double locationLng, Double locationLat);

    DatasetName selectByPrimaryKey(String name);

    int updateByPrimaryKeySelective(DatasetName record);

    int updateByPrimaryKey(DatasetName record);

    List<DatasetInfo> selectDatasetInfo();

    DatasetLocation selectDatasetLocation(String setName);

    // 添加一个方法来更新数据集名称
    void updateDatasetName(String oldName, String newName);

    // 检查特定名称的数据集是否存在
    boolean existsByName(String name);

    // 查询所有 is_dataset 等于 1 的数据集名称列表
    List<String> selectNamesByIsDataset();

    // 根据名称更新 is_dataset 的状态值 (0 或 1)
    int updateIsDatasetStatus(@Param("name") String name, @Param("status") Integer status);
}