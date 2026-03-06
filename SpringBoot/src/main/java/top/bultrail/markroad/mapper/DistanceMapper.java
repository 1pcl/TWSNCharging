package top.bultrail.markroad.mapper;

import org.apache.ibatis.annotations.Param;
import org.springframework.stereotype.Repository;
import top.bultrail.markroad.entity.DistanceNode;

import java.util.List;

@Repository
public interface DistanceMapper {

    int batchInsertDistance(List<DistanceNode> DistanceNode);
    void truncateDistance();
    List<DistanceNode> selectAllDistance();

    List<DistanceNode> selectDistanceWithLimit(@Param("limit") int limit);
    List<DistanceNode> selectDistanceBatch(@Param("limit") int limit, @Param("offset") int offset);
    int countAllDistance();
    DistanceNode selectDistanceByNodes(
            @Param("startId") Integer startId,
            @Param("endId") Integer endId
    );
}
