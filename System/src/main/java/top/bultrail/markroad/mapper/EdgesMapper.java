package top.bultrail.markroad.mapper;

import org.apache.ibatis.annotations.Param;
import org.springframework.stereotype.Repository;
import top.bultrail.markroad.entity.Edges;

import java.util.List;

@Repository
public interface EdgesMapper {

    /**
     * 清空所有边
     */
    void truncateEdges();

    /**
     * 查询所有边
     */
    List<Edges> selectAllEdges();

    /**
     * 根据边ID查询
     */
    Edges selectEdgeById(@Param("edgeId") String edgeId);

    /**
     * 根据起终点节点查询边
     */
    Edges selectEdgeByNodes(@Param("node1") String node1, @Param("node2") String node2);

    /**
     * 查询某个节点的所有邻居
     */
    List<Edges> selectNeighborsByNode(@Param("nodeId") String nodeId);

    /**
     * 插入单条边
     */
    int insertEdge(Edges edge);

    /**
     * 批量插入边
     */
    int batchInsertEdges(List<Edges> edges);

    /**
     * 根据边ID删除
     */
    int deleteEdgeById(@Param("edgeId") String edgeId);

    /**
     * 根据节点删除相关边
     */
    int deleteEdgesByNode(@Param("nodeId") String nodeId);

    /**
     * 统计边数量
     */
    int countEdges();

    /**
     * 统计总道路长度（米）
     */
    Double sumTotalLength();

}