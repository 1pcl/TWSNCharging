package top.bultrail.markroad.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;
import top.bultrail.markroad.entity.ResultEntity;
import top.bultrail.markroad.service.WSNSimulationService;

import javax.servlet.http.HttpSession;
import java.util.HashMap;
import java.util.Map;

@Controller
public class WSNController {

    @Autowired
    private WSNSimulationService wsnSimulationService;

    /**
     * 运行WSN仿真
     * 映射到 /api/runWSN
     *
     * @param session HTTP会话，用于存储仿真结果
     * @param params 可选的仿真参数
     * @return 仿真结果
     */
    @RequestMapping(value = {"/runWSN", "/api/runWSN"}, method = RequestMethod.POST)
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runWSN(
            HttpSession session,
            @RequestBody(required = false) Map<String, Object> params) {

        try {
            System.out.println("========== 开始执行WSN仿真 ==========");
            long startTime = System.currentTimeMillis();

            // 调用服务执行WSN仿真
            Map<String, Object> simulationResult = wsnSimulationService.runWSNSimulation(params);

            // 将结果存入Session，供后续算法使用
            session.setAttribute("LAST_WSN_DATA", simulationResult);

            long endTime = System.currentTimeMillis();
            System.out.println("WSN仿真完成，耗时: " + (endTime - startTime) + "ms");

            // 打印统计信息
            Map<String, Object> stats = (Map<String, Object>) simulationResult.get("stats");
            if (stats != null) {
                System.out.println("仿真统计: 需要充电的传感器=" + stats.get("charging_nodes") +
                        ", 总传感器=" + stats.get("sensors") +
                        ", 总中继=" + stats.get("relays"));
            }

            // 返回成功结果 - 使用 sucessWithData（注意拼写）
            return ResponseEntity.ok(ResultEntity.sucessWithData(simulationResult));

        } catch (Exception e) {
            System.err.println("WSN仿真失败: " + e.getMessage());
            e.printStackTrace();

            // 返回错误结果
            return ResponseEntity.ok(ResultEntity.failedWithMessage("WSN仿真失败: " + e.getMessage()));
        }
    }

    /**
     * 运行WSN仿真（GET方式，使用默认参数）
     */
    @RequestMapping(value = {"/runWSN", "/api/runWSN"}, method = RequestMethod.GET)
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> runWSN(HttpSession session) {
        return runWSN(session, null);
    }

    /**
     * 获取当前Session中保存的WSN仿真数据
     */
    @RequestMapping(value = {"/getWSNData", "/api/getWSNData"}, method = RequestMethod.GET)
    @ResponseBody
    public ResponseEntity<ResultEntity<Map<String, Object>>> getWSNData(HttpSession session) {
        try {
            @SuppressWarnings("unchecked")
            Map<String, Object> wsnData = (Map<String, Object>) session.getAttribute("LAST_WSN_DATA");

            if (wsnData == null) {
                return ResponseEntity.ok(ResultEntity.failedWithMessage("没有可用的WSN数据，请先运行WSN仿真"));
            }

            // 返回成功结果 - 使用 sucessWithData（注意拼写）
            return ResponseEntity.ok(ResultEntity.sucessWithData(wsnData));

        } catch (Exception e) {
            return ResponseEntity.ok(ResultEntity.failedWithMessage(e.getMessage()));
        }
    }
}