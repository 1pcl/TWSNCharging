package top.bultrail.markroad.config; // 假设这是你的配置包名，与 WebMvcConfig 放在一起

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class JacksonSnakeCaseConfig {

    /**
     * 定义一个专用于 Python 通信的 ObjectMapper Bean。
     * 强制其使用 SNAKE_CASE 命名策略，独立于全局配置。
     */
    @Bean("pythonObjectMapper") // 使用 @Qualifier 引用这个名字
    public ObjectMapper pythonObjectMapper() {
        // 创建一个新的 ObjectMapper 实例，并配置为 SNAKE_CASE
        return new ObjectMapper().setPropertyNamingStrategy(
                PropertyNamingStrategies.SNAKE_CASE
        );
    }
}