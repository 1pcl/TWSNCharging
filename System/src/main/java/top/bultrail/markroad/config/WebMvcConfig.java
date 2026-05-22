package top.bultrail.markroad.config;

        import org.apache.http.client.config.RequestConfig;
        import org.apache.http.impl.client.CloseableHttpClient;
        import org.apache.http.impl.client.HttpClientBuilder;
        import org.springframework.beans.factory.annotation.Value;
        import org.springframework.context.annotation.Bean;
        import org.springframework.context.annotation.Configuration;
        import org.springframework.http.client.HttpComponentsClientHttpRequestFactory;
        import org.springframework.web.client.RestTemplate;
        import org.springframework.web.servlet.config.annotation.CorsRegistry;
        import org.springframework.web.servlet.config.annotation.ResourceHandlerRegistry;
        import org.springframework.web.servlet.config.annotation.ViewControllerRegistry;
        import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

@Configuration
public class WebMvcConfig implements WebMvcConfigurer {


    @Value("${frontend.dev.url:http://localhost:3000}")
    private String frontendDevUrl;

    /**
     * 【极端配置】支持运行几天不超时的 RestTemplate
     * 设置为0表示永不超时
     */
    @Bean
    public RestTemplate restTemplate() {
        // 设置为0表示永不超时（极端配置）
        RequestConfig config = RequestConfig.custom()
                .setConnectTimeout(0)                    // 连接永不超时
                .setSocketTimeout(0)                     // Socket永不超时
                .setConnectionRequestTimeout(0)          // 请求永不超时
                .build();

        // 配置连接池，支持大量长时间连接
        CloseableHttpClient httpClient = HttpClientBuilder.create()
                .setDefaultRequestConfig(config)
                .setMaxConnTotal(200)                    // 最大连接数增加到200
                .setMaxConnPerRoute(100)                 // 每个路由最大连接数100
                .disableConnectionState()                // 禁用连接状态检查
                .build();

        HttpComponentsClientHttpRequestFactory requestFactory =
                new HttpComponentsClientHttpRequestFactory(httpClient);
        requestFactory.setConnectTimeout(0);             // 连接永不超时
        requestFactory.setReadTimeout(0);                // 读取永不超时

        System.out.println("🔧 RestTemplate 已配置为永不超时模式，支持长时间批量计算");

        return new RestTemplate(requestFactory);
    }


    @Override
    public void addViewControllers(ViewControllerRegistry registry) {
        // 开发环境：重定向到前端开发服务器
        // 生产环境：如果需要，可以保留forward到静态资源
        // registry.addViewController("/").setViewName("forward:/index.html");
    }

    @Override
    public void addCorsMappings(CorsRegistry registry) {
        // 配置跨域，允许前端开发服务器访问
        registry.addMapping("/api/**")
                .allowedOrigins(frontendDevUrl, "http://localhost:3000")
                .allowedMethods("GET", "POST", "PUT", "DELETE", "OPTIONS")
                .allowedHeaders("*")
                .allowCredentials(true)
                .maxAge(3600); // 预检请求缓存1小时;
    }

    @Override
    public void addResourceHandlers(ResourceHandlerRegistry registry) {
        // 开发环境不需要配置静态资源，由Vite开发服务器处理
        // 生产环境可以在这里配置静态资源路径
        // registry.addResourceHandler("/**")
        //         .addResourceLocations("classpath:/static/");
    }
}
