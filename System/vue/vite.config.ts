import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import AutoImport from 'unplugin-auto-import/vite'
import Components from 'unplugin-vue-components/vite'
import { ElementPlusResolver } from 'unplugin-vue-components/resolvers'

// https://vitejs.dev/config/
export default defineConfig({
    plugins: [
        vue(),
        AutoImport({
            resolvers: [ElementPlusResolver()],
        }),
        Components({
            resolvers: [ElementPlusResolver()],
        }),
    ],
    base: './',
    // 开发服务器配置
    server: {
        port: 3000, // 前端开发服务器端口
        host: '0.0.0.0', // 允许外部访问
        open: true, // 自动打开浏览器
        cors: true, // 允许跨域
        // 代理配置
        proxy: {
            '/api': {
                target: 'http://localhost:8082', // 后端API地址
                changeOrigin: true,
                secure: false,
                // 如果需要重写路径
                // rewrite: (path) => path.replace(/^\/api/, '')
                // 🔥 添加这些配置解决超时问题
                timeout: 0,                    // 代理永不超时
                proxyTimeout: 0,               // 上游请求永不超时
                configure: (proxy, options) => {
                    proxy.on('error', (err, req, res) => {
                        console.log('代理错误', err);
                    });
                    proxy.on('proxyReq', (proxyReq, req, res) => {
                        console.log('发送请求到目标:', req.method, req.url);
                        proxyReq.setTimeout(0);  // 设置请求永不超时
                    });
                }
            },
        }
    },
    // 构建配置
    build: {
        outDir: 'dist', // 构建输出目录
        assetsDir: 'assets', // 静态资源目录
        sourcemap: false, // 生产环境关闭sourcemap
    }
})