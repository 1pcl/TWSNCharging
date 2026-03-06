@echo off
echo GraphHopper config...

:: 临时设置 PATH，让 Java 8 优先
set PATH=C:\Program Files\Java\jdk1.8.0_331\bin;%PATH%

echo Java version:
java -version

echo starting...

:: *** 关键修改：使用 'import config.yml' 导入数据 ***
:: 'import' 命令会从 config.yml 中读取 china-latest.osm.pbf 和 graph-cache 的路径。
java -Xmx4g -Xms2g -jar graphhopper-3.2.jar import config.yml

:: 如果导入成功，接下来再使用 server 命令启动服务
if exist graph-cache\properties (
    echo Graph data imported successfully. Starting server...
    java -Xmx4g -Xms2g -jar graphhopper-3.2.jar server config.yml
) else (
    echo Graph data import failed! Check logs above.
)

pause