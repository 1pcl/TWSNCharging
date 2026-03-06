@echo off
echo GraphHopper config...

set PATH=C:\Program Files\Java\jdk1.8.0_331\bin;%PATH%

set GH_GRAPH_LOCATION=graph-cache
set GH_DATAREADER_FILE=china-latest.osm.pbf

echo Java version:
java -version

echo starting...
java -Xmx4g -Xms2g -jar graphhopper-3.2.jar server config.yml

pause