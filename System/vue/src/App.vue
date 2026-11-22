<template>
    <div class="common-layout">
        <el-container>
            <el-main>
                <div id="map"></div>
                <!-- 添加地图背景淡化控制按钮 -->
                <div class="map-controls">
                    <el-button
                            :type="mapBgDimmed ? 'primary' : 'default'"
                            @click="toggleMapDim"
                            circle
                            :title="mapBgDimmed ? '恢复地图亮度' : '淡化地图背景'"
                    >
                        <el-icon>
                            <Sunny v-if="mapBgDimmed" />
                            <Cloudy v-else />
                        </el-icon>
                    </el-button>
                </div>
            </el-main>
            <el-footer>
                <div style="display: flex; justify-content: center;" v-if="!ifmark">
                    <el-form :inline="true" class="demo-form-inline">
                        <el-form-item label="Current Zoom">
                            {{ currentZoomLevel }}
                        </el-form-item>
                        <el-form-item label="Mark Mode">
                            <el-switch v-model="ifmark" @change="handleSwitchChange"></el-switch>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="danger" @click="formatting">Formatting</el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="warning" @click="handleCalculation" class="yellow-color">Single VUE
                            </el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="warning" @click="handleFilesUpload" class="yellow-color">Files Upload
                            </el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="warning" @click="handleFilesDownload" class="yellow-color">Files Download
                            </el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="success" @click="handleDatasetLoad" class="green-color">Dataset</el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="success" @click="handleTestUpload" class="green-color">TestData Load
                            </el-button>
                        </el-form-item>
                        <el-form-item label="Chosen Number">
                            {{ chosenNumberStr }}
                        </el-form-item>
                    </el-form>
                </div>
                <div style="display: flex; justify-content: center;" v-if="ifmark">
                    <el-form :inline="true" class="demo-form-inline">
                        <el-form-item label="Stations">
                            {{ stationsValue }}
                        </el-form-item>
                        <el-form-item label="Sensors">
                            {{ sensorsValue }}
                        </el-form-item>
                        <el-form-item label="Relay">
                            {{ relayValue }}
                        </el-form-item>
                        <el-form-item label="Mark Mode">
                            <el-switch v-model="ifmark" @change="handleSwitchChange"></el-switch>
                        </el-form-item>
                        <el-form-item label="NodeType">
                            <el-select v-model="nodeType" value-key="id" placeholder="Select" placement="top-start"
                                       :popper-append-to-body="false" :teleported="false" popper-class="custom-popper"
                                       :disabled="nodeTypeIsDisabled" @change="handleNodeTypeChange" style="width: 200px; min-width: 200px;">
                                <el-option v-for="item in options" :key="item.id" :label="item.label" :value="item"/>
                            </el-select>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="warning" @click="undo" class="yellow-color">Undo</el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="danger" @click="formatting">Formatting</el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="primary" @click="saveData">Save Data</el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="primary" @click="saveDataset" :disabled="saveDatasetIsDisabled">Save
                                Dataset
                            </el-button>
                        </el-form-item>
                        <el-form-item>
                            <el-button type="success" @click="handleDatasetLoad" class="green-color">Dataset</el-button>
                        </el-form-item>
                    </el-form>
                </div>
            </el-footer>
        </el-container>
        <el-dialog title="Single VUE" v-model="caculate_dialogVisible" width="80%">
            <div style="display: flex; justify-content: center;">
                <el-tooltip class="box-item" effect="dark" content="TWSN models" placement="top">
                    <el-button @click="TWSN_models">TWSN models</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="single_greed" placement="top">
                    <el-button @click="single_greed">Greed</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="single_RAWCS" placement="top">
                    <el-button @click="single_RAWCS">RAWCS</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="single_ADRL" placement="top">
                    <el-button @click="single_ADRL">ADRL</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="NSGA Algorithm" placement="top">
                    <el-button @click="single_NSGA">NSGA</el-button>
                </el-tooltip>
            </div>
        </el-dialog>
        <el-dialog title="Upload" v-model="upload_dialogVisible" width="50%">
            <div style="display: flex; justify-content: center;">
                <el-button @click="sensorupload">Sensor Upload</el-button>
                <el-button @click="relayupload">relay Upload</el-button>
                <el-button @click="stationupload">Station Upload</el-button>
                <el-button @click="roadupload">Roads Upload</el-button>
                <el-button @click="loadIntersections">Intersections Upload</el-button>
            </div>
        </el-dialog>
        <el-dialog title="Download" v-model="download_dialogVisible" width="50%">
            <div style="display: flex; justify-content: center;">
                <el-button @click="sensordownload">Sensor Download</el-button>
                <el-button @click="relaydownload">relay Download</el-button>
                <el-button @click="stationdownload">Station Download</el-button>
                <el-button @click="downloadIntersections">Intersections Download</el-button>
                <el-button @click="downloadRoads">Roads Download</el-button>
                <el-button @click="chargingSchedulingdownload">Charging Scheduling Download</el-button>
            </div>
        </el-dialog>
        <el-dialog title="Which test data would you like to upload?" v-model="TestData_dialogVisible" width="50%">
            <div style="display: flex; justify-content: center;">
                <el-button type="primary" @click="handleLoad('RN373')">RN373</el-button>
                <el-button type="primary" @click="handleLoad('RN446')">RN446</el-button>
                <el-button type="primary" @click="handleLoad('RN528')">RN528</el-button>
                <el-button type="primary" @click="handleLoad('RN629')">RN629</el-button>
            </div>
        </el-dialog>
        <el-dialog title="Which dataset would you like to upload?" v-model="Dataset_dialogVisible" width="45%">
            <el-table :data="setNames" height="250" style="width: 100%">
                <el-table-column type="index" width="50"/>
                <el-table-column label="Name" width="150" prop="name"/>
                <el-table-column label="Sensor Size" width="100" prop="sensorSize"/>
                <el-table-column label="Station Size" width="130" prop="stationSize"/>
                <el-table-column label="relay Size" width="130" prop="relaySize"/>
                <el-table-column label="Operations">
                    <template #default="scope">
                        <el-button size="small" @click="handleLoad(scope.row.name)">Load</el-button>
                        <el-button size="small" type="danger" @click="handleDelete(scope.$index, scope.row.name)"
                                   :disabled="isRestrictedDataset(scope.row.name)">Delete
                        </el-button>
                        <el-button size="small" @click="renameDataset(scope.row, scope.$index)"
                                   :disabled="isRestrictedDataset(scope.row.name)">Rename
                        </el-button>
                    </template>
                </el-table-column>
            </el-table>
        </el-dialog>
        <el-dialog title="Select the data set operation you want to perform" v-model="AboutDatasetdialogVisible"
                   width="45%">
            <div style="display: flex; justify-content: center;">
                <el-button type="success" @click="handleDatasetLoad" class="green-color">Dataset Load</el-button>
                <el-button type="danger" @click="deleteDataset">Delete Dataset</el-button>
            </div>
        </el-dialog>
    </div>
</template>

<script setup lang="ts">
    import {ref, onMounted, computed, shallowRef, nextTick} from 'vue';
    // 导入Element Plus图标
    import { Sunny, Cloudy } from '@element-plus/icons-vue';
    //@ts-ignore
    import axios from 'axios';
    import { ElLoading, ElMessage, ElMessageBox } from 'element-plus';
    import 'ol/ol.css';
    import Map from 'ol/Map';
    import View from 'ol/View';
    import TileLayer from 'ol/layer/Tile';
    import OSM from 'ol/source/OSM';
    import {fromLonLat, toLonLat} from 'ol/proj';
    import {Icon, Style} from 'ol/style';
    import Text from 'ol/style/Text';
    import Feature from 'ol/Feature';
    import Point from 'ol/geom/Point';
    import VectorSource from 'ol/source/Vector';
    import VectorLayer from 'ol/layer/Vector';
    import {defaults as defaultControls, ScaleLine} from 'ol/control';
    import {defaults as defaultInteractions} from 'ol/interaction';
    import {Coordinate} from 'ol/coordinate';
    import {Fill, Stroke} from 'ol/style';
    import {LineString} from 'ol/geom';
    import {getDistance} from 'ol/sphere';
    import MouseWheelZoom from 'ol/interaction/MouseWheelZoom';

    // 图片资源
    const cIconSrc = "./assets/icon/green.png";
    const cStaionSrc = "./assets/icon/mycar.png";

    // 图片资源（蓝色是原本的，绿色是充好电的，红色是没电的）
    const sensorBlue="./assets/icon/sensorBlue.png";
    const sensorGreen="./assets/icon/sensorGreen.png";
    const sensorRed="./assets/icon/sensorRed.png";
    const relayBlue="./assets/icon/gatewayBlue.png";
    const relayGreen="./assets/icon/gatewayGreen.png";
    const relayRed="./assets/icon/gatewayRed.png";

    // 地图相关变量 - 性能优化的关键是复用 Layer 和 Source
    let map: any;

    // 定义各个类型的 Source，避免重复创建
    const sensorSource = new VectorSource();
    const relaySource = new VectorSource();
    const stationSource = new VectorSource();
    const roadSource = new VectorSource();           // 道路线图层
    const roadPointSource = new VectorSource();      // 道路端点（起点终点图标）
    const tempMarkSource = new VectorSource(); // 用于临时的标记（如正在画的线）
    const pathSource = new VectorSource(); // 用于路径规划
    const schedulePathSource = new VectorSource(); // 调度路径
    const scheduleMarkerSource = new VectorSource(); // 调度标记
    const intersectionSource = new VectorSource();

    // 定义 Layer
    let sensorLayer: VectorLayer<VectorSource>;
    let relayLayer: VectorLayer<VectorSource>;
    let stationLayer: VectorLayer<VectorSource>;
    let roadLayer: VectorLayer<VectorSource>;
    let roadPointLayer: VectorLayer<VectorSource>;
    let tempMarkLayer: VectorLayer<VectorSource>;
    let pathLayer: VectorLayer<VectorSource>;
    let schedulePathLayer: VectorLayer<VectorSource>;
    let scheduleMarkerLayer: VectorLayer<VectorSource>;
    let intersectionLayer: VectorLayer<VectorSource>;

    // 添加地图背景淡化控制变量
    const mapBgDimmed = ref(false);
    let originalOSMLayer: TileLayer<OSM> | null = null;

    const createMap = () => {
        // 初始化图层
        sensorLayer = new VectorLayer({ source: sensorSource, zIndex: 5 });
        relayLayer = new VectorLayer({ source: relaySource, zIndex: 6 });
        stationLayer = new VectorLayer({ source: stationSource, zIndex: 7 });
        roadLayer = new VectorLayer({ source: roadSource, zIndex: 4 });
        roadPointLayer = new VectorLayer({ source: roadPointSource, zIndex: 5 });
        tempMarkLayer = new VectorLayer({ source: tempMarkSource, zIndex: 10 });
        pathLayer = new VectorLayer({ source: pathSource, zIndex: 3 });
        schedulePathLayer = new VectorLayer({ source: schedulePathSource, zIndex: 8 });
        scheduleMarkerLayer = new VectorLayer({ source: scheduleMarkerSource, zIndex: 9 });
        intersectionLayer = new VectorLayer({ source: intersectionSource, zIndex: 4 });

        // 创建OSM图层并保存引用
        const osmLayer = new TileLayer({
            source: new OSM()
        });
        originalOSMLayer = osmLayer; // 保存引用

        map = new Map({
            target: 'map',
            interactions: defaultInteractions({
                doubleClickZoom: false,
                mouseWheelZoom: false,
            }),
            layers: [
                osmLayer,
                pathLayer,
                roadLayer,
                roadPointLayer,
                intersectionLayer,
                sensorLayer,
                relayLayer,
                stationLayer,
                schedulePathLayer,
                scheduleMarkerLayer,
                tempMarkLayer
            ],
            view: new View({
                center: fromLonLat([113.34823, 23.15829]),
                zoom: 16,
                minZoom: 10,
                maxZoom: 22,
            }),
            controls: defaultControls({
                attribution: false
            }).extend([
                new ScaleLine(),
            ]),
        });

        map.on('moveend', () => {
            currentZoomLevel.value = map.getView().getZoom();
        });
        currentZoomLevel.value = map.getView().getZoom();

        // 添加点击交互
        map.on('click', (evt) => {
            const feature = map.forEachFeatureAtPixel(evt.pixel, (f) => f);
            if (feature && feature.get('isStepMarker')) {
                const detail = feature.get('detail');
                ElMessageBox.alert(detail, 'Step Details', {
                    confirmButtonText: 'OK',
                    callback: () => {}
                });
            }
        });
    }

    // 切换地图背景淡化
    const toggleMapDim = () => {
        mapBgDimmed.value = !mapBgDimmed.value;
        if (!map) {
            ElMessage.warning('地图还未初始化完成');
            return;
        }
        if (originalOSMLayer) {
            originalOSMLayer.setOpacity(mapBgDimmed.value ? 0.2 : 1);
            map.render();
            ElMessage.success(mapBgDimmed.value ? '地图背景已淡化' : '地图背景已恢复');
        } else {
            ElMessage.warning('无法找到地图图层');
        }
    };

    // 辅助函数：创建一个带图标的 Feature
    const createIconFeature = (iconSrc: string, size: number[], position: Coordinate, anchorvalue = { x: 0.5, y: 0.5 }) => {
        const iconFeature = new Feature({
            geometry: new Point(fromLonLat(position)),
        });
        iconFeature.setStyle(new Style({
            image: new Icon({
                anchor: [anchorvalue.x, anchorvalue.y],
                src: iconSrc,
                size: size,
                scale: [1, 1],
            }),
        }));
        return iconFeature;
    };

    const addSingleIconToSource = (source: VectorSource, iconSrc: string, size: number[], position: Coordinate) => {
        const feature = createIconFeature(iconSrc, size, position);
        source.addFeature(feature);
    }

    // 绘制道路线 - 黑色粗虚线
    const drawRoadLine = (start: any, end: any) => {
        const startCoord = fromLonLat(start);
        const endCoord = fromLonLat(end);

        const lineFeature = new Feature({
            geometry: new LineString([startCoord, endCoord]),
            roadStart: start,
            roadEnd: end
        });

        lineFeature.setStyle(new Style({
            stroke: new Stroke({
                color: '#000000',
                width: 5,
                lineDash: [12, 10]
            })
        }));

        roadSource.addFeature(lineFeature);
    }

    // 响应式变量
    const all_sensor = shallowRef<any[][]>([]);
    const all_station = shallowRef<any[][]>([]);
    const all_relay = shallowRef<any[][]>([]);
    const chosenNumber = ref(0);
    const currentZoomLevel = ref(19);

    let roads: any[] = [];
    let pointArr: any[] = [];
    let pts: any[] = [];
    let pointIndex = 0;
    let isDrawingRoad = false;
    let isContinuousMode = false;
    let currentRoadStart: any = null;
    let currentRoadEnd: any = null;

    const ifmark = ref(false);
    const caculate_dialogVisible = ref(false);
    const upload_dialogVisible = ref(false);
    const download_dialogVisible = ref(false);
    const TestData_dialogVisible = ref(false);
    const Dataset_dialogVisible = ref(false);
    const AboutDatasetdialogVisible = ref(false);

    const setNames = ref<Array<{ name: string; sensorSize: number; stationSize:number;relaySize:number }>>([]);
    const nodeTypeIsDisabled = computed(() => !ifmark.value);
    const saveDatasetIsDisabled = computed(() => !ifmark.value);

    const stationsValue = computed(() => {
        let total = 0;
        all_station.value.forEach(row => total += row.length);
        return total;
    });

    const sensorsValue = computed(() => {
        let total = 0;
        all_sensor.value.forEach(row => total += row.length);
        return total;
    });

    const relayValue = computed(() => {
        let total = 0;
        all_relay.value.forEach(row => total += row.length);
        return total;
    });

    const chosenNumberStr = computed(() => chosenNumber.value === 0 ? "_ _" : chosenNumber.value);

    const isRestrictedDataset = (name: string) => ['RN373', 'RN446', 'RN528','RN629'].includes(name);

    type Option = { id: number, label: string, desc: string }
    const nodeType = ref<Option>({id: 1, label: 'station', desc: '服务站点'})
    const options = ref([
        {id: 1, label: 'station', desc: '服务站点'},
        {id: 2, label: 'sensor', desc: '路段传感器标记'},
        {id: 3, label: 'sensor_single', desc: '单点传感器标记'},
        {id: 4, label: 'road_continuous', desc: '道路绘制(连续)'},
        {id: 5, label: 'road_single', desc: '道路绘制(单点)'},
        {id: 6, label: 'relay', desc: '路段网关标记'},
        {id: 7, label: 'relay_single', desc: '单点网关标记'},
        {id: 8, label: 'None', desc: '无标记'},
    ])

    // 清除所有 Source 的数据
    function clearVectorLayers() {
        sensorSource.clear();
        relaySource.clear();
        stationSource.clear();
        roadSource.clear();
        roadPointSource.clear();
        intersectionSource.clear();
        tempMarkSource.clear();
        pathSource.clear();
        schedulePathSource.clear();
        scheduleMarkerSource.clear();
    }

    // 定义操作历史记录项的类型
    interface HistoryItem {
        type: 'sensor' | 'relay' | 'station' | 'road' | 'pointArr';
        data: any;
        timestamp: number;
    }

    // 历史栈
    let historyStack: HistoryItem[] = [];
    let guard_undo: string;

    // 添加历史记录
    function addToHistory(type: HistoryItem['type'], data: any) {
        historyStack.push({
            type,
            data,
            timestamp: Date.now()
        });
        if (historyStack.length > 100) {
            historyStack.shift();
        }
    }

    // 专门重绘道路的函数
    function redrawAllRoads() {
        roadSource.clear();
        roadPointSource.clear();

        roads.forEach(road => {
            drawRoadLine(road.startPoint, road.endPoint);
            addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], road.startPoint);
            addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], road.endPoint);
        });

        // 只有当正在绘制道路且没有道路时，才需要绘制当前起点
        if (currentRoadStart !== null && isDrawingRoad && roads.length === 0) {
            addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], currentRoadStart);
        }
    }

    // 专门重绘传感器的函数
    function redrawAllSensors() {
        sensorSource.clear();
        const sensorFeatures: any[] = [];
        all_sensor.value.forEach(group => {
            group.forEach(pt => sensorFeatures.push(createIconFeature(sensorBlue, [16, 16], pt)));
        });
        sensorSource.addFeatures(sensorFeatures);
    }

    // 专门重绘中继的函数
    function redrawAllRelays() {
        relaySource.clear();
        const relayFeatures: any[] = [];
        all_relay.value.forEach(group => {
            group.forEach(pt => relayFeatures.push(createIconFeature(relayBlue, [64, 64], pt)));
        });
        relaySource.addFeatures(relayFeatures);
    }

    // 专门重绘服务站的函数
    function redrawAllStations() {
        stationSource.clear();
        const stationFeatures: any[] = [];
        all_station.value.forEach(group => {
            group.forEach(pt => stationFeatures.push(createIconFeature(cStaionSrc, [128, 128], pt)));
        });
        stationSource.addFeatures(stationFeatures);
    }

    // 全量重绘
    function drawAllMarked() {
        roadSource.clear();
        roadPointSource.clear();
        relaySource.clear();
        sensorSource.clear();
        stationSource.clear();
        tempMarkSource.clear();

        roads.forEach(road => {
            drawRoadLine(road.startPoint, road.endPoint);
            addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], road.startPoint);
            addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], road.endPoint);
        });

        const relayFeatures: any[] = [];
        all_relay.value.forEach(group => {
            group.forEach(pt => relayFeatures.push(createIconFeature(relayBlue, [64, 64], pt)));
        });
        relaySource.addFeatures(relayFeatures);

        const sensorFeatures: any[] = [];
        all_sensor.value.forEach(group => {
            group.forEach(pt => sensorFeatures.push(createIconFeature(sensorBlue, [16, 16], pt)));
        });
        sensorSource.addFeatures(sensorFeatures);

        const stationFeatures: any[] = [];
        all_station.value.forEach(group => {
            group.forEach(pt => stationFeatures.push(createIconFeature(cStaionSrc, [128, 128], pt)));
        });
        stationSource.addFeatures(stationFeatures);
    }

    // undo 函数
    const undo = () => {
        if (guard_undo === "road_start" || guard_undo === "road_end") {
            if (guard_undo === "road_start") {
                currentRoadStart = null;
                roadPointSource.clear();
                if (roads.length > 0) {
                    redrawAllRoads();
                }
            } else if (guard_undo === "road_end") {
                const features = roadPointSource.getFeatures();
                if (features.length > 0) {
                    roadPointSource.removeFeature(features[features.length - 1]);
                }
                currentRoadEnd = null;
                if (currentRoadStart !== null) {
                    const hasStart = roadPointSource.getFeatures().some(f => {
                        const geom = f.getGeometry();
                        if (geom instanceof Point) {
                            const coords = toLonLat(geom.getCoordinates());
                            return Math.abs(coords[0] - currentRoadStart[0]) < 0.000001 &&
                                Math.abs(coords[1] - currentRoadStart[1]) < 0.000001;
                        }
                        return false;
                    });
                    if (!hasStart) {
                        addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], currentRoadStart);
                    }
                }
            }
            guard_undo = '';
            return;
        }

        if (historyStack.length === 0) {
            ElMessage.info('没有更多可撤销的操作');
            return;
        }

        const lastItem = historyStack.pop()!;

        switch (lastItem.type) {
            case 'sensor': {
                const arr = [...all_sensor.value];
                if (arr.length > 0) {
                    arr.pop();
                    all_sensor.value = arr;
                    redrawAllSensors();
                    ElMessage.success(`已撤销传感器添加`);
                }
                break;
            }
            case 'relay': {
                const arr = [...all_relay.value];
                if (arr.length > 0) {
                    arr.pop();
                    all_relay.value = arr;
                    redrawAllRelays();
                    ElMessage.success(`已撤销中继添加`);
                }
                break;
            }
            case 'station': {
                const arr = [...all_station.value];
                if (arr.length > 0) {
                    arr.pop();
                    all_station.value = arr;
                    redrawAllStations();
                    ElMessage.success(`已撤销服务站添加`);
                }
                break;
            }
            case 'road': {
                if (roads.length > 0) {
                    roads.pop();

                    if (roads.length === 0) {
                        currentRoadStart = null;
                        currentRoadEnd = null;
                        isDrawingRoad = false;
                        roadPointSource.clear();
                    } else {
                        redrawAllRoads();
                        if (isContinuousMode) {
                            const lastRoad = roads[roads.length - 1];
                            currentRoadStart = lastRoad.endPoint;
                            currentRoadEnd = null;
                            addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], currentRoadStart);
                        } else {
                            currentRoadStart = null;
                            currentRoadEnd = null;
                        }
                    }
                    ElMessage.success(`已撤销道路`);
                }
                break;
            }
            case 'pointArr': {
                if (pointArr.length > 0) {
                    pointArr.pop();
                    tempMarkSource.clear();
                    if (pointArr.length === 1) {
                        addSingleIconToSource(tempMarkSource, cIconSrc, [64, 64], pointArr[0]);
                    }
                    ElMessage.success('已撤销临时标记点');
                }
                break;
            }
            default:
                ElMessage.warning('无法撤销此操作');
                break;
        }
    };

    const handleLoad = (name: string) => {
        axios.post('datasetLoad', null, { params: { datasetName: name } })
            .then((response) => {
                all_sensor.value = [];
                all_relay.value = [];
                all_station.value = [];
                roads = [];
                chosenNumber.value = 0;
                pointArr = [];
                pts = [];
                historyStack = [];

                isDrawingRoad = false;
                isContinuousMode = false;
                currentRoadStart = null;
                currentRoadEnd = null;

                clearVectorLayers();

                let location_lng = response.data.data[0];
                let location_lat = response.data.data[1];

                const view = map.getView();
                view.setCenter(fromLonLat([location_lng, location_lat]));
                view.setZoom(19);

                // 并行加载所有数据
                Promise.all([
                    sensorupload(),      // 加载传感器（蓝色小圆点）
                    relayupload(),       // 加载中继（蓝色网关图标）
                    stationupload(),     // 加载服务站（汽车图标）
                    roadupload(),        // 加载道路线（黑色虚线）
                    loadIntersections()  // 加载路口节点（ross.png 图标）
                ]).then(() => {
                    Dataset_dialogVisible.value = false;
                    console.log('所有数据加载完成');
                    ElMessage.success('数据集加载完成');
                }).catch((error) => {
                    console.error('数据加载失败:', error);
                    ElMessage.error('部分数据加载失败');
                });
            })
            .catch(function (error: any) {
                ElMessage({ message: error, type: 'error' })
            });
    }

    const handleDelete = (index: number, name: string) => {
        ElMessageBox.confirm('Are you sure you want to delete the dataset?', 'Warning', { type: 'warning' })
            .then(() => {
                const loading = ElLoading.service({ lock: true, text: 'Loading' })
                axios.post('deleteDataset', null, { params: { datasetName: name } })
                    .then(() => {
                        if (index >= 0 && index < setNames.value.length) {
                            setNames.value.splice(index, 1);
                        }
                        loading.close();
                        ElMessage.success('Dataset deleted successfully')
                    })
                    .catch((error: any) => {
                        loading.close();
                        ElMessage.error(error.response?.data?.message || '请求失败')
                    });
            })
            .catch(() => {})
    }

    const handleSwitchChange = () => {
        if (ifmark.value) {
            map.getTargetElement().style.cursor = 'crosshair';
            map.on('singleclick', handleClick);
            const mouseWheelZoom = new MouseWheelZoom();
            map.addInteraction(mouseWheelZoom);
        } else {
            map.getTargetElement().style.cursor = '';
            map.un('singleclick', handleClick);
            map.getInteractions().getArray()
                .filter((i: any) => i instanceof MouseWheelZoom)
                .forEach((i: any) => map.removeInteraction(i));
            map.getView().setZoom(19);
        }
    };

    function handleNodeTypeChange() {
        var the_nodetype = nodeType.value?.label;

        if (the_nodetype === "road_continuous") {
            // 连续绘制模式
            isDrawingRoad = true;
            isContinuousMode = true;
            // 不要清空 roadPointSource，保留已有道路端点
            currentRoadStart = null;
            currentRoadEnd = null;
        } else if (the_nodetype === "road_single") {
            // 单点绘制模式
            isDrawingRoad = true;
            isContinuousMode = false;
            // 不要清空 roadPointSource，保留已有道路端点
            currentRoadStart = null;
            currentRoadEnd = null;
        } else {
            // 其他模式
            isDrawingRoad = false;
            isContinuousMode = false;
            currentRoadStart = null;
            currentRoadEnd = null;
        }

        map.getTargetElement().style.cursor = (the_nodetype === "None" || the_nodetype === "road_continuous" || the_nodetype === "road_single") ?
            "crosshair" : "url(https://api.map.baidu.com/images/openhand.cur) 8 8, default";
    }

    function handleClick(e: { coordinate: any; }) {
        var coordinate = e.coordinate;
        var lonLat = toLonLat(coordinate);
        var the_nodetype = nodeType.value?.label

        if (the_nodetype === "None") {
            return;
        }
        else if (the_nodetype === "road_continuous" || the_nodetype === "road_single") {
            if (!isDrawingRoad) {
                isDrawingRoad = true;
                isContinuousMode = (the_nodetype === "road_continuous");
                currentRoadStart = null;
                currentRoadEnd = null;
                // 初始化时不清理 roadPointSource，保持现有道路端点
            }

            if (isContinuousMode && currentRoadStart !== null && currentRoadEnd === null) {
                currentRoadEnd = lonLat;
                addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], lonLat);

                const roadLength = getDistance(currentRoadStart, currentRoadEnd);

                const roadData = {
                    startPoint: currentRoadStart,
                    endPoint: currentRoadEnd,
                    lengthM: roadLength,
                    roadType: isContinuousMode ? 'continuous' : 'single'
                };
                roads.push(roadData);
                addToHistory('road', roadData);
                drawRoadLine(currentRoadStart, currentRoadEnd);

                currentRoadStart = currentRoadEnd;
                currentRoadEnd = null;

                const hasStart = roadPointSource.getFeatures().some(f => {
                    const geom = f.getGeometry();
                    if (geom instanceof Point) {
                        const coords = toLonLat(geom.getCoordinates());
                        return Math.abs(coords[0] - currentRoadStart[0]) < 0.000001 &&
                            Math.abs(coords[1] - currentRoadStart[1]) < 0.000001;
                    }
                    return false;
                });
                if (!hasStart) {
                    addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], currentRoadStart);
                }
                return;
            }

            if (currentRoadStart === null) {
                currentRoadStart = lonLat;
                guard_undo = 'road_start';
                addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], lonLat);
            }
            else if (currentRoadEnd === null) {
                currentRoadEnd = lonLat;
                addSingleIconToSource(roadPointSource, cIconSrc, [64, 64], lonLat);

                const roadLength = getDistance(currentRoadStart, currentRoadEnd);

                const roadData = {
                    startPoint: currentRoadStart,
                    endPoint: currentRoadEnd,
                    lengthM: roadLength,
                    roadType: isContinuousMode ? 'continuous' : 'single'
                };
                roads.push(roadData);
                addToHistory('road', roadData);
                drawRoadLine(currentRoadStart, currentRoadEnd);

                if (isContinuousMode) {
                    currentRoadStart = currentRoadEnd;
                    currentRoadEnd = null;
                } else {
                    // 单点模式：重置当前绘制状态，但不清空已存在的道路端点
                    currentRoadStart = null;
                    currentRoadEnd = null;
                }
            }
            return;
        }
        else if (the_nodetype === "relay_single") {
            const newRelay = [lonLat];
            all_relay.value = [...all_relay.value, newRelay];
            addToHistory('relay', newRelay);
            addSingleIconToSource(relaySource, relayBlue, [64, 64], lonLat);
            ElMessage.success(`已添加中继节点: (${lonLat[0].toFixed(6)}, ${lonLat[1].toFixed(6)})`);
        }
        else if (the_nodetype === "station") {
            const newStation = [lonLat];
            all_station.value = [...all_station.value, newStation];
            addToHistory('station', newStation);
            addSingleIconToSource(stationSource, cStaionSrc, [128, 128], lonLat);
            ElMessage.success(`已添加服务站: (${lonLat[0].toFixed(6)}, ${lonLat[1].toFixed(6)})`);
        }
        else if (the_nodetype === "sensor_single") {
            const newSensor = [lonLat];
            all_sensor.value = [...all_sensor.value, newSensor];
            addToHistory('sensor', newSensor);
            addSingleIconToSource(sensorSource, sensorBlue, [16, 16], lonLat);
            ElMessage.success(`已添加传感器: (${lonLat[0].toFixed(6)}, ${lonLat[1].toFixed(6)})`);
        }
        else {
            pointArr.push(lonLat);
            if (pointArr.length === 1) {
                addSingleIconToSource(tempMarkSource, cIconSrc, [64, 64], pointArr[0]);
                guard_undo = 'pointArr_first';
            }
            if (pointArr.length === 2) {
                tempMarkSource.clear();

                var start = pointArr[0];
                var end = pointArr[1];
                pointArr = []

                var x1 = start[0], y1 = start[1];
                var x2 = end[0], y2 = end[1];

                pointIndex = (the_nodetype == "sensor") ? 50 : 150;
                var chang = getDistance(start, end);

                let tempPts = [];
                for (var i = pointIndex; i < chang; i += pointIndex) {
                    var ratio = i / chang;
                    var x3 = ratio * (x1 - x2) + x2;
                    var y3 = ratio * (y1 - y2) + y2;
                    tempPts.push([x3, y3]);
                }

                if (the_nodetype == "sensor") {
                    all_sensor.value = [...all_sensor.value, tempPts];
                    addToHistory('sensor', tempPts);
                    guard_undo = 'sensor';
                } else {
                    all_relay.value = [...all_relay.value, tempPts];
                    addToHistory('relay', tempPts);
                    guard_undo = 'relay';
                }

                const targetSource = (the_nodetype == "sensor") ? sensorSource : relaySource;
                const targetIcon = (the_nodetype == "sensor") ? sensorBlue : relayBlue;
                const targetSize = (the_nodetype == "sensor") ? [32, 32] : [64, 64];

                const newFeatures = tempPts.map(pt => createIconFeature(targetIcon, targetSize, pt));
                targetSource.addFeatures(newFeatures);

                const pointCount = tempPts.length;
                const typeName = the_nodetype == "sensor" ? "传感器" : "中继";
                ElMessage.success(`已添加${pointCount}个${typeName}节点`);
            }
        }
    }

    const formatting = () => {
        ElMessageBox.confirm('All points marked on the map will be cleared，Continue?', 'Warning', { type: 'warning' })
            .then(() => {
                all_sensor.value = [];
                all_station.value = [];
                all_relay.value = [];
                roads = [];
                chosenNumber.value = 0;
                pointArr = [];
                pts = [];

                isDrawingRoad = false;
                isContinuousMode = false;
                currentRoadStart = null;
                currentRoadEnd = null;
                guard_undo = '';
                historyStack = [];

                clearVectorLayers();

                if (mapBgDimmed.value) {
                    mapBgDimmed.value = false;
                    if (originalOSMLayer) {
                        originalOSMLayer.setOpacity(1);
                    }
                }

                ElMessage.success('All data cleared successfully');
            })
            .catch(() => {})
    };

    const saveData = () => {
        const relay_array = all_relay.value;
        const sensor_array = all_sensor.value;
        const station_array = all_station.value;
        const roads_array = roads;

        // 转换道路数据为后端期望的格式
        const convertedRoads = roads_array.map(road => {
            return [
                [String(road.startPoint[0]), String(road.startPoint[1])],
                [String(road.endPoint[0]), String(road.endPoint[1])],
                [String(road.lengthM)]
            ];
        });

        const quickSaveData = {
            "relay_points": relay_array,
            "sensor_array": sensor_array,
            "station_array": station_array,
            "road_array": convertedRoads,  // 关键修改：字段名必须是 road_array
        };

        console.log('保存的数据:', quickSaveData); // 调试用

        const loading = ElLoading.service({ lock: true, text: 'Saving...' })
        axios.post('hhquicksaveNew', quickSaveData)
            .then(() => {
                loading.close();
                ElMessage.success('保存成功！');
                pointArr = [];
            })
            .catch((error: any) => {
                loading.close();
                console.error('保存失败详情:', error.response?.data);
                const errorMsg = error.response?.data?.message || error.response?.data?.error || '保存失败';
                ElMessage.error(errorMsg);
            });
    };
    const deleteDataset = () => {
        ElMessageBox.prompt('Please enter the name of the dataset to delete', 'Tip', {
            inputPattern: /^[a-zA-Z0-9]{1,8}$/,
        }).then(({value}) => {
            axios.post('deleteDataset', null, { params: { datasetName: value } })
                .then(() => ElMessage.success('Dataset deleted successfully'))
                .catch((e: any) => ElMessage.error(e.response?.data?.message || 'Error'));
        });
    }

    const saveDataset = () => {
        ElMessageBox.prompt('Please set dataset name', 'Tip', {
            inputPattern: /^[a-zA-Z0-9]{1,8}$/,
        }).then(({value}) => {
            const loading = ElLoading.service({ lock: true, text: 'Saving...' });
            let center = toLonLat(map.getView().getCenter());
            const requestData = { datasetName: value, current_location: center };

            axios.post('saveDataset', requestData)
                .then(() => {
                    loading.close();
                    ElMessage.success('Dataset saved successfully');
                })
                .catch((e: any) => {
                    loading.close();
                    ElMessage.error(e.response?.data?.message || 'Error');
                });
        });
    }

    const renameDataset = (row: any, rowIndex: number) => {
        ElMessageBox.prompt('Please enter the new dataset name', `Rename: ${row.name}`, {
            inputPattern: /^[a-zA-Z0-9]{1,8}$/,
        }).then(({value}) => {
            axios.post('renameDataset', { old_name: row.name, new_name: value })
                .then(() => {
                    ElMessage.success('Dataset renamed successfully');
                    if (rowIndex >= 0 && rowIndex < setNames.value.length) {
                        setNames.value[rowIndex].name = value;
                    }
                })
                .catch((e: any) => ElMessage.error(e.response?.data?.message || 'Error'));
        });
    }

    const handleCalculation = () => { caculate_dialogVisible.value = true; };
    const handleFilesUpload = () => { upload_dialogVisible.value = true; };
    const handleFilesDownload= () => { download_dialogVisible.value = true; };
    const handleTestUpload = () => { TestData_dialogVisible.value = true; };

    const handleDatasetLoad = () => {
        axios.post('searchDatasetInfo')
            .then((response: { data: { data: Array<any> } }) => {
                setNames.value = response.data.data.map((d: any) => ({
                    name: d.name,
                    sensorSize: d.sensor_size,
                    stationSize: d.station_size,
                    relaySize: d.relay_size
                }));
                Dataset_dialogVisible.value = true;
            })
            .catch((e: any) => ElMessage.error(e.response?.data?.message || 'Error'));
    };

    const relayupload = async (isAppend = false) => {
        const loading = ElLoading.service({ lock: true, text: 'Loading relay...' })
        try {
            const response = await axios.post('cup');
            const relay = response.data.data;

            console.log('中继数据:', relay); // 调试用

            all_relay.value = [];
            const features: any[] = [];

            for (let i = 0; i < relay.length; i++) {
                // 根据后端返回的数据格式，索引 1 是经度，索引 2 是纬度
                const pos = [relay[i][1], relay[i][2]];  // 修改这里：从 [2] 和 [3] 改为 [1] 和 [2]
                all_relay.value.push([pos]);
                features.push(createIconFeature(relayBlue, [64, 64], pos));
            }

            console.log('all_relay.value:', all_relay.value); // 调试用

            if (!isAppend) {
                relaySource.clear();
            }
            relaySource.addFeatures(features);

            ElMessage.success('中继节点绘制完成');
        } catch (error) {
            console.error(error);
            ElMessage.error(String(error));
        } finally {
            loading.close();
        }
    };

    const roadupload = async () => {
        try {
            const response = await axios.post('loadRoads');
            const loadedRoads = response.data.data;

            console.log('加载的道路线数据:', loadedRoads);

            if (loadedRoads && loadedRoads.length > 0) {
                // 只清空道路线图层，不清空 roadPointSource
                roadSource.clear();

                // 重新绘制所有道路线
                loadedRoads.forEach(road => {
                    if (road.startPoint && road.endPoint) {
                        drawRoadLine(road.startPoint, road.endPoint);

                        roads.push({
                            startPoint: road.startPoint,
                            endPoint: road.endPoint,
                            lengthM: road.lengthM || getDistance(road.startPoint, road.endPoint)
                        });
                    }
                });

                console.log(`成功加载 ${loadedRoads.length} 条道路线`);
                ElMessage.success(`道路线加载完成，共 ${loadedRoads.length} 条`);
            } else {
                console.log('没有道路线数据');
            }
        } catch (error) {
            console.error('加载道路线失败:', error);
            ElMessage.error('加载道路线失败: ' + (error.response?.data?.message || error.message));
        }
    };

    // 加载路口节点（使用 ross.png 图标）
    const loadIntersections = async () => {
        try {
            const response = await axios.post('loadIntersections');
            const intersections = response.data.data;

            console.log('路口数据:', intersections);

            if (intersections && intersections.length > 0) {
                intersectionSource.clear();
                const features: any[] = [];

                for (let i = 0; i < intersections.length; i++) {
                    const lng = parseFloat(intersections[i][1]);
                    const lat = parseFloat(intersections[i][2]);
                    const pos = [lng, lat];

                    // 使用用户画图时的 ross.png 图标
                    features.push(createIconFeature(cIconSrc, [64, 64], pos));
                }

                intersectionSource.addFeatures(features);
                console.log(`加载 ${intersections.length} 个路口节点`);
                ElMessage.success(`路口节点加载完成，共 ${intersections.length} 个`);
            } else {
                console.log('没有路口节点数据');
            }
        } catch (error) {
            console.error('加载路口失败:', error);
            ElMessage.error('加载路口失败: ' + (error.response?.data?.message || error.message));
        }
    };

    // 传感器上传
    const sensorupload = async (isAppend = false) => {
        const loading = ElLoading.service({ lock: true, text: 'Loading Sensor...' });
        try {
            const response = await axios.post('sup');
            const sensors = response.data.data;

            console.log('传感器原始数据:', sensors); // 调试用

            all_sensor.value = [];
            const features: any[] = [];
            const tempSensorData = [];

            for (let i = 0; i < sensors.length; i++) {
                // 数据格式: [id, lng, lat]
                const id = sensors[i][0];
                const lng = parseFloat(sensors[i][1]);  // 经度
                const lat = parseFloat(sensors[i][2]);  // 纬度
                const pos = [lng, lat];

                tempSensorData.push([pos]);
                features.push(createIconFeature(sensorBlue, [16, 16], pos));
            }

            all_sensor.value = tempSensorData;

            if (!isAppend) {
                sensorSource.clear();
            }
            sensorSource.addFeatures(features);

            console.log('传感器数量:', all_sensor.value.length);
            ElMessage.success(`传感器节点绘制完成，共 ${all_sensor.value.length} 个节点`);
        } catch (error) {
            console.error('传感器加载失败:', error);
            ElMessage.error(String(error));
        } finally {
            loading.close();
        }
    };

    // 服务站上传
    const stationupload = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Loading Station...' })
        try {
            const response = await axios.post('loadStation');
            const stations = response.data.data;

            console.log('服务站原始数据:', stations); // 调试用

            all_station.value = [];
            const features: any[] = [];

            for (let i = 0; i < stations.length; i++) {
                // 数据格式: [id, lng, lat]
                const id = stations[i][0];
                const lng = parseFloat(stations[i][1]);  // 经度
                const lat = parseFloat(stations[i][2]);  // 纬度
                const pos = [lng, lat];

                all_station.value.push([pos]);
                features.push(createIconFeature(cStaionSrc, [128, 128], pos));
            }

            stationSource.clear();
            stationSource.addFeatures(features);

            console.log('服务站数量:', all_station.value.length);
            ElMessage.success(`服务站绘制完成，共 ${all_station.value.length} 个节点`);
        } catch (error) {
            console.error('服务站加载失败:', error);
            ElMessage.error(String(error));
        } finally {
            loading.close();
        }
    };

    const createPathSegmentFeatures = (pathCoords: any[]) => {
        if (pathCoords.length < 2) return [];
        const transformedCoordinates = pathCoords.map(coord => fromLonLat(coord));
        const pathGeometry = new LineString(transformedCoordinates);
        const pathStyle = new Style({
            stroke: new Stroke({
                color: 'rgba(255, 69, 0, 0.7)',
                width: 4,
                lineDash: [10, 5],
            }),
        });
        const pathFeature = new Feature({ geometry: pathGeometry });
        pathFeature.setStyle(pathStyle);
        return [pathFeature];
    };

    function measureTextSize(text: string, font: string, padding: number[]): { width: number, height: number } {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d')!;
        ctx.font = font;
        const metrics = ctx.measureText(text);
        const textWidth = metrics.width;
        // 获取字号（假设字体字符串包含 "bold 38px ..." 这种格式）
        const fontSizeMatch = font.match(/(\d+)px/);
        const fontSize = fontSizeMatch ? parseInt(fontSizeMatch[1], 10) : 38;
        // 加上内边距 (padding: [top, right, bottom, left])
        const width = textWidth + padding[1] + padding[3];
        const height = fontSize + padding[0] + padding[2];
        return { width, height };
    }

    const textOptions = {
        font: 'bold 38px "SF Mono", "Consolas", "Monaco", "Courier New", monospace',
        padding: [6, 10, 6, 10],
        offsetY: -40,  // 基础偏移
        offsetX: 0
    };

    function adjustTextOffsetsToAvoidOverlap(
        source: VectorSource,
        map: any,
        maxSteps = 100   // 每个文本最多尝试的偏移数量
    ) {
        const features = source.getFeatures().filter(f => f.get('isStepMarker') === true);
        if (features.length < 2) return;

        // 预先计算每个文本的基础信息（屏幕坐标、宽高等）
        let items = features.map(f => {
            const geom = f.getGeometry();
            const coord = geom.getCoordinates();
            const screenCoord = map.getPixelFromCoordinate(coord);
            const step = f.getStyle().getText().getText();
            const fontString = 'bold 38px "SF Mono", "Consolas", "Monaco", "Courier New", monospace';
            const size = measureTextSize(step, fontString, [6, 10, 6, 10]);
            return {
                feature: f,
                originalScreen: screenCoord,
                offsetX: 0,
                offsetY: -25,    // 基础偏移
                width: size.width,
                height: size.height,
                // 存储当前选中的偏移（初始为基础偏移）
                currentOffsetX: 0,
                currentOffsetY: -25,
            };
        });

        // 检测两个矩形是否重叠
        function overlaps(itemA: any, itemB: any): boolean {
            const aLeft = itemA.originalScreen[0] + itemA.currentOffsetX - itemA.width / 2;
            const aRight = aLeft + itemA.width;
            const aTop = itemA.originalScreen[1] + itemA.currentOffsetY - itemA.height / 2;
            const aBottom = aTop + itemA.height;

            const bLeft = itemB.originalScreen[0] + itemB.currentOffsetX - itemB.width / 2;
            const bRight = bLeft + itemB.width;
            const bTop = itemB.originalScreen[1] + itemB.currentOffsetY - itemB.height / 2;
            const bBottom = bTop + itemB.height;

            return !(aRight < bLeft || aLeft > bRight || aBottom < bTop || aTop > bBottom);
        }

        // 检查一个文本是否与任何其他文本重叠
        function hasAnyOverlap(item: any, allItems: any[]): boolean {
            for (let other of allItems) {
                if (item === other) continue;
                if (overlaps(item, other)) return true;
            }
            return false;
        }

        // 生成螺旋偏移量（相对于原始偏移的偏移量，单位像素）
        function* spiralOffsets(startRadius = 5, step = 5): Generator<[number, number]> {
            // 初始偏移 (0,0) 表示原始位置
            yield [0, 0];
            let radius = startRadius;
            while (true) {
                // 上
                yield [0, -radius];
                // 右上
                yield [radius, -radius];
                // 右
                yield [radius, 0];
                // 右下
                yield [radius, radius];
                // 下
                yield [0, radius];
                // 左下
                yield [-radius, radius];
                // 左
                yield [-radius, 0];
                // 左上
                yield [-radius, -radius];
                radius += step;
            }
        }

        // 为每个文本独立寻找最佳偏移
        // 按顺序处理，后处理的文本避开前面已确定的文本
        for (let i = 0; i < items.length; i++) {
            const current = items[i];
            const offsetGen = spiralOffsets(5, 5);
            let found = false;

            for (let attempt = 0; attempt < maxSteps; attempt++) {
                const [dx, dy] = offsetGen.next().value;
                current.currentOffsetX = current.offsetX + dx;
                current.currentOffsetY = current.offsetY + dy;

                // 只检查与之前已处理的文本是否重叠（保证前面的不变）
                let overlapWithPrevious = false;
                for (let j = 0; j < i; j++) {
                    if (overlaps(current, items[j])) {
                        overlapWithPrevious = true;
                        break;
                    }
                }
                if (!overlapWithPrevious) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                console.warn(`Step text ${i} could not find non-overlapping offset after ${maxSteps} attempts`);
            }
        }

        // 最后再整体检查一次（避免后处理的文本与后面未处理的文本重叠？实际上后续会处理后面的，所以最终无重叠）
        // 应用样式
        items.forEach(item => {
            const oldStyle = item.feature.getStyle();
            const oldText = oldStyle.getText();
            if (oldText) {
                const newStyle = new Style({
                    text: new Text({
                        font: oldText.getFont(),
                        text: oldText.getText(),
                        fill: oldText.getFill(),
                        backgroundFill: oldText.getBackgroundFill(),
                        padding: oldText.getPadding(),
                        offsetX: item.currentOffsetX,
                        offsetY: item.currentOffsetY,
                        stroke: oldText.getStroke(),
                        textAlign: oldText.getTextAlign(),
                        textBaseline: oldText.getTextBaseline(),
                    })
                });
                item.feature.setStyle(newStyle);
            }
        });
    }

    const createStepTextFeature = (coordinate: any, step: any, moveTime: number, chargeTime: number, isDestination: any, nodeInfo: { type: string; }) => {
        const coords = fromLonLat(coordinate);
        let textContent = `Step: ${step}\nMove: ${moveTime.toFixed(2)}s`;
        if (chargeTime > 0.001) {
            textContent += `\nCharge: ${chargeTime.toFixed(2)}s`;
        }

        const textFeature = new Feature({
            geometry: new Point(coords),
        });

        textFeature.set('detail', textContent);
        textFeature.set('isStepMarker', true);

        textFeature.setStyle(new Style({
            text: new Text({
                // font: 'bold 24px sans-serif',
                font: 'bold 64px "SF Mono", "Consolas", "Monaco", "Courier New", monospace',
                text: step.toString(),
                fill: new Fill({ color: '#fff' }),
                // stroke: new Stroke({ color: '#000', width: 2 }),
                // stroke: new Stroke({ color: '#333333', width: 2 }),  // 细一点的描边
                backgroundFill: new Fill({ color: 'rgba(0, 0, 0, 0.6)' }),
                padding: [6, 10, 6, 10],
                offsetY: -25
            }),
        }));

        if (nodeInfo.type != 'station') {
            if (isDestination && nodeInfo) {
                const iconSrc = (nodeInfo.type === 'sensor') ? sensorGreen : relayGreen;
                const size = (nodeInfo.type === 'sensor') ? [32, 32] : [64, 64];
                const markerFeature = createIconFeature(iconSrc, size, coordinate);
                scheduleMarkerSource.addFeature(markerFeature);
            }
        }

        return textFeature;
    };

    const showMetricsDialog = (metrics: { [x: string]: any; }) => {
        let contentHtml = '<ul>';
        for (const key in metrics) {
            if (Object.prototype.hasOwnProperty.call(metrics, key)) {
                const formattedKey = key.replace(/([A-Z])/g, ' $1').replace(/^./, str => str.toUpperCase());
                const value = typeof metrics[key] === 'number' ? metrics[key].toFixed(4) : metrics[key];
                contentHtml += `<li><strong>${formattedKey}:</strong> ${value}</li>`;
            }
        }
        contentHtml += '</ul>';
        ElMessageBox.alert(contentHtml, 'Metrics', { dangerouslyUseHTMLString: true, confirmButtonText: 'OK' });
    };

    const TWSN_models = async () => {
        const loading = ElLoading.service({
            lock: true,
            text: '正在进行WSN仿真计算...',
            background: 'rgba(255, 255, 255, 0.5)',
        });

        try {
            const response = await axios.post('/runWSN');
            const result = response.data;

            if (result.result === 'SUCCESS') {
                const simulationData = result.data;
                const pointInfoList = simulationData.point_info;

                // 不清除任何原有数据，只更新图标颜色
                // 遍历所有仿真结果节点，更新对应图标的颜色
                pointInfoList.forEach((point) => {
                    const longitude = point.lng;
                    const latitude = point.lat;
                    const position = [longitude, latitude];

                    // 在 sensorSource 中查找对应位置的 feature 并更新图标
                    const sensorFeatures = sensorSource.getFeatures();
                    let found = false;

                    for (const feature of sensorFeatures) {
                        const geom = feature.getGeometry();
                        if (geom instanceof Point) {
                            const coords = toLonLat(geom.getCoordinates());
                            if (Math.abs(coords[0] - longitude) < 0.000001 &&
                                Math.abs(coords[1] - latitude) < 0.000001) {
                                found = true;
                                const icon = point.needs_charge ? sensorRed : sensorBlue;
                                feature.setStyle(new Style({
                                    image: new Icon({
                                        anchor: [0.5, 0.5],
                                        src: icon,
                                        size: [32, 32],
                                        scale: [1, 1],
                                    }),
                                }));
                                break;
                            }
                        }
                    }

                    // 如果在传感器中没找到，尝试在中继中查找
                    if (!found) {
                        const relayFeatures = relaySource.getFeatures();
                        for (const feature of relayFeatures) {
                            const geom = feature.getGeometry();
                            if (geom instanceof Point) {
                                const coords = toLonLat(geom.getCoordinates());
                                if (Math.abs(coords[0] - longitude) < 0.000001 &&
                                    Math.abs(coords[1] - latitude) < 0.000001) {
                                    const icon = point.needs_charge ? relayRed : relayBlue;
                                    feature.setStyle(new Style({
                                        image: new Icon({
                                            anchor: [0.5, 0.5],
                                            src: icon,
                                            size: [64, 64],
                                            scale: [1, 1],
                                        }),
                                    }));
                                    break;
                                }
                            }
                        }
                    }
                });

                loading.close();

                const needChargeCount = pointInfoList.filter(p => p.needs_charge).length;
                const totalChargingNodes = pointInfoList.filter(p => p.type !== 'station').length;

                ElMessage({
                    message: `WSN仿真完成！共处理 ${pointInfoList.length} 个节点，` +
                        `其中 ${needChargeCount}/${totalChargingNodes} 个节点需要充电`,
                    type: 'success',
                    duration: 5000
                });

                sessionStorage.setItem('lastWSNData', JSON.stringify(simulationData));

            } else {
                loading.close();
                ElMessage.error('WSN仿真失败: ' + (result.message || '未知错误'));
            }
        } catch (error) {
            loading.close();
            console.error('WSN仿真请求失败:', error);
            ElMessage.error(error.response?.data?.message || error.message || '网络请求失败');
        }
    };

    const single_greed = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Running Greedy Algorithm...' })
        try {
            const response = await axios.post('/runGreedyAlgorithm');
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            sensorSource.clear();
            relaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false);
            await relayupload(false);
            await stationupload();

            const sensorFeatures = [];
            const relayFeatures = [];

            pointInfoList.forEach((point) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    sensorFeatures.push(createIconFeature(sensorRed, [32, 32], position));
                } else if (point.type === 'relay') {
                    relayFeatures.push(createIconFeature(relayRed, [64, 64], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            relaySource.addFeatures(relayFeatures);

            const allPathFeatures = [];
            const allMarkerFeatures = [];
            let stepCounter = 1;

            if (actionLog && actionLog.length > 0) {
                actionLog.forEach((logEntry) => {
                    const pathNodes = logEntry.path_nodes || [];
                    const moveTime = logEntry.move_time || 0;
                    const chargeTime = logEntry.charge_time || 0;
                    const uavTo = logEntry.uav_to;
                    const destinationType = logEntry.destination_type;

                    const pathCoordinates = pathNodes.map(coord => {
                        if (Array.isArray(coord) && coord.length >= 2) {
                            return [coord[0], coord[1]];
                        }
                        return null;
                    }).filter(c => c !== null);

                    if (pathCoordinates.length >= 2) {
                        const pathFeatures = createPathSegmentFeatures(pathCoordinates);
                        allPathFeatures.push(...pathFeatures);

                        const endCoords = pathCoordinates[pathCoordinates.length - 1];
                        const endMarker = createStepTextFeature(
                            endCoords,
                            stepCounter,
                            moveTime,
                            chargeTime,
                            true,
                            { nodeId: uavTo, type: destinationType }
                        );
                        allMarkerFeatures.push(endMarker);
                        stepCounter++;
                    }
                });
            }

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            // 在添加完所有标记后，调整文本偏移避免重叠
            adjustTextOffsetsToAvoidOverlap(scheduleMarkerSource, map);

            loading.close();

            if (metrics) {
                showMetricsDialog(metrics);
            }

            ElMessage.success(`Greedy Algorithm completed!`);
        } catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('Greedy Algorithm 运行出错: ' + (error.response?.data?.message || error.message));
        }
    };

    const single_RAWCS = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Running RAWCS Algorithm...' })
        try {
            const response = await axios.post('/runRAWCSAlgorithm');
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            sensorSource.clear();
            relaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false);
            await relayupload(false);
            await stationupload();

            const sensorFeatures = [];
            const relayFeatures = [];

            pointInfoList.forEach((point) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    sensorFeatures.push(createIconFeature(sensorRed, [32, 32], position));
                } else if (point.type === 'relay') {
                    relayFeatures.push(createIconFeature(relayRed, [64, 64], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            relaySource.addFeatures(relayFeatures);

            const allPathFeatures = [];
            const allMarkerFeatures = [];
            let stepCounter = 1;

            if (actionLog && actionLog.length > 0) {
                actionLog.forEach((logEntry) => {
                    const pathNodes = logEntry.path_nodes || [];
                    const moveTime = logEntry.move_time || 0;
                    const chargeTime = logEntry.charge_time || 0;
                    const uavTo = logEntry.uav_to;
                    const destinationType = logEntry.destination_type;

                    const pathCoordinates = pathNodes.map(coord => {
                        if (Array.isArray(coord) && coord.length >= 2) {
                            return [coord[0], coord[1]];
                        }
                        return null;
                    }).filter(c => c !== null);

                    if (pathCoordinates.length >= 2) {
                        const pathFeatures = createPathSegmentFeatures(pathCoordinates);
                        allPathFeatures.push(...pathFeatures);

                        const endCoords = pathCoordinates[pathCoordinates.length - 1];
                        const endMarker = createStepTextFeature(
                            endCoords,
                            stepCounter,
                            moveTime,
                            chargeTime,
                            true,
                            { nodeId: uavTo, type: destinationType }
                        );
                        allMarkerFeatures.push(endMarker);
                        stepCounter++;
                    }
                });
            }

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            // 在添加完所有标记后，调整文本偏移避免重叠
            adjustTextOffsetsToAvoidOverlap(scheduleMarkerSource, map);

            loading.close();

            if (metrics) {
                showMetricsDialog(metrics);
            }

            ElMessage.success('RAWCS Algorithm completed!');
        } catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('RAWCS Algorithm 运行出错: ' + (error.response?.data?.message || error.message));
        }
    };

    // const single_ACO = async () => {
    //     const loading = ElLoading.service({ lock: true, text: 'Running ACO Algorithm ...' })
    //     try {
    //         const response = await axios.post('/runACOAlgorithm');
    //         const resultData = response.data.data;
    //         const pointInfoList = resultData.point_info;
    //         const actionLog = resultData.action_log;
    //         const metrics = resultData.metrics;
    //
    //         sensorSource.clear();
    //         relaySource.clear();
    //         schedulePathSource.clear();
    //         scheduleMarkerSource.clear();
    //
    //         await sensorupload(false);
    //         await relayupload(false);
    //         await stationupload();
    //
    //         const sensorFeatures = [];
    //         const relayFeatures = [];
    //
    //         pointInfoList.forEach((point) => {
    //             const position = [point.lng, point.lat];
    //             if (point.type === 'sensor') {
    //                 sensorFeatures.push(createIconFeature(sensorRed, [32, 32], position));
    //             } else if (point.type === 'relay') {
    //                 relayFeatures.push(createIconFeature(relayRed, [64, 64], position));
    //             }
    //         });
    //         sensorSource.addFeatures(sensorFeatures);
    //         relaySource.addFeatures(relayFeatures);
    //
    //         const allPathFeatures = [];
    //         const allMarkerFeatures = [];
    //         let stepCounter = 1;
    //
    //         if (actionLog && actionLog.length > 0) {
    //             actionLog.forEach((logEntry) => {
    //                 const pathNodes = logEntry.path_nodes || [];
    //                 const moveTime = logEntry.move_time || 0;
    //                 const chargeTime = logEntry.charge_time || 0;
    //                 const uavTo = logEntry.uav_to;
    //                 const destinationType = logEntry.destination_type;
    //
    //                 const pathCoordinates = pathNodes.map(coord => {
    //                     if (Array.isArray(coord) && coord.length >= 2) {
    //                         return [coord[0], coord[1]];
    //                     }
    //                     return null;
    //                 }).filter(c => c !== null);
    //
    //                 if (pathCoordinates.length >= 2) {
    //                     const pathFeatures = createPathSegmentFeatures(pathCoordinates);
    //                     allPathFeatures.push(...pathFeatures);
    //
    //                     const endCoords = pathCoordinates[pathCoordinates.length - 1];
    //                     const endMarker = createStepTextFeature(
    //                         endCoords,
    //                         stepCounter,
    //                         moveTime,
    //                         chargeTime,
    //                         true,
    //                         { nodeId: uavTo, type: destinationType }
    //                     );
    //                     allMarkerFeatures.push(endMarker);
    //                     stepCounter++;
    //                 }
    //             });
    //         }
    //
    //         schedulePathSource.addFeatures(allPathFeatures);
    //         scheduleMarkerSource.addFeatures(allMarkerFeatures);
    //
    //         // 在添加完所有标记后，调整文本偏移避免重叠
    //         adjustTextOffsetsToAvoidOverlap(scheduleMarkerSource, map);
    //
    //         loading.close();
    //
    //         if (metrics) {
    //             showMetricsDialog(metrics);
    //         }
    //
    //         ElMessage.success('ACO Algorithm completed!');
    //     } catch (error) {
    //         loading.close();
    //         console.error(error);
    //         ElMessage.error('ACO算法运行出错: ' + (error.response?.data?.message || error.message));
    //     }
    // };

    const single_ADRL = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Running ADRL Algorithm...' })
        try {
            // 这里改成 adrl 后端接口，注意和你flask路由保持一致：/api/runADRLAlgorithm
            const response = await axios.post('/runADRLAlgorithm');
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            sensorSource.clear();
            relaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false);
            await relayupload(false);
            await stationupload();

            const sensorFeatures = [];
            const relayFeatures = [];

            pointInfoList.forEach((point) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    sensorFeatures.push(createIconFeature(sensorRed, [32, 32], position));
                } else if (point.type === 'relay') {
                    relayFeatures.push(createIconFeature(relayRed, [64, 64], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            relaySource.addFeatures(relayFeatures);

            const allPathFeatures = [];
            const allMarkerFeatures = [];
            let stepCounter = 1;

            if (actionLog && actionLog.length > 0) {
                actionLog.forEach((logEntry) => {
                    const pathNodes = logEntry.path_nodes || [];
                    const moveTime = logEntry.move_time || 0;
                    const chargeTime = logEntry.charge_time || 0;
                    const uavTo = logEntry.uav_to;
                    const destinationType = logEntry.destination_type;

                    const pathCoordinates = pathNodes.map(coord => {
                        if (Array.isArray(coord) && coord.length >= 2) {
                            return [coord[0], coord[1]];
                        }
                        return null;
                    }).filter(c => c !== null);

                    if (pathCoordinates.length >= 2) {
                        const pathFeatures = createPathSegmentFeatures(pathCoordinates);
                        allPathFeatures.push(...pathFeatures);

                        const endCoords = pathCoordinates[pathCoordinates.length - 1];
                        const endMarker = createStepTextFeature(
                            endCoords,
                            stepCounter,
                            moveTime,
                            chargeTime,
                            true,
                            { nodeId: uavTo, type: destinationType }
                        );
                        allMarkerFeatures.push(endMarker);
                        stepCounter++;
                    }
                });
            }

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            // 文本防重叠
            adjustTextOffsetsToAvoidOverlap(scheduleMarkerSource, map);

            loading.close();
            if (metrics) {
                showMetricsDialog(metrics);
            }
            ElMessage.success('ADRL Algorithm completed!');
        } catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('ADRL Algorithm 运行出错: ' + (error.response?.data?.message || error.message));
        }
    };



    const single_NSGA = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Running NSGA Algorithm...' })
        try {
            const response = await axios.post('/runNSGAAlgorithm');
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            sensorSource.clear();
            relaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false);
            await relayupload(false);
            await stationupload();

            const sensorFeatures = [];
            const relayFeatures = [];

            pointInfoList.forEach((point) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    sensorFeatures.push(createIconFeature(sensorRed, [32, 32], position));
                } else if (point.type === 'relay') {
                    relayFeatures.push(createIconFeature(relayRed, [64, 64], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            relaySource.addFeatures(relayFeatures);

            const allPathFeatures = [];
            const allMarkerFeatures = [];
            let stepCounter = 1;

            if (actionLog && actionLog.length > 0) {
                actionLog.forEach((logEntry) => {
                    const pathNodes = logEntry.path_nodes || [];
                    const moveTime = logEntry.move_time || 0;
                    const chargeTime = logEntry.charge_time || 0;
                    const uavTo = logEntry.uav_to;
                    const destinationType = logEntry.destination_type;

                    const pathCoordinates = pathNodes.map(coord => {
                        if (Array.isArray(coord) && coord.length >= 2) {
                            return [coord[0], coord[1]];
                        }
                        return null;
                    }).filter(c => c !== null);

                    if (pathCoordinates.length >= 2) {
                        const pathFeatures = createPathSegmentFeatures(pathCoordinates);
                        allPathFeatures.push(...pathFeatures);

                        const endCoords = pathCoordinates[pathCoordinates.length - 1];
                        const endMarker = createStepTextFeature(
                            endCoords,
                            stepCounter,
                            moveTime,
                            chargeTime,
                            true,
                            { nodeId: uavTo, type: destinationType }
                        );
                        allMarkerFeatures.push(endMarker);
                        stepCounter++;
                    }
                });
            }

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            // 在添加完所有标记后，调整文本偏移避免重叠
            adjustTextOffsetsToAvoidOverlap(scheduleMarkerSource, map);

            loading.close();

            if (metrics) {
                showMetricsDialog(metrics);
            }

            ElMessage.success('NSGA Algorithm completed!');
        } catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('NSGA算法运行出错: ' + (error.response?.data?.message || error.message));
        }
    };

    // 下载函数
    function sensordownload() {
        window.location.href = '/api/downloadSensors';
    }
    function relaydownload() {
        window.location.href = '/api/downloadrelays';
    }
    function stationdownload() {
        window.location.href = '/api/downloadStations';
    }
    function downloadIntersections() {
        window.location.href = '/api/downloadIntersections';
    }
    function downloadRoads() {
        window.location.href = '/api/downloadRoads';
    }
    function chargingSchedulingdownload() {
        window.location.href = '/api/downloadChargingScheduling';
    }
    async function downloadFile(url, filename) {
        try {
            // 直接使用 window.location 跳转下载
            window.location.href = `${window.location.origin}/api/${url}`;
            ElMessage.success(`${filename} 下载已开始`);
        } catch (error) {
            ElMessage.error(`下载失败: ${error.message}`);
        }
    }


    onMounted(() => {
        createMap();
        nextTick(() => {});
    });

</script>

<style scoped>
    #map {
        width: 100%;
        height: 85vh;
    }
    .map-controls {
        position: absolute;
        top: 20px;
        right: 20px;
        z-index: 1000;
        display: flex;
        flex-direction: column;
        gap: 10px;
    }
    :deep(#map-dim-overlay) {
        position: absolute;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        background-color: rgba(255, 255, 255, 0.3);
        pointer-events: none;
        z-index: 1;
    }
    .yellow-color { background-color: #FFB800; color: white; }
    .green-color { background-color: #009688; color: white; }
    .green-color:hover { background-color: #33ABA0; }
</style>