<template>
    <div class="common-layout">
        <el-container>
            <el-main>
                <div id="map"></div>
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
                            <el-button type="warning" @click="handleMul" class="yellow-color">Mul VUE
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
                        <el-form-item label="GateWay">
                            {{ gateWayValue }}
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
        <el-dialog title="Single VUE" v-model="caculate_dialogVisible" width="50%">
            <div style="display: flex; justify-content: center;">
                <el-tooltip class="box-item" effect="dark" content="TWSN models" placement="top">
                    <el-button @click="TWSN_models">TWSN models</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="Greed" placement="top">
                    <el-button @click="single_greed">Greed</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="3D-TWCS" placement="top">
                    <el-button @click="single_PPO">PPO</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="ACO Algorithm" placement="top">
                    <el-button @click="single_ACO">ACO</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="NPSO Algorithm" placement="top">
                    <el-button @click="single_NPSO">NPSO</el-button>
                </el-tooltip>
<!--                <el-tooltip class="box-item" effect="dark" content="Mul VUE Charging scheduling" placement="top">-->
<!--                    <el-button @click="mul_charging_scheduling">Mul VUE Charging scheduling</el-button>-->
<!--                </el-tooltip>-->
            </div>
        </el-dialog>
        <el-dialog title="Mul VUE" v-model="mul_dialogVisible" width="50%">
            <div style="display: flex; justify-content: center;">
                <el-tooltip class="box-item" effect="dark" content="TWSN models" placement="top">
                    <el-button @click="TWSN_models2">TWSN models</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="Greed" placement="top">
                    <el-button @click="mul_greed">Mul Greed</el-button>
                </el-tooltip>
                <el-tooltip class="box-item" effect="dark" content="3D-TWCS" placement="top">
                    <el-button @click="mul_PPO">Mul PPO</el-button>
                </el-tooltip>
                <!--                <el-tooltip class="box-item" effect="dark" content="Mul VUE Charging scheduling" placement="top">-->
                <!--                    <el-button @click="mul_charging_scheduling">Mul VUE Charging scheduling</el-button>-->
                <!--                </el-tooltip>-->
            </div>
        </el-dialog>
        <el-dialog title="Upload" v-model="upload_dialogVisible" width="50%">
            <div style="display: flex; justify-content: center;">
                <el-button @click="sensorupload">Sensor Upload</el-button>
                <el-button @click="gateWayupload">gateWay Upload</el-button>
                <el-button @click="stationupload">Station Upload</el-button>
            </div>
        </el-dialog>
        <el-dialog title="Download" v-model="download_dialogVisible" width="50%">
            <div style="display: flex; justify-content: center;">
                <el-button @click="sensordownload">Sensor Download</el-button>
                <el-button @click="gateWaydownload">gateWay Download</el-button>
                <el-button @click="stationdownload">Station Download</el-button>
                <el-button @click="chargingSchedulingdownload">Charging Scheduling Download</el-button>
            </div>
        </el-dialog>
        <el-dialog title="Which test data would you like to upload?" v-model="TestData_dialogVisible" width="20%">
            <div style="display: flex; justify-content: center;">
                <el-button type="primary" @click="handleLoad('data1')">data1</el-button>
                <el-button type="primary" @click="handleLoad('data2')">data2</el-button>
                <el-button type="primary" @click="handleLoad('data3')">data3</el-button>
                <el-button type="primary" @click="handleLoad('data4')">data4</el-button>
            </div>
        </el-dialog>
        <el-dialog title="Which dataset would you like to upload?" v-model="Dataset_dialogVisible" width="45%">
            <el-table :data="setNames" height="250" style="width: 100%">
                <el-table-column type="index" width="50"/>
                <el-table-column label="Name" width="150" prop="name"/>
                <el-table-column label="Sensor Size" width="100" prop="sensorSize"/>
                <el-table-column label="Station Size" width="130" prop="stationSize"/>
                <el-table-column label="gateWay Size" width="130" prop="gateWaySize"/>
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
    import {ref, onMounted, computed, shallowRef} from 'vue';
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
    import Polygon from 'ol/geom/Polygon';
    import {Fill, Stroke} from 'ol/style';
    import {Circle, LineString} from 'ol/geom';
    import {getDistance} from 'ol/sphere';
    import MouseWheelZoom from 'ol/interaction/MouseWheelZoom';

    // 图片资源
    const gIconSrc = "./assets/icon/jian.png";
    const sIconSrc = "./assets/icon/point16g.png";
    const cIconSrc = "./assets/icon/green.png";
    const cStaionSrc = "./assets/icon/car.png";
    const redSensor = "./assets/icon/point16red.png";
    const redGateWay = "./assets/icon/jianred.png";
    const blueSensor = "./assets/icon/point16blue.png";
    const blueGateWay = "./assets/icon/jianblue.png";

    // 地图相关变量 - 性能优化的关键是复用 Layer 和 Source
    let map: any;

    // 定义各个类型的 Source，避免重复创建
    const sensorSource = new VectorSource();
    const gatewaySource = new VectorSource();
    const stationSource = new VectorSource();
    const crossSource = new VectorSource();
    const tempMarkSource = new VectorSource(); // 用于临时的标记（如正在画的线）
    const pathSource = new VectorSource(); // 用于路径规划
    const schedulePathSource = new VectorSource(); // 调度路径
    const scheduleMarkerSource = new VectorSource(); // 调度标记

    // 定义 Layer
    let sensorLayer: VectorLayer<VectorSource>;
    let gatewayLayer: VectorLayer<VectorSource>;
    let stationLayer: VectorLayer<VectorSource>;
    let crossLayer: VectorLayer<VectorSource>;
    let tempMarkLayer: VectorLayer<VectorSource>;
    let pathLayer: VectorLayer<VectorSource>;
    let schedulePathLayer: VectorLayer<VectorSource>;
    let scheduleMarkerLayer: VectorLayer<VectorSource>;

    const createMap = () => {
        // 初始化图层
        sensorLayer = new VectorLayer({ source: sensorSource, zIndex: 5 });
        gatewayLayer = new VectorLayer({ source: gatewaySource, zIndex: 6 });
        stationLayer = new VectorLayer({ source: stationSource, zIndex: 7 });
        crossLayer = new VectorLayer({ source: crossSource, zIndex: 4 });
        tempMarkLayer = new VectorLayer({ source: tempMarkSource, zIndex: 10 });
        pathLayer = new VectorLayer({ source: pathSource, zIndex: 3 });
        schedulePathLayer = new VectorLayer({ source: schedulePathSource, zIndex: 8 });
        scheduleMarkerLayer = new VectorLayer({ source: scheduleMarkerSource, zIndex: 9 });

        map = new Map({
            target: 'map',
            interactions: defaultInteractions({
                doubleClickZoom: false,
                mouseWheelZoom: false,
            }),
            layers: [
                new TileLayer({ source: new OSM() }),
                // 一次性添加所有矢量图层
                pathLayer,
                crossLayer,
                sensorLayer,
                gatewayLayer,
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
            controls: defaultControls().extend([
                new ScaleLine(),
            ]),
        });

        map.on('moveend', () => {
            currentZoomLevel.value = map.getView().getZoom();
        });
        currentZoomLevel.value = map.getView().getZoom();

        // 添加点击交互
        map.on('click', (evt) => {
            // 在点击位置搜索 Feature
            const feature = map.forEachFeatureAtPixel(evt.pixel, (f) => f);

            if (feature && feature.get('isStepMarker')) {
                const detail = feature.get('detail');
                // 使用 ElMessageBox 显示，体验更好
                ElMessageBox.alert(detail, 'Step Details', {
                    confirmButtonText: 'OK',
                    callback: () => {}
                });
            }
        });
    }

    // 辅助函数：创建一个带图标的 Feature，但不创建图层
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

    // 兼容旧的 addIcon 调用，现在它会将 feature 添加到 tempMarkSource (除非指定 source)
    // 注意：尽量不要在循环中调用这个，而是一次性生成 feature 数组添加
    const addSingleIconToSource = (source: VectorSource, iconSrc: string, size: number[], position: Coordinate) => {
        const feature = createIconFeature(iconSrc, size, position);
        source.addFeature(feature);
    }

    // 响应式变量
    // 使用 shallowRef 对大数据数组进行性能优化，避免深层递归监听
    const all_sensor = shallowRef<any[][]>([]);
    const all_station = shallowRef<any[][]>([]);
    const all_gateWay = shallowRef<any[][]>([]);
    const chosenNumber = ref(0);
    const currentZoomLevel = ref(19);

    let cross: any[] = [];
    let pointArr: any[] = [];
    let pts: any[] = [];
    let pointIndex = 0;

    const ifmark = ref(false);
    const caculate_dialogVisible = ref(false);
    const mul_dialogVisible = ref(false);
    const upload_dialogVisible = ref(false);
    const download_dialogVisible = ref(false);
    const TestData_dialogVisible = ref(false);
    const Dataset_dialogVisible = ref(false);
    const AboutDatasetdialogVisible = ref(false);

    const setNames = ref<Array<{ name: string; sensorSize: number; stationSize:number;gateWaySize:number }>>([]);
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

    const gateWayValue = computed(() => {
        let total = 0;
        all_gateWay.value.forEach(row => total += row.length);
        return total;
    });

    const chosenNumberStr = computed(() => chosenNumber.value === 0 ? "_ _" : chosenNumber.value);

    const isRestrictedDataset = (name: string) => ['data1', 'data2', 'data3','data4'].includes(name);

    type Option = { id: number, label: string, desc: string }
    const nodeType = ref<Option>({id: 1, label: 'station', desc: '服务站点'})
    const options = ref([
        {id: 1, label: 'station', desc: '服务站点'},
        {id: 2, label: 'sensor', desc: '路段传感器标记'},
        {id: 3, label: 'sensor_single', desc: '单点传感器标记'},
        {id: 5, label: 'gateway', desc: '路段网关标记'},
        {id: 6, label: 'gateway_single', desc: '单点网关标记'},
        {id: 7, label: 'cross', desc: '路口标记'},
        {id: 8, label: 'None', desc: '无标记'},
    ])

    // 清除所有 Source 的数据 (不再移除图层)
    function clearVectorLayers() {
        sensorSource.clear();
        gatewaySource.clear();
        stationSource.clear();
        crossSource.clear();
        tempMarkSource.clear();
        pathSource.clear();
        schedulePathSource.clear();
        scheduleMarkerSource.clear();
    }

    const handleLoad = (name: string) => {
        axios.post('datasetLoad', null, { params: { datasetName: name } })
            .then((response) => {
                all_sensor.value = [];
                all_gateWay.value = [];
                chosenNumber.value = 0;
                cross = [];
                pointArr = [];
                pts = [];

                clearVectorLayers();

                let location_lng = response.data.data[0]
                let location_lat = response.data.data[1]

                const view = map.getView();
                view.setCenter(fromLonLat([location_lng, location_lat]));
                view.setZoom(19);

                // 并行加载，提高速度
                Promise.all([
                    sensorupload(),
                    gateWayupload(),
                    stationupload()
                ]).then(() => {
                    Dataset_dialogVisible.value = false;
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

    // 撤销逻辑
    let guard_undo: string;
    const undo = () => {
        // 注意：shallowRef 需要通过重新赋值来触发响应性，pop 不会自动触发
        // 但这里我们主要依赖 drawAllMarked 来重绘
        if (guard_undo === "sensor") {
            const arr = [...all_sensor.value]; arr.pop(); all_sensor.value = arr;
        } else if (guard_undo === "gateWay") {
            const arr = [...all_gateWay.value]; arr.pop(); all_gateWay.value = arr;
        } else if (guard_undo === "cross") {
            cross.pop();
        } else if (guard_undo === "pointArr_first") {
            pointArr.pop();
        } else if (guard_undo === "station") {
            const arr = [...all_station.value]; arr.pop(); all_station.value = arr;
        }
        drawAllMarked();
    };

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
        map.getTargetElement().style.cursor = (the_nodetype === "None") ?
            "url(https://api.map.baidu.com/images/openhand.cur) 8 8, default" : "crosshair";
    }

    function handleClick(e: { coordinate: any; }) {
        var coordinate = e.coordinate;
        var lonLat = toLonLat(coordinate);
        var the_nodetype = nodeType.value?.label

        if (the_nodetype === "None") {
            return;
        } else if (the_nodetype === "cross") {
            cross.push(lonLat);
            guard_undo = 'cross';
            addSingleIconToSource(crossSource, cIconSrc, [32, 32], lonLat);
        } else if (the_nodetype === "gateway_single") {
            all_gateWay.value = [...all_gateWay.value, [lonLat]];
            guard_undo = 'gateWay';
            addSingleIconToSource(gatewaySource, gIconSrc, [32, 32], lonLat);
        } else if (the_nodetype === "station") {
            all_station.value = [...all_station.value, [lonLat]];
            guard_undo = 'station';
            addSingleIconToSource(stationSource, cStaionSrc, [32, 32], lonLat);
        } else if (the_nodetype === "sensor_single") {
            all_sensor.value = [...all_sensor.value, [lonLat]];
            guard_undo = 'sensor';
            addSingleIconToSource(sensorSource, sIconSrc, [16, 16], lonLat);
        } else {
            // 连线模式
            pointArr.push(lonLat);
            if (pointArr.length === 1) {
                addSingleIconToSource(tempMarkSource, cIconSrc, [32, 32], pointArr[0]);
                guard_undo = 'pointArr_first';
            }
            if (pointArr.length === 2) {
                // 清除临时的点
                tempMarkSource.clear();

                var start = pointArr[0];
                var end = pointArr[1];
                pointArr = []

                var x1 = start[0], y1 = start[1];
                var x2 = end[0], y2 = end[1];

                pointIndex = (the_nodetype == "sensor") ? 5 : 20;
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
                    guard_undo = 'sensor';
                } else {
                    all_gateWay.value = [...all_gateWay.value, tempPts];
                    guard_undo = 'gateWay';
                }

                // 绘制新添加的一组点
                const targetSource = (the_nodetype == "sensor") ? sensorSource : gatewaySource;
                const targetIcon = (the_nodetype == "sensor") ? sIconSrc : gIconSrc;
                const targetSize = (the_nodetype == "sensor") ? [16, 16] : [32, 32];

                const newFeatures = tempPts.map(pt => createIconFeature(targetIcon, targetSize, pt));
                targetSource.addFeatures(newFeatures);
            }
        }
    }

    // 重绘所有标注 (性能优化版)
    function drawAllMarked() {
        // 清空所有相关 Source
        crossSource.clear();
        gatewaySource.clear();
        sensorSource.clear();
        stationSource.clear();
        tempMarkSource.clear();

        const features = [];

        // Cross
        cross.forEach(pt => features.push(createIconFeature(cIconSrc, [32, 32], pt)));
        crossSource.addFeatures(features);

        // Gateway - 批量生成
        const gatewayFeatures: any[] = [];
        all_gateWay.value.forEach(group => {
            group.forEach(pt => gatewayFeatures.push(createIconFeature(gIconSrc, [32, 32], pt)));
        });
        gatewaySource.addFeatures(gatewayFeatures);

        // Sensor - 批量生成
        const sensorFeatures: any[] = [];
        all_sensor.value.forEach(group => {
            group.forEach(pt => sensorFeatures.push(createIconFeature(sIconSrc, [16, 16], pt)));
        });
        sensorSource.addFeatures(sensorFeatures);

        // Station - 批量生成
        const stationFeatures: any[] = [];
        all_station.value.forEach(group => {
            group.forEach(pt => stationFeatures.push(createIconFeature(cStaionSrc, [32, 32], pt)));
        });
        stationSource.addFeatures(stationFeatures);
    }

    const formatting = () => {
        ElMessageBox.confirm('All points marked on the map will be cleared，Continue?', 'Warning', { type: 'warning' })
            .then(() => {
                all_sensor.value = [];
                all_station.value = [];
                all_gateWay.value = [];
                chosenNumber.value = 0;
                cross = [];
                pointArr = [];
                pts = [];
                clearVectorLayers();
                ElMessage.success('Delete completed')
            })
            .catch(() => {})
    };


    const saveData = () => {
        // 构造数据
        const gateWay_array = all_gateWay.value;
        const sensor_array = all_sensor.value;
        const station_array = all_station.value;

        const quickSaveData = {
            "gate_way_points": gateWay_array,
            "sensor_array": sensor_array,
            "station_array": station_array,
        };

        const loading = ElLoading.service({ lock: true, text: 'Saving...' })
        axios.post('hhquicksaveNew', quickSaveData)
            .then(() => {
                loading.close();
                ElMessage.success('Success!');
                pointArr = [];
            })
            .catch((error: any) => {
                loading.close();
                ElMessage.error(String(error));
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
    const handleMul = () => { mul_dialogVisible.value = true; };
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
                    gateWaySize: d.gate_way_size
                }));
                Dataset_dialogVisible.value = true;
            })
            .catch((e: any) => ElMessage.error(e.response?.data?.message || 'Error'));
    };

    // 优化的 Upload 函数 - 批量处理
    const gateWayupload = async (isAppend = false) => {
        const loading = ElLoading.service({ lock: true, text: 'Loading Gateway...' })
        try {
            const response = await axios.post('cup');
            const gateWay = response.data.data;

            // 数据处理
            all_gateWay.value = [];
            const features: any[] = [];

            // 批量创建 Feature
            for (let i = 0; i < gateWay.length; i++) {
                const pos = [gateWay[i][2], gateWay[i][3]];
                all_gateWay.value.push([pos]);
                features.push(createIconFeature(gIconSrc, [32, 32], pos));
            }

            // 一次性清空并添加
            // gatewaySource.clear();
            if (!isAppend) {
                gatewaySource.clear(); // 只有不是追加模式才清空
            }
            gatewaySource.addFeatures(features);

            ElMessage.success('中继节点绘制完成');
        } catch (error) {
            console.error(error);
            ElMessage.error(String(error));
        } finally {
            loading.close();
        }
    };


    const sensorupload = async (isAppend = false) => {
        const loading = ElLoading.service({ lock: true, text: 'Loading Sensor...' });
        try {
            const response = await axios.post('sup');
            const sensors = response.data.data;

            all_sensor.value = [];
            const features: any[] = [];

            // 大量数据处理时，避免 Vue 响应式带来的额外开销，先用普通数组
            const tempSensorData = [];

            for (let i = 0; i < sensors.length; i++) {
                const pos = [sensors[i][2], sensors[i][3]];
                tempSensorData.push([pos]);
                features.push(createIconFeature(sIconSrc, [16, 16], pos));
            }

            // 更新响应式数据
            all_sensor.value = tempSensorData;

            // 更新地图 Source (这是性能提升的关键点)
            // sensorSource.clear();
            if (!isAppend) {
                sensorSource.clear(); // 只有不是追加模式才清空
            }
            sensorSource.addFeatures(features);

            ElMessage.success('传感器节点绘制完成');
        } catch (error) {
            ElMessage.error(String(error));
        } finally {
            loading.close();
        }
    };

    const stationupload = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Loading Station...' })
        try {
            const response = await axios.post('loadStation');
            const stations = response.data.data;

            all_station.value = [];
            const features: any[] = [];

            for (let i = 0; i < stations.length; i++) {
                const pos = [stations[i][2], stations[i][3]];
                all_station.value.push([pos]);
                features.push(createIconFeature(cStaionSrc, [32, 32], pos));
            }

            stationSource.clear();
            stationSource.addFeatures(features);

            ElMessage.success('服务站绘制完成');
        } catch (error) {
            ElMessage.error(String(error));
        } finally {
            loading.close();
        }
    };

    /**
     * 创建单个路径段的 Feature 列表（只包含线段，无箭头）。
     */
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

    const createStepTextFeature = (coordinate: any, step: any, moveTime: number, chargeTime: number, isDestination: any, nodeInfo: { type: string; }) => {
        const coords = fromLonLat(coordinate);
        let textContent = `Step: ${step}\nMove: ${moveTime.toFixed(2)}s`;
        if (chargeTime > 0.001) {
            textContent += `\nCharge: ${chargeTime.toFixed(2)}s`;
        }

        const textFeature = new Feature({
            geometry: new Point(coords),
        });

        // 将详细数据绑定到 Feature
        textFeature.set('detail', textContent);
        textFeature.set('isStepMarker', true); // 标记这是一个步骤编号，方便过滤点击事件

        // textFeature.setStyle(new Style({
        //     text: new Text({
        //         font: '14px sans-serif',
        //         text: textContent,
        //         fill: new Fill({ color: 'black' }),
        //         stroke: new Stroke({ color: 'white', width: 3 }),
        //         offsetX: isDestination ? 15 : -15,
        //         offsetY: isDestination ? 15 : -15,
        //         textAlign: isDestination ? 'left' : 'right',
        //         backgroundFill: new Fill({ color: 'rgba(255, 255, 255, 0.7)' }),
        //         padding: [2, 5, 2, 5]
        //     }),
        // }));
        textFeature.setStyle(new Style({
            text: new Text({
                font: 'bold 14px sans-serif',
                text: step.toString(), // 只显示编号
                fill: new Fill({ color: '#fff' }),
                stroke: new Stroke({ color: '#000', width: 2 }),
                backgroundFill: new Fill({ color: 'rgba(0, 0, 0, 0.6)' }),
                padding: [3, 5, 3, 5],
                offsetY: -20 // 稍微向上偏移，避免盖住节点图标
            }),
        }));

        // 如果是终点，直接更新图标到 scheduleMarkerSource
        if (nodeInfo.type != 'station') {
            if (isDestination && nodeInfo) {
                const iconSrc = (nodeInfo.type === 'sensor') ? blueSensor : blueGateWay;
                const size = (nodeInfo.type === 'sensor') ? [16, 16] : [32, 32];
                // 注意：这里我们不调用 addLayer，而是返回 feature 或者添加到 source
                // 为了逻辑统一，这里我们添加一个额外的 Feature 来盖住原来的
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

    /**
     * 运行单个ACO算法
     * 参考 single_greed 函数的结构
     */
    const single_ACO = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Running ACO Algorithm...' })
        try {
            // 使用默认参数运行ACO算法
            const response = await axios.post('runACOSimulation');
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            // 1. 重置传感器和网关显示 (基于 point_info 状态)
            sensorSource.clear();
            gatewaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false);
            await gateWayupload(false);
            await stationupload();

            const sensorFeatures: any[] = [];
            const gatewayFeatures: any[] = [];

            pointInfoList.forEach((point: { lng: any; lat: any; type: string; }) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    sensorFeatures.push(createIconFeature(redSensor, [16, 16], position));
                } else if (point.type === 'gateWay') {
                    gatewayFeatures.push(createIconFeature(redGateWay, [32, 32], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            gatewaySource.addFeatures(gatewayFeatures);

            // 2. 绘制路径和步骤
            const allPathFeatures: any[] = [];
            const allMarkerFeatures: any[] = [];
            let stepCounter = 1;

            actionLog.forEach((logEntry: { path_nodes: any[]; move_time: any; charge_time: any; uav_to: any; destination_type: any; }) => {
                const pathNodes = logEntry.path_nodes;
                const moveTime = logEntry.move_time;
                const chargeTime = logEntry.charge_time;
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
                        {nodeId: uavTo, type: destinationType}
                    );
                    allMarkerFeatures.push(endMarker);
                    stepCounter++;
                }
            });

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            loading.close();

            // 显示额外的ACO信息（如果存在）
            if (resultData.aco_path) {
                ElMessageBox.alert(
                    `ACO优化路径: ${resultData.aco_path.join(' → ')}`,
                    'ACO Path',
                    { confirmButtonText: 'OK' }
                );
            }

            showMetricsDialog(metrics);
            ElMessage.success('ACO调度完成!');
        } catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('ACO算法运行出错');
        }
    };

    /**
     * 运行单个NPSO算法
     * 参考 single_greed 函数的结构
     */
    const single_NPSO = async () => {
        const loading = ElLoading.service({ lock: true, text: 'Running NPSO Algorithm...' })
        try {
            // 使用默认参数运行NPSO算法
            const response = await axios.post('runNPSOSimulation');
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            // 1. 重置传感器和网关显示 (基于 point_info 状态)
            sensorSource.clear();
            gatewaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false);
            await gateWayupload(false);
            await stationupload();

            const sensorFeatures: any[] = [];
            const gatewayFeatures: any[] = [];

            pointInfoList.forEach((point: { lng: any; lat: any; type: string; }) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    sensorFeatures.push(createIconFeature(redSensor, [16, 16], position));
                } else if (point.type === 'gateWay') {
                    gatewayFeatures.push(createIconFeature(redGateWay, [32, 32], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            gatewaySource.addFeatures(gatewayFeatures);

            // 2. 绘制路径和步骤
            const allPathFeatures: any[] = [];
            const allMarkerFeatures: any[] = [];
            let stepCounter = 1;

            actionLog.forEach((logEntry: { path_nodes: any[]; move_time: any; charge_time: any; uav_to: any; destination_type: any; }) => {
                const pathNodes = logEntry.path_nodes;
                const moveTime = logEntry.move_time;
                const chargeTime = logEntry.charge_time;
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
                        {nodeId: uavTo, type: destinationType}
                    );
                    allMarkerFeatures.push(endMarker);
                    stepCounter++;
                }
            });

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            loading.close();

            // 显示额外的NPSO信息（如果存在）
            if (resultData.npso_path) {
                ElMessageBox.alert(
                    `NPSO优化路径: ${resultData.npso_path.join(' → ')}`,
                    'NPSO Path',
                    { confirmButtonText: 'OK' }
                );
            }

            if (resultData.node_times) {
                console.log('Node charging time ratios:', resultData.node_times);
            }

            showMetricsDialog(metrics);
            ElMessage.success('NPSO调度完成!');
        } catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('NPSO算法运行出错');
        }
    };

    const single_PPO = async () => {

        const loading = ElLoading.service({ lock: true, text: 'Running PPO Algorithm...' })
        try {
            // 1. 使用 await 获取响应，而不是使用 .then
            const response = await axios.post('runPPOSimulation');
            // await axios.post('runPPOSimulation')
            // .then((response) => {
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            // 1. 重置传感器和网关显示 (基于 point_info 状态)
            sensorSource.clear();
            gatewaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false); // 第一次加载，清空旧的，画上全绿
            await gateWayupload(false);
            await stationupload();

            const sensorFeatures: any[] = [];
            const gatewayFeatures: any[] = [];

            pointInfoList.forEach((point: { lng: any; lat: any; type: string; }) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    // 使用红色图标表示需要充电
                    sensorFeatures.push(createIconFeature(redSensor, [16, 16], position));
                } else if (point.type === 'gateWay') {
                    gatewayFeatures.push(createIconFeature(redGateWay, [32, 32], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            gatewaySource.addFeatures(gatewayFeatures);

            // 2. 绘制路径和步骤
            const allPathFeatures: any[] = [];
            const allMarkerFeatures: any[] = [];
            let stepCounter = 1;

            actionLog.forEach((logEntry: { path_nodes: any[]; move_time: any; charge_time: any; uav_to: any; destination_type: any; }) => {
                // const pathNodesStr = logEntry.path_nodes;
                const pathNodes = logEntry.path_nodes;
                const moveTime = logEntry.move_time;
                const chargeTime = logEntry.charge_time;
                const uavTo = logEntry.uav_to;
                const destinationType = logEntry.destination_type;

                // const pathCoordinates = pathNodesStr
                //     .map((coordStr: string) => coordStr.split(',').map(Number))
                //     .filter((coord: string | any[]) => coord.length === 2 && !isNaN(coord[0]));
                const pathCoordinates = pathNodes.map(coord => {
                    // coord 现在是 [经度, 纬度] 的数字数组
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
                        {nodeId: uavTo, type: destinationType}
                    );
                    allMarkerFeatures.push(endMarker);
                    stepCounter++;
                }
            });

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            loading.close();
            showMetricsDialog(metrics);
            ElMessage.success('Scheduling completed!');
        }catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('算法运行出错');
        }
        // })
        // .catch((error: any) => {
        //     loading.close();
        //     console.error(error);
        //     ElMessage.error(String(error));
        // });
    };

    // const TWSN_models = () => {
    //     const loading = ElLoading.service({
    //         lock: true,
    //         text: '正在进行TWSN模拟计算',
    //         background: 'rgba(255, 255, 255, 0.5)',
    //     })
    //     axios.post('runTWSNSimulation')
    //         .then((response: { data: { data: any; }; }) => {
    //             const simulationData = response.data.data; // 得到 Map: { graph_G: {...}, point_info: [...] }
    //             const pointInfoList = simulationData.point_info; // 得到节点列表
    //
    //             all_sensor.value = [];
    //             all_gateWay.value = [];
    //
    //             pointInfoList.forEach(point => {
    //                 let iconPath = '';
    //
    //                 const longitude = point.lng;
    //                 const latitude = point.lat;
    //                 // 构建位置数组，OpenLayers 需要先经度后纬度的顺序
    //                 const position = [longitude, latitude];
    //
    //                 // 存储处理过的充电点位置数据（可选）
    //                 let tmp = [];
    //                 tmp.push(position);
    //
    //                 if (point.type === 'sensor') {
    //                     // 传感器：低电量则标红
    //                     iconPath = redSensor;
    //                     all_sensor.value.push(tmp);
    //                     addSingleIconToSource(iconPath, [16, 16], position);
    //                 } else if (point.type === 'gate_way') {
    //                     // 中继节点：低电量则标红
    //                     iconPath = redGateWay
    //                     all_gateWay.value.push(tmp);
    //                     addSingleIconToSource(iconPath, [32, 32], position);
    //                 }
    //
    //             });
    //             loading.close();
    //             ElMessage({
    //                 message: '电量仿真结束！',
    //                 type: 'success',
    //             })
    //         })
    //         .catch((error: any) => {
    //             loading.close();
    //             console.error(error);
    //             ElMessage({
    //                 message: error,
    //                 type: 'error',
    //             })
    //         });
    // };
    const TWSN_models = () => {
        const loading = ElLoading.service({
            lock: true,
            text: '正在进行TWSN模拟计算',
            background: 'rgba(255, 255, 255, 0.5)',
        });

        // console.log('开始调用 runTWSNSimulation API...'); // 添加日志

        axios.post('runTWSNSimulation')
            .then((response: { data: { data: any; }; }) => {
                // console.log('API响应原始数据:', response); // 打印完整响应

                // // 深度检查响应结构
                // if (!response) {
                //     console.error('response 为空');
                //     loading.close();
                //     return;
                // }
                //
                // if (!response.data) {
                //     console.error('response.data 为空');
                //     loading.close();
                //     return;
                // }
                //
                // console.log('response.data:', response.data);

                // if (!response.data.data) {
                //     console.error('response.data.data 为空');
                //     loading.close();
                //     return;
                // }

                const simulationData = response.data.data;
                // console.log('simulationData:', simulationData);
                // console.log('simulationData 类型:', typeof simulationData);

                // 检查关键字段
                // console.log('simulationData.graph_G:', simulationData.graph_G);
                // console.log('simulationData.point_info:', simulationData.point_info);
                // console.log('simulationData.point_info 类型:', typeof simulationData.point_info);

                // if (!simulationData.point_info) {
                //     // console.error('simulationData.point_info 为空');
                //     loading.close();
                //     ElMessage.error('数据中缺少 point_info 字段');
                //     return;
                // }

                // if (!Array.isArray(simulationData.point_info)) {
                //     // console.error('point_info 不是数组，实际类型:', typeof simulationData.point_info);
                //     loading.close();
                //     ElMessage.error(`point_info 不是数组，而是 ${typeof simulationData.point_info}`);
                //     return;
                // }

                const pointInfoList = simulationData.point_info;
                console.log(`pointInfoList 长度: ${pointInfoList.length}`);

                // // 检查第一个元素的结构
                // if (pointInfoList.length > 0) {
                //     console.log('第一个点数据:', pointInfoList[0]);
                //     console.log('第一个点的字段:', Object.keys(pointInfoList[0]));
                //     console.log('第一个点的 type:', pointInfoList[0].type);
                //     console.log('第一个点的 lng:', pointInfoList[0].lng);
                //     console.log('第一个点的 lat:', pointInfoList[0].lat);
                // }

                all_sensor.value = [];
                all_gateWay.value = [];

                // 详细处理每个点
                pointInfoList.forEach((point, index) => {
                    console.log(`处理第 ${index + 1} 个点:`, point);

                    // 检查点数据
                    if (!point) {
                        console.error(`第 ${index + 1} 个点为 null 或 undefined`);
                        return;
                    }

                    const longitude = point.lng;
                    const latitude = point.lat;

                    console.log(`  坐标: lng=${longitude}, lat=${latitude}`);
                    console.log(`  类型: ${point.type}`);

                    // 检查坐标是否有效
                    if (longitude === undefined || latitude === undefined) {
                        console.error(`第 ${index + 1} 个点坐标缺失`, point);
                        return;
                    }

                    if (isNaN(longitude) || isNaN(latitude)) {
                        console.error(`第 ${index + 1} 个点坐标不是数字`, point);
                        return;
                    }

                    const position = [longitude, latitude];
                    // console.log(`  position 数组:`, position);

                    const tmp = [position];
                    let iconPath = '';

                    if (point.type === 'sensor') {
                        iconPath = redSensor;
                        all_sensor.value.push(tmp);
                        // console.log(`  添加到 sensor，使用图标: ${iconPath}`);
                        addSingleIconToSource(sensorSource, iconPath, [16, 16], position);
                    } else if (point.type === 'gateWay'  || point.type === 'gate_way') {
                        iconPath = redGateWay;
                        all_gateWay.value.push(tmp);
                        // console.log(`  添加到 gateWay，使用图标: ${iconPath}`);
                        addSingleIconToSource(gatewaySource, iconPath, [32, 32], position);
                    }
                    // else if (point.type === 'station') {
                    //     // console.log(`  忽略 station 类型节点`);
                    // } else {
                    //     // console.warn(`  未知类型: ${point.type}`, point);
                    // }
                });

                // console.log('处理完成，开始绘制...');
                loading.close();
                ElMessage({
                    message: `电量仿真结束！共处理 ${pointInfoList.length} 个节点`,
                    type: 'success',
                });
            })
            .catch((error: any) => {
                console.error('API请求失败详情:');
                console.error('错误对象:', error);
                console.error('错误消息:', error.message);
                console.error('错误响应:', error.response);
                if (error.response) {
                    console.error('状态码:', error.response.status);
                    console.error('响应数据:', error.response.data);
                    console.error('响应头:', error.response.headers);
                }

                loading.close();
                ElMessage({
                    message: error.response?.data?.message || error.message || '网络请求失败',
                    type: 'error',
                });
            });
    };

    const single_greed = async () => {

        const loading = ElLoading.service({ lock: true, text: 'Running Greedy Algorithm...' })
        try {
            // 1. 使用 await 获取响应，而不是使用 .then
            const response = await axios.post('runGreedyAlgorithm');
            // await axios.post('runGreedyAlgorithm')
            // .then((response) => {
            const resultData = response.data.data;
            const pointInfoList = resultData.point_info;
            const actionLog = resultData.action_log;
            const metrics = resultData.metrics;

            // 1. 重置传感器和网关显示 (基于 point_info 状态)
            sensorSource.clear();
            gatewaySource.clear();
            schedulePathSource.clear();
            scheduleMarkerSource.clear();

            await sensorupload(false); // 第一次加载，清空旧的，画上全绿
            await gateWayupload(false);
            await stationupload();

            const sensorFeatures: any[] = [];
            const gatewayFeatures: any[] = [];

            pointInfoList.forEach((point: { lng: any; lat: any; type: string; }) => {
                const position = [point.lng, point.lat];
                if (point.type === 'sensor') {
                    // 使用红色图标表示需要充电
                    sensorFeatures.push(createIconFeature(redSensor, [16, 16], position));
                } else if (point.type === 'gateWay') {
                    gatewayFeatures.push(createIconFeature(redGateWay, [32, 32], position));
                }
            });
            sensorSource.addFeatures(sensorFeatures);
            gatewaySource.addFeatures(gatewayFeatures);

            // 2. 绘制路径和步骤
            const allPathFeatures: any[] = [];
            const allMarkerFeatures: any[] = [];
            let stepCounter = 1;

            actionLog.forEach((logEntry: { path_nodes: any[]; move_time: any; charge_time: any; uav_to: any; destination_type: any; }) => {
                // const pathNodesStr = logEntry.path_nodes;
                const pathNodes = logEntry.path_nodes;
                const moveTime = logEntry.move_time;
                const chargeTime = logEntry.charge_time;
                const uavTo = logEntry.uav_to;
                const destinationType = logEntry.destination_type;

                // const pathCoordinates = pathNodesStr
                //     .map((coordStr: string) => coordStr.split(',').map(Number))
                //     .filter((coord: string | any[]) => coord.length === 2 && !isNaN(coord[0]));
                const pathCoordinates = pathNodes.map(coord => {
                    // coord 现在是 [经度, 纬度] 的数字数组
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
                        {nodeId: uavTo, type: destinationType}
                    );
                    allMarkerFeatures.push(endMarker);
                    stepCounter++;
                }
            });

            schedulePathSource.addFeatures(allPathFeatures);
            scheduleMarkerSource.addFeatures(allMarkerFeatures);

            loading.close();
            showMetricsDialog(metrics);
            ElMessage.success('Scheduling completed!');
        }catch (error) {
            loading.close();
            console.error(error);
            ElMessage.error('算法运行出错');
        }
            // })
            // .catch((error: any) => {
            //     loading.close();
            //     console.error(error);
            //     ElMessage.error(String(error));
            // });
    };

    function sensordownload() { downloadFile('downloadSensors', 'sensors_data.json'); }
    function gateWaydownload() { downloadFile('downloadgateWays', 'gateWays_data.json'); }
    function stationdownload() { downloadFile('downloadStations', 'stations_data.json'); }
    function chargingSchedulingdownload() { downloadFile('downloadChargingScheduling', 'chargingScheduling_data.json'); }

    async function downloadFile(url: any, filename: string) {
        try {
            const baseUrl = window.location.origin;
            const apiUrl = `${baseUrl}/api/${url}`;
            const response = await fetch(apiUrl, { method: 'GET', headers: { 'Content-Type': 'application/json' }});

            if (!response.ok) throw new Error(`Download failed: ${response.status}`);
            const blob = await response.blob();
            if (blob.size === 0) throw new Error('Empty file received');

            const blobUrl = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = blobUrl;
            a.download = filename;
            document.body.appendChild(a);
            a.click();
            setTimeout(() => { window.URL.revokeObjectURL(blobUrl); document.body.removeChild(a); }, 100);
            ElMessage.success(`${filename} download success`);
        } catch (error: any) {
            ElMessage.error(`Download error: ${error.message}`);
        }
    }

    onMounted(() => {
        createMap();
    });

</script>

<style scoped>
    #map {
        width: 100%;
        height: 85vh;
    }

    .yellow-color { background-color: #FFB800; color: white; }
    .green-color { background-color: #009688; color: white; }
    .green-color:hover { background-color: #33ABA0; }
</style>