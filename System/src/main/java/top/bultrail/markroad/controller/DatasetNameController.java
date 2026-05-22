package top.bultrail.markroad.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;
import top.bultrail.markroad.entity.*;

import top.bultrail.markroad.service.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.servlet.http.HttpSession;

@Controller
public class DatasetNameController {

    @Autowired
    DatasetNameService transformService;

    // 一键保存
    @ResponseBody
    @RequestMapping(value = {"/hhquicksaveNew", "/api/hhquicksaveNew"}, method = RequestMethod.POST, consumes = MediaType.APPLICATION_JSON_VALUE)
    public ResponseEntity<ResultEntity<String>> quicksave_new(@RequestBody QuickSave quick_save) {
        // 将数据保存在数据库
        try {
            transformService.quicksaveDB(quick_save);
            System.out.println("Data Save Success!");
            return ResponseEntity.ok(ResultEntity.successWithoutData());
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failed(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 保存数据集
    @ResponseBody
    @RequestMapping(value = {"/api/saveDataset"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<String>> saveDataset(@RequestBody Map<String, Object> requestBody) {
        String setName = (String) requestBody.get("datasetName");
        List<Double> currentLocation = (List<Double>) requestBody.get("current_location");
        try {
            transformService.saveDataset(setName, currentLocation);
            System.out.println("Dataset saved successfully");
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failed(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
        return ResponseEntity.ok(ResultEntity.successWithoutData());
    }

    // 删除数据集
    @ResponseBody
    @RequestMapping(value = {"/api/deleteDataset"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<String>> deleteDataset(@RequestParam(value = "datasetName") String setName) {
        try {
            transformService.deleteDataset(setName);
            System.out.println("Dataset deleted successfully");
            return ResponseEntity.ok(ResultEntity.successWithoutData());
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failedWithMessage(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 获取数据集信息
    @ResponseBody
    @RequestMapping(value = {"/api/searchDatasetInfo"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<List<DatasetInfo>>> searchDatasetInfo() {
        try {
            List<DatasetInfo> datasetInfos;
            datasetInfos = transformService.searchDatasetInfo();
            System.out.println("Search Datasets Information Successfully");
            return ResponseEntity.ok(ResultEntity.successWithDatasetInfo(datasetInfos, null));
        } catch (Exception e) {
            ResultEntity<List<DatasetInfo>> errorResponse = ResultEntity.failedWithMessage(e.getMessage());
            return new ResponseEntity<>(errorResponse, HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 对数据库表格进行重命名
    @ResponseBody
    @RequestMapping(value = {"/api/renameDataset"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<String>> renameDataset(@RequestBody RenameDataset request) {
        try {
            transformService.renameDataset(request.getOldName(), request.getNewName());
            System.out.println("Dataset renamed successfully");
            return ResponseEntity.ok(ResultEntity.successWithoutData());
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failedWithMessage(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }


    //数据集加载
    @ResponseBody
    @RequestMapping(value = {"/api/datasetLoad"}, method = RequestMethod.POST)
    public ResponseEntity<ResultEntity<List<Double>>> datasetLoad(@RequestParam(value = "datasetName") String setName) {
        try {
            List<Double> location = transformService.datasetLoad(setName);
            System.out.println("Dataset Load Success");
            return ResponseEntity.ok(ResultEntity.successWithDataDouble(location, null));
        } catch (Exception e) {
            return new ResponseEntity<>(ResultEntity.failedWithMessage(e.getMessage()), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

}
