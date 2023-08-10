#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "../conio.h"
#include "../window.hpp"

#define KEY_ESC 27
#define KEY_SPACE 32

extern "C"
{

// 保存深度图为png格式
void saveDepth(std::shared_ptr<ob::DepthFrame> depthFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string depthName = "./Img/Depth_01.png";
    cv::Mat depthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
    cv::imwrite(depthName, depthMat, compression_params);
    std::cout << "Depth saved:" << depthName << std::endl;
}

// 保存彩色图为png格式
void saveColor(std::shared_ptr<ob::ColorFrame> colorFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string colorName = "./Img/Color_01.png";
    cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
    cv::imwrite(colorName, colorRawMat, compression_params);
    std::cout << "Color saved:" << colorName << std::endl;
}

auto framePipeline() {
    // 创建pipeline
    ob::Pipeline pipeline;
    // 通过创建Config来配置Pipeline要启用或者禁用哪些流
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
 
    try {
        // 获取彩色相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        std::shared_ptr<ob::VideoStreamProfile> colorProfile  = nullptr;
        try {
            // 根据指定的格式查找对应的Profile,优先选择RGB888格式
            colorProfile = colorProfiles->getVideoStreamProfile(640, 0, OB_FORMAT_RGB, 30);
        }
        catch(ob::Error &e) {
            //没找到指定格式后查找默认的Profile进行开流
            colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        config->enableStream(colorProfile);
    }
    catch(ob::Error &e) {
        // 没有Color Sensor
        std::cerr << "Current device is not support color sensor!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // 获取深度相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
    auto depthProfiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
    try {
        //根据指定的格式查找对应的Profile,优先查找Y16格式
        depthProfile = depthProfiles->getVideoStreamProfile(640, 0, OB_FORMAT_Y16, 30);
    }
    catch(ob::Error &e) {
        //没找到指定格式后查找默认的Profile进行开流
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
    }
    config->enableStream(depthProfile);

    // 启动
    pipeline.start(config);

    auto frameset = pipeline.waitForFrames(100);
    

    return frameset;
}

void saveColorImg(auto frameset){
    if(frameset == nullptr) {
            std::cout << "The frameset is null!" << std::endl;
    }
    ob::FormatConvertFilter formatConverFilter;
    auto colorFrame = frameset->colorFrame();
    if(colorFrame != nullptr) {
        // 保存彩色图
        //formatConverFilter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
        //colorFrame = formatConverFilter.process(colorFrame)->as<ob::ColorFrame>();
        saveColor(colorFrame);
    }

}

void saveDepthImg(auto frameset){
     if(frameset == nullptr) {
            std::cout << "The frameset is null!" << std::endl;
    }
    ob::FormatConvertFilter formatConverFilter;
    auto depthFrame = frameset->depthFrame();
    if(depthFrame != nullptr) {
        // 保存深度图
        saveDepth(depthFrame);
    }

}


}