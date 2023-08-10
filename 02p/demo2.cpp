#include "libobsensor/ObSensor.hpp"

#include "opencv2/opencv.hpp"
#include <iostream>
#include <thread>
#include "../conio.h"
#include "../window.hpp"

#define KEY_ESC 27
#define KEY_SPACE 32

extern "C"
{

// 保存深度图为png格式
std::string saveDepth(std::shared_ptr<ob::DepthFrame> depthFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    //std::string depthName = "./Img/Depth_" + std::to_string(depthFrame->timeStamp()) + ".png";
    std::string depthName = "./Img/Depth_00.png";
    cv::Mat depthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
    cv::imwrite(depthName, depthMat, compression_params);
    std::cout << "Depth saved:" << depthName << std::endl;
    return depthName;
}

// 保存彩色图为png格式
std::string saveColor(std::shared_ptr<ob::ColorFrame> colorFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    //std::string colorName = "./Img/Color_" + std::to_string(colorFrame->timeStamp()) + ".png";
    std::string colorName = "./Img/Color_00.png";
    cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
    cv::imwrite(colorName, colorRawMat, compression_params);
    std::cout << "Color saved:" << colorName << std::endl;
    return colorName;
}

std::string SaveImg() {

    // 创建pipeline
    ob::Pipeline pipeline;

    // 通过创建Config来配置Pipeline要启用或者禁用哪些流
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    int colorCount = 0;
    int depthCount = 0;
    int key_catch = 0;
    int key_quit = 0;
    std::string s1, s2;

    
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
        colorCount = -1;
        std::cerr << "Current device is not support color sensor!" << std::endl;
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

    // 创建格式转换Filter
    ob::FormatConvertFilter formatConverFilter;

    // 启动
    pipeline.start(config);

    int frameCount = 0;
    while(true) {
        // 等待一帧数据，超时时间为100ms
        auto frameset = pipeline.waitForFrames(100);
        if(frameset == nullptr) {
            std::cout << "The frameset is null!" << std::endl;
            continue;
        }


        // 过滤前面3帧数据，待数据稳定后再进行保存
        if(frameCount < 3) {
            frameCount++;
            continue;
        }

        // 获取彩色和深度帧
        auto colorFrame = frameset->colorFrame();
        auto depthFrame = frameset->depthFrame();

        if(colorFrame != nullptr && colorCount < 1) {
            // 保存彩色图
            colorCount++;
            /*  保留这段会导致无法保存彩色图
            if(colorFrame->format() == OB_FORMAT_MJPG) {
                formatConverFilter.setFormatConvertType(FORMAT_MJPG_TO_RGB888);
            }
            else if(colorFrame->format() == OB_FORMAT_UYVY) {
                formatConverFilter.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
            }
            else if(colorFrame->format() == OB_FORMAT_YUYV) {
                formatConverFilter.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
            }
            else {
                std::cout << "Color format is not support!" << std::endl;
                continue;
            }
            colorFrame = formatConverFilter.process(colorFrame)->as<ob::ColorFrame>();
            */
            formatConverFilter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
            colorFrame = formatConverFilter.process(colorFrame)->as<ob::ColorFrame>();
            s1 = saveColor(colorFrame);
        }

        if(depthFrame != nullptr && depthCount < 1) {
            // 保存深度图
            depthCount++;
            s2 = saveDepth(depthFrame);
        }


        // 当彩色图和深度图都保存1张时退出程序
        if(depthCount == 1 && (colorCount == 1 || colorCount == -1)) {
            // 停止pipeline
            pipeline.stop();
            return s1 + "&" + s2;
        }
    }

    return "false";
}



}