#include "libobsensor/ObSensor.hpp"

#include "opencv2/opencv.hpp"
#include <iostream>
#include "../conio.h"
#include "../window.hpp"

#define KEY_ESC 27
#define KEY_SPACE 32

// 保存深度图为png格式
void saveDepth(std::shared_ptr<ob::DepthFrame> depthFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string depthName = "./Img/Depth_" + std::to_string(depthFrame->timeStamp()) + ".png";
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
    std::string colorName = "./Img/Color_" + std::to_string(colorFrame->timeStamp()) + ".png";
    cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
    cv::imwrite(colorName, colorRawMat, compression_params);
    std::cout << "Color saved:" << colorName << std::endl;
}

int main(int argc, char **argv) try {
    // 创建pipeline
    ob::Pipeline pipeline;
    // 通过创建Config来配置Pipeline要启用或者禁用哪些流
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    int colorCount = 0;
    int depthCount = 0;
    int key_catch = 0;
    int key_quit;

    std::shared_ptr<ob::VideoStreamProfile> colorProfile  = nullptr;
    try {
        // 获取彩色相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
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

    // 创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("ColorViewer", colorProfile->width(), colorProfile->height());

   // 获取镜像属性是否有可写的权限
    try {
        if(pipeline.getDevice()->isPropertySupported(OB_PROP_COLOR_MIRROR_BOOL, OB_PERMISSION_WRITE)) {
            // 设置镜像
            pipeline.getDevice()->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, true);
        }
    }
    catch(const ob::Error &e) {
        std::cerr << "Failed to set mirror property: " << e.getMessage() << std::endl;
    }


    int frameCount = 0;
    while(true) {
        // 等待一帧数据，超时时间为100ms
        auto frameset = pipeline.waitForFrames(100);
        if(frameset == nullptr) {
            std::cout << "The frameset is null!" << std::endl;
            continue;
        }

        // 在窗口中渲染一组帧数据，这里只渲染彩色帧，但是也要以数组的形式传入，RENDER_SINGLE表示
        // 只渲染数组中的第一个帧
        app.render({ frameset->colorFrame() }, RENDER_SINGLE);

        // 过滤前面5帧数据，待数据稳定后再进行保存
        if(frameCount < 5) {
            frameCount++;
            continue;
        }

        // 获取彩色和深度帧
        auto colorFrame = frameset->colorFrame();
        auto depthFrame = frameset->depthFrame();

        if(colorFrame != nullptr && colorCount < 5) {
            // 保存彩色图
            colorCount++;
            /*   保留这段会导致无法保存彩色图
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
            saveColor(colorFrame);
        }

        if(depthFrame != nullptr && depthCount < 5) {
            // 保存深度图
            depthCount++;
            saveDepth(depthFrame);
        }


        // 当彩色图和深度图都保存5张时按ESC键退出程序
        if(depthCount == 5 && (colorCount == 5 || colorCount == -1)) {
            // 停止pipeline
            pipeline.stop();
            std::cout << "The demo is over, please press ESC to exit manually!" << std::endl;
            key_quit = getch();
            // 按ESC键退出
            if(key_quit == KEY_ESC)
                break;
        }
    }

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    return EXIT_FAILURE;
}
