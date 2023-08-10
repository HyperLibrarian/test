#include "libobsensor/ObSensor.hpp"
#include "../conio.h"
#include <iostream>
#include <thread>

#define ESC 27

extern "C"
{
std::shared_ptr<ob::Pipeline> pipeline = NULL;

void CreateAndStartWithConfig() {
    //按配置文件的流配置启动流，如果没有配置文件，将使用第0个流配置启动流
    try {
        pipeline->start(nullptr);
    }
    catch(...) {
        std::cout << "Pipeline start failed!" << std::endl;
    }
}

//设备连接回调
void DeviceConnectCallback(std::shared_ptr<ob::DeviceList> connectList) {
    std::cout << "Device connect: " << connectList->deviceCount() << std::endl;
    if(connectList->deviceCount() > 0) {
        if(pipeline == NULL) {
            pipeline = std::make_shared<ob::Pipeline>();
            CreateAndStartWithConfig();
        }
    }
}

//设备断开回调
void DeviceDisconnectCallback(std::shared_ptr<ob::DeviceList> disconnectList) {
    std::cout << "Device disconnect: " << disconnectList->deviceCount() << std::endl;
    if(disconnectList->deviceCount() > 0) {

        try {
            pipeline->stop();
        }
        catch(ob::Error &e) {
            std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType()
                      << std::endl;
        }
        pipeline.reset();
    }
}

int USBReboot() {

    //创建上下文
    ob::Context ctx;

    //注册设备回调
    ctx.setDeviceChangedCallback([](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList) {
        DeviceDisconnectCallback(removedList);
        DeviceConnectCallback(addedList);
    });

    //获取当前设备数量，并开流
    if(ctx.queryDeviceList()->deviceCount() > 0) {
        if(pipeline == NULL) {
            pipeline = std::make_shared<ob::Pipeline>();
            CreateAndStartWithConfig();
        }
    }

    
    if(pipeline) {
                    pipeline->getDevice()->reboot();
                    return 1;
    }

    return 0;
}

}
