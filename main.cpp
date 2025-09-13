#include "ArenaApi.h"
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
// #include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

bool parseFrame(Arena::IDevice* pDevice)
{
    std::cout << "正在解析帧数据 ..." << std::endl;

    GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

    /// *************************************** 相机设置备份 ******************************************
    GenICam::gcstring acquisitionModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "AcquisitionMode");
    GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat");
    GenICam::gcstring exposureTimeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureTimeSelector");
    GenICam::gcstring conversionGainInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "ConversionGain");
    bool acquisitionFrameRateEnableInitial = Arena::GetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable");
    double acquisitionFrameRateInitial = Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate");
    GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");
    int64_t imageAccumulationInitial = Arena::GetNodeValue<int64_t>(pNodeMap, "Scan3dImageAccumulation");
    bool spatialFilterInitial = Arena::GetNodeValue<bool>(pNodeMap, "Scan3dSpatialFilterEnable");
    bool confidenceThresholdInitial = Arena::GetNodeValue<bool>(pNodeMap, "Scan3dConfidenceThresholdEnable");
    int64_t confidenceThresholdMinInitial = Arena::GetNodeValue<int64_t>(pNodeMap, "Scan3dConfidenceThresholdMin");
    /// **********************************************************************************************

    /// *************************************** 相机参数设置 ******************************************
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "AcquisitionMode", "Continuous");
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

    {
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", "Mono8");
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureTimeSelector", "Exp62_5Us");
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ConversionGain", "Low");
        Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
        Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate", 30);
    }
    {
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", "Coord3D_ABCY16");
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance5000mmMultiFreq");
        Arena::SetNodeValue<int64_t>(pNodeMap, "Scan3dImageAccumulation", 1);
        Arena::SetNodeValue<bool>(pNodeMap, "Scan3dSpatialFilterEnable", false);
        Arena::SetNodeValue<bool>(pNodeMap, "Scan3dConfidenceThresholdEnable", false);
        Arena::SetNodeValue<int64_t>(pNodeMap, "Scan3dConfidenceThresholdMin", 2000);
    }
    /// **********************************************************************************************

    /// *************************************** 启动相机数据流 ******************************************
    pDevice->StartStream();
    /// **********************************************************************************************

    const int TIMEOUT = 2000;
    bool running = true;

    // 获取坐标比例和偏移
    float scaleX, offsetX, scaleY, offsetY, scaleZ, offsetZ;
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateA");
    scaleX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));
    offsetX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateB");
    scaleY = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));
    offsetY = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateC");
    scaleZ = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));
    offsetZ = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));

    // 创建点云可视化器，OpenCV可视化窗口
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
    cv::namedWindow("Gray Image", cv::WINDOW_AUTOSIZE);

    // 帧率计算变量
    auto last_time = std::chrono::steady_clock::now();
    double fps = 0.0;
    int frame_count = 0;

    // 循环体
    while (running && !viewer.wasStopped())
    {
        Arena::IImage* pImage = pDevice->GetImage(TIMEOUT);
        if (pImage->IsIncomplete())
        {
            pDevice->RequeueBuffer(pImage);
            continue;
        }

        size_t width = pImage->GetWidth();
        size_t height = pImage->GetHeight();
        size_t size = width * height;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cv::Mat gray_image(height, width, CV_8UC1);
        pt_cloud->is_dense = false;
        pt_cloud->width = width;
        pt_cloud->height = height;
        pt_cloud->resize(size);

        const uint16_t* pData = reinterpret_cast<const uint16_t*>(pImage->GetData());
        for (size_t i = 0; i < size; i++)
        {
            float x = (float)((uint16_t)(*(pData + i * 4)));
            float y = (float)((uint16_t)(*(pData + i * 4 + 1)));
            float z = (float)((uint16_t)(*(pData + i * 4 + 2)));
            float intensity = (uint8_t)((int16_t)(*(pData + i * 4 + 3)) >> 8);

            // 计算图像中的行和列
            size_t row = i / width;
            size_t col = i % width;

            // 将 intensity 值写入 cv::Mat
            gray_image.at<uint8_t>(row, col) = intensity;

            if (z < 65535.0 && intensity != 0)
            {
                x = x * scaleX + offsetX;
                y = -(y * scaleY + offsetY);
                z = -(z * scaleZ + offsetZ);

                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = intensity;
                pt_cloud->points[i] = point;
            }
        }// end for all pixels

        // 计算帧率
        auto current_time = std::chrono::steady_clock::now();
        double delta_time = std::chrono::duration<double, std::milli>(current_time - last_time).count();
        if (delta_time > 0)
        {
            double current_fps = 1000.0 / delta_time; // 转换为 FPS
            fps = current_fps; // 直接显示当前 FPS
            frame_count++;
        }
        last_time = current_time;

        // 在灰度图像上显示 FPS
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
        cv::putText(gray_image, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);

        // 同时显示灰度图像和点云
        cv::imshow("Gray Image", gray_image);
        viewer.showCloud(pt_cloud);

        // 检查用户输入以退出
        int key = cv::waitKey(1);
        if (key == 27) // ESC key to exit
        {
            running = false;
        }

        pDevice->RequeueBuffer(pImage);//图像重排
    }

    // 销毁 OpenCV 窗口
    cv::destroyWindow("Gray Image");

    pDevice->StopStream();//停止采集

    /// *************************************** 恢复相机参数 ***************************************
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "AcquisitionMode", acquisitionModeInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", pixelFormatInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureTimeSelector", exposureTimeInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ConversionGain", conversionGainInitial);
    Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", acquisitionFrameRateEnableInitial);
    Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate", acquisitionFrameRateInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", operatingModeInitial);
    Arena::SetNodeValue<int64_t>(pNodeMap, "Scan3dImageAccumulation", imageAccumulationInitial);
    Arena::SetNodeValue<bool>(pNodeMap, "Scan3dSpatialFilterEnable", spatialFilterInitial);
    Arena::SetNodeValue<bool>(pNodeMap, "Scan3dConfidenceThresholdEnable", confidenceThresholdInitial);
    Arena::SetNodeValue<int64_t>(pNodeMap, "Scan3dConfidenceThresholdMin", confidenceThresholdMinInitial);
    /// ******************************************************************************************

    return true;
}

int main()
{
    bool exceptionThrown = false;
    try
    {
        Arena::ISystem* pSystem = Arena::OpenSystem();
        std::cout << "Arena SDK Version: " << Arena::GetVersion() << std::endl;
        pSystem->UpdateDevices(100);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();

        if (deviceInfos.size() != 0)
        {
            std::cout << "Found " << deviceInfos.size() << " devices." << std::endl;
            for (size_t i = 0; i < deviceInfos.size(); i++)
            {
                std::cout << "【相机 " << (i + 1) << " / " << deviceInfos.size() << "】" << std::endl;
                std::cout << "默认网关: " << deviceInfos[i].DefaultGatewayStr() << std::endl;
                std::cout << "设备版本: " << deviceInfos[i].DeviceVersion() << std::endl;
                std::cout << "IP地址: " << deviceInfos[i].IpAddressStr() << std::endl;
                std::cout << "MAC地址: " << deviceInfos[i].MacAddressStr() << std::endl;
                std::cout << "型号名称: " << deviceInfos[i].ModelName() << std::endl;
                std::cout << "序列号: " << deviceInfos[i].SerialNumber() << std::endl;
                std::cout << "子网掩码: " << deviceInfos[i].SubnetMaskStr() << std::endl;
                std::cout << "厂商名称: " << deviceInfos[i].VendorName() << std::endl;
                std::cout << "----------------------------------------" << std::endl;
            }
        }
        else
        {
            std::cout << "No camera connected";
            return 0;
        }

        Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);//创建设备
        if (!pDevice)
        {
            throw std::logic_error("No camera connected");
        }

        if (parseFrame(pDevice))//数据解析
        {
            std::cout << "相机数据解析成功!" << std::endl;
        }
        else
        {
            std::cout << "相机数据解析失败!" << std::endl;
        }

        pSystem->DestroyDevice(pDevice);//关闭设备
        Arena::CloseSystem(pSystem);//关闭系统
    }
    catch (GenICam::GenericException& ge)
    {
        std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
        exceptionThrown = true;
    }
    catch (std::exception& ex)
    {
        std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
        exceptionThrown = true;
    }
    catch (...)
    {
        std::cout << "\nUnexpected exception thrown\n";
        exceptionThrown = true;
    }

    return exceptionThrown ? -1 : 0;
}