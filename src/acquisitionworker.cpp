/*!
 * \file    acquisitionworker.cpp
 * \author  IDS Imaging Development Systems GmbH
 * \date    2022-06-01
 * \since   1.1.6
 *
 * \brief   The AcquisitionWorker class is used in a worker thread to capture
 *          images from the device continuously and do an image conversion into
 *          a desired pixel format.
 *
 * \version 1.1.1
 *
 * Copyright (C) 2020 - 2023, IDS Imaging Development Systems GmbH.
 *
 * The information in this document is subject to change without notice
 * and should not be construed as a commitment by IDS Imaging Development Systems GmbH.
 * IDS Imaging Development Systems GmbH does not assume any responsibility for any errors
 * that may appear in this document.
 *
 * This document, or source code, is provided solely as an example of how to utilize
 * IDS Imaging Development Systems GmbH software libraries in a sample application.
 * IDS Imaging Development Systems GmbH does not assume any responsibility
 * for the use or reliability of any portion of this document.
 *
 * General permission to copy or modify is hereby granted.
 */

#include "include/acquisitionworker.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <cmath>
#include <rosbag/bag.h>
#include <sensor_msgs/fill_image.h>

#include <peak_ipl/peak_ipl.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>

AcquisitionWorker::AcquisitionWorker(QObject* parent) : QObject(parent)
{
    m_running = false;
    m_frameCounter = 0;
    m_errorCounter = 0;

    // 1 byte for each channel of RGBa
    m_bytesPerPixel = 4;

    m_imageConverter = std::make_unique<peak::ipl::ImageConverter>();
}

void AcquisitionWorker::start()
{
    image_transport::ImageTransport it(nh);
#ifdef CAM_BAG
    rosbag::Bag camera_bag;
    camera_bag.open("/home/shreyash/workspace/ids_ros_ws/src/front_camera.bag", rosbag::bagmode::Write);
    camera_bag.setCompression(rosbag::compression::LZ4);
#endif
    pub = it.advertise("/right_cam/image_raw", 100);
    right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/right_cam/camera_info", 10);

    nh.getParam("/right_cam/image_width", img_width);
    nh.getParam("/right_cam/image_height", img_height);
    nh.getParam("/right_cam/distortion_model", distortion_model);

    nh.getParam("/right_cam/distortion_coefficients/data", D);
    nh.getParam("/right_cam/camera_matrix/data", K);
    nh.getParam("/right_cam/rectification_matrix/data", R);
    nh.getParam("/right_cam/projection_matrix/data", P);

    right_cam_info.height = img_height;
    right_cam_info.width = img_width;
    right_cam_info.distortion_model = distortion_model;

    right_cam_info.D = D;

    // K is of type std::vector<double>, converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
    boost::array<double, 9> K_array;
    if (K.size() == 9) {
        for (size_t i = 0; i < 9; ++i) {
            K_array[i] = K[i];
        }
    } else {
        // Error handler
        throw std::invalid_argument("K vector must have exactly 9 elements");
    }
    right_cam_info.K = K_array;

    // R is of type std::vector<double> converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
    boost::array<double, 9> R_array;
    if (R.size() == 9) {
        for (size_t i = 0; i < 9; ++i) {
            R_array[i] = R[i];
        }
    } else {
        // Error handler
        throw std::invalid_argument("R vector must have exactly 9 elements");
    }
    right_cam_info.R = R_array;

    // P is of type std::vector<double> converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
    boost::array<double, 12> P_array;
    if (P.size() == 12) {
        for (size_t i = 0; i < 12; ++i) {
            P_array[i] = P[i];
        }
    } else {
        // Error handler
        throw std::invalid_argument("P vector must have exactly 12 elements");
    }
    right_cam_info.P = P_array;


    try
    {
        // Lock critical features to prevent them from changing during acquisition
        m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

        // Determine image size
        m_imageWidth = m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->Value();
        m_imageHeight = m_nodemapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->Value();
        m_size = static_cast<const size_t>(m_imageWidth * m_imageHeight * m_bytesPerPixel);

        // Pre-allocate images for conversion that can be used simultaneously
        // This is not mandatory but it can increase the speed of image conversions
        size_t imageCount = 1;
        const auto inputPixelFormat = static_cast<peak::ipl::PixelFormatName>(
            m_nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")
                ->CurrentEntry()
                ->Value());

        m_imageConverter->PreAllocateConversion(
            inputPixelFormat, peak::ipl::PixelFormatName::BGRa8, m_imageWidth, m_imageHeight, imageCount);

        // Start acquisition
        m_dataStream->StartAcquisition();
        m_nodemapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();
        m_nodemapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->WaitUntilDone();
    }
    catch (const std::exception& e)
    {
        qDebug() << "Exception: " << e.what();
        emit messageBoxTrigger("Exception", e.what(), false);
    }

    m_running = true;

    while (m_running)
    {
        try
        {
            // Get buffer from device's datastream
            const auto buffer = m_dataStream->WaitForFinishedBuffer(5000);

            double chunkDataExposureTime_ms = -1;
            if (buffer->HasChunks() && m_enableChunks)
            {
                m_nodemapRemoteDevice->UpdateChunkNodes(buffer);

                // Get the value of the exposure time chunk
                const auto chunkData = m_nodemapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ChunkExposureTime")->Value();
                chunkDataExposureTime_ms = round(chunkData) / 1000.0;
            }

            QImage qImage(static_cast<int>(m_imageWidth), static_cast<int>(m_imageHeight), QImage::Format_RGB32);

            // Create IDS peak IPL image for debayering and convert it to RGBa8 format

            // Using the image converter ...
            auto image_processed = m_imageConverter->Convert(peak::BufferTo<peak::ipl::Image>(buffer),
                peak::ipl::PixelFormatName::BGRa8, qImage.bits(), static_cast<size_t>(qImage.byteCount()));

            // ... or without image converter
            // NOTE: The `ConvertTo` function re-allocates conversion buffers on every call
            //       using the `ImageConverter` should be preferred for performance.
            // peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(
            //     peak::ipl::PixelFormatName::BGRa8, qImage.bits(), static_cast<size_t>(qImage.byteCount()));
            
            // Queue buffer so that it can be used again
            m_dataStream->QueueBuffer(buffer);

            // Emit signal that the image is ready to be displayed
            emit imageReceived(qImage, chunkDataExposureTime_ms);
 
            uint8_t* data_ptr = image_processed.Data();

            std_msgs::Header head_info = std_msgs::Header();
            head_info.stamp = ros::Time::now();
            head_info.frame_id = "sensor_origin";

            // header info for the camera info msg
            right_cam_info.header = head_info;

            sensor_msgs::Image image_msg;
            image_msg.header = head_info;

            sensor_msgs::fillImage( image_msg,
                                    sensor_msgs::image_encodings::BGRA8,
                                    image_processed.Height(),
                                    image_processed.Width(),
                                    image_processed.Width()*4,
                                    data_ptr
                                    );     
            pub.publish(image_msg); // Publishing raw image
            
            right_cam_info_pub.publish(right_cam_info); // Publish the camera matrix details

            m_frameCounter++;
        }
        catch (const std::exception& e)
        {
            m_errorCounter++;

            qDebug() << "Exception: " << e.what();
            emit messageBoxTrigger("Exception", e.what(), false);
        }

        // Send signal with current frame and error counter
        emit counterChanged(m_frameCounter, m_errorCounter);
    }
#ifdef CAM_BAG
    camera_bag.close();
#endif
}

void AcquisitionWorker::setDataStream(std::shared_ptr<peak::core::DataStream> dataStream)
{
    m_dataStream = dataStream;
}

void AcquisitionWorker::setNodemapRemoteDevice(std::shared_ptr<peak::core::NodeMap> nodeMap)
{
    m_nodemapRemoteDevice = nodeMap;
}

void AcquisitionWorker::stop()
{
    m_running = false;
}

void AcquisitionWorker::setEnableChunks(bool enable)
{
    m_enableChunks = enable;
}

