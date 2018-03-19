// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <thread>

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams
#include <string>
#include <chrono>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#ifdef HAVE_OPENCV
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

namespace {
    static int s_image_counter = 0;
    static bool s_save_thread_running = false;
    static rs2::colorizer s_color_map;
    static int S_FPS = 8;
} // End of anonymous namespace

void saveFrame(rs2::frame&& frame, rs2_stream streamType, int sequence = 0)
{
    if (auto vf = frame.as<rs2::video_frame>())
    {
        // Write images to disk
        //if(vf.is<rs2::depth_frame>()) vf = s_color_map(frame);
        std::string type;
        unsigned int IMAGE_SIZE;
        float width = vf.get_width();
        float height = vf.get_height();
        switch(streamType)
        {
            case RS2_STREAM_DEPTH:
                type = "depth";
                #ifdef HAVE_OPENCV
                    IMAGE_SIZE = width * height * 3;
                #endif
                break;
            case RS2_STREAM_COLOR:
                #ifdef HAVE_OPENCV
                    IMAGE_SIZE = width * height * 3;
                #endif
                type = "color";
                break;
            case RS2_STREAM_INFRARED:
                #ifdef HAVE_OPENCV
                    IMAGE_SIZE = width * height;
                #endif
                type = "infrared";
                break;
        }
        std::stringstream png_file;
        png_file << type << "-" << sequence << "_" << s_image_counter << ".png";

        #ifdef HAVE_OPENCV
            if(streamType == RS2_STREAM_DEPTH || streamType == RS2_STREAM_COLOR) {
                cv::Mat rgbMat(height, width, CV_8UC3);
                memcpy(rgbMat.data, vf.get_data(), sizeof(unsigned char) * IMAGE_SIZE);
                cv::imwrite(png_file.str(), rgbMat);
            }else if (streamType == RS2_STREAM_INFRARED) {
                cv::Mat rgbMat(height, width, CV_8UC1);
                memcpy(rgbMat.data, vf.get_data(), sizeof(unsigned char) * IMAGE_SIZE);
                cv::imwrite(png_file.str(), rgbMat);
            }
        #else
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                       vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
        #endif
        std::cout << "Saved " << png_file.str() << std::endl;
        
        // Record per-frame metadata for UVC streams
        std::stringstream csv_file;
        csv_file << vf.get_profile().stream_name()
        << "-metadata.csv";
        metadata_to_csv(vf, csv_file.str());
    }
}

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
//    const auto CAPACITY = 100; // allow max latency of 5 frames
//    rs2::frame_queue depthQueue(CAPACITY);
//    rs2::frame_queue colorQueue(CAPACITY);
//    rs2::frame_queue infraredQueue0(CAPACITY);
//    rs2::frame_queue infraredQueue1(CAPACITY);
//    s_save_thread_running = true;
//    std::thread depthT([&]() {
//        while (s_save_thread_running)
//        {
//            rs2::frame frame;
////            int counter = 4;
////            int infraredCount = 0;
////            while(queue.poll_for_frame(&frame) && counter > 0) {
////                auto type = frame.get_profile().stream_type();
////                if(type == RS2_STREAM_INFRARED) saveFrame(std::move(frame), frame.get_profile().stream_type(), infraredCount++);
////                else saveFrame(std::move(frame), frame.get_profile().stream_type());
////                --counter;
////            }
////            if(counter == 0) ++s_image_counter;
////            else if (counter == 4)
////            {
////                std::cout << "No images yet, waiting\n";
////            }
////            else
////            {
////                std::cerr << "There should be four images per capture, something is wrong!\n";
////                exit(-1);
////            }
//            if(depthQueue.poll_for_frame(&frame)) saveFrame(std::move(frame), frame.get_profile().stream_type());
//        }
//    });
//    std::thread colorT([&]() {
//        while (s_save_thread_running)
//        {
//            rs2::frame frame;
//            if(colorQueue.poll_for_frame(&frame)) saveFrame(std::move(frame), frame.get_profile().stream_type());
//        }
//    });
//
//    std::thread infraredT0([&]() {
//        while (s_save_thread_running)
//        {
//            rs2::frame frame;
//            if(infraredQueue0.poll_for_frame(&frame)) saveFrame(std::move(frame), frame.get_profile().stream_type());
//        }
//    });
//
//    std::thread infraredT1([&]() {
//        while (s_save_thread_running)
//        {
//            rs2::frame frame;
//            if(infraredQueue1.poll_for_frame(&frame)) saveFrame(std::move(frame), frame.get_profile().stream_type(), 1);
//        }
//    });
    
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");
    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, color_image, infrared0_image, infrared1_image;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2);
    //cfg.enable_all_streams();

    
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();
    
    while(app) // Application still alive?
    {
        auto data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        rs2::frame depth = color_map(data.get_depth_frame()); // Find and colorize the depth data
        rs2::frame color = data.get_color_frame();            // Find the color data
        auto infrared0 = data.get_infrared_frame(0);
        auto infrared1 = data.get_infrared_frame(1);
        // Render depth on to the first half of the screen and color on to the second
        depth_image.render(depth, { 0,               0, app.width() / 2, app.height() / 2 });
        color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() / 2});
        infrared0_image.render(infrared0, { 0, app.height() / 2, app.width() / 2, app.height() / 2 });
        infrared1_image.render(infrared1, { app.width() / 2, app.height() / 2, app.width() / 2, app.height() / 2 });

        //std::cout << "enquing depth frame: " << depth.get_profile().stream_type() << "\n";
        //depthQueue.enqueue(depth);
        //std::cout << "enquing color frame: " << color.get_profile().stream_type() << "\n";
        //colorQueue.enqueue(color);
        //std::cout << "enquing infrared frame 0: " << infrared0.get_profile().stream_type() << "\n";
        //infraredQueue0.enqueue(infrared0);
        //std::cout << "enquing infrared frame 1: " << infrared1.get_profile().stream_type() << "\n";
        //infraredQueue1.enqueue(infrared1);
        
        //std::cout << "depth format: " << depth.get_profile().format() << "\n";
        saveFrame(std::move(depth), depth.get_profile().stream_type());
        //std::cout << "color format: " << color.get_profile().format() << "\n";
        saveFrame(std::move(color), color.get_profile().stream_type());
        //std::cout << "infrared format: " << infrared0.get_profile().format() << "\n";
        saveFrame(std::move(infrared0), infrared0.get_profile().stream_type());
        saveFrame(std::move(infrared1), infrared1.get_profile().stream_type());
        ++s_image_counter;
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000 / S_FPS));
    }

    //s_save_thread_running = false;
    
//    if(depthT.joinable()) depthT.join();
//    if(colorT.joinable()) colorT.join();
//    if(infraredT0.joinable()) infraredT0.join();
//    if(infraredT1.joinable()) infraredT1.join();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;
    
    csv.open(filename);
    
    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";
    
    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
            << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }
    
    csv.close();
}
                


