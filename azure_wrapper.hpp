//
// Created by allen on 2019/12/30. K4A_COLOR_CONTROL_MODE_MANUAL
//

#ifndef AZURE_POSE_LIB_AZURE_WRAPPER_HPP
#define AZURE_POSE_LIB_AZURE_WRAPPER_HPP

#include <k4a/k4a.h>
//#include <k4abt.h>
#include <iostream>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>
#include <mutex>
using namespace cv;

class body_wrapper {
public:
	body_wrapper();
	bool open_device(int gpu_id);
	bool start_capture();
	void stop_capture();
	bool get_all_data(cv::Mat &color_image, cv::Mat &transformed_depth_image, cv::Mat &out_index_map_image,
		 uint64_t &timestamp);
	void close_device();

private:
	void get_images_data();
	//void get_bodies_data();
	k4a_device_t device;
	k4a_device_configuration_t device_config;
	k4a_calibration_t sensor_calibration;
//	k4abt_tracker_t tracker;
//	k4abt_tracker_configuration_t tracker_config;
	k4a_transformation_t transformation;
	k4a_calibration_t calibration;
	bool is_running;
	std::tuple<cv::Mat, cv::Mat, cv::Mat, uint64_t> images_data; // color depth transformed_depth
	std::tuple<std::vector<uint32_t>, std::vector<float>, std::vector<float> > bodies_data; // keypoints_3d, keypoints_2d
	std::thread *t_capture_images{};
	//std::thread *t_capture_bodies{};
	std::mutex m_images;
	//std::mutex m_bodies;
	float depth_alpha;
	float depth_beta;

	k4a_capture_t sensor_capture;
	std::mutex m_capture;
	k4a_image_t transformed_depth_image;
	k4a_image_t color_image;
	k4a_image_t depth_image;
	k4a_image_t body_index_map;
	k4a_image_t body_index_map_image;
	std::vector<float> body_keypoints_3d;
	std::vector<float> body_keypoints_2d;
	std::vector<uint32_t> body_identities;
};

#endif //AZURE_POSE_LIB_AZURE_WRAPPER_HPP
