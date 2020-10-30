//
// Created by allen on 2019/12/30.
//

/*
K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE  col1
//exp,   2^exp,   50Hz,   60Hz,

	{ -11,     488,    500,    500},

	{ -10,     977,   1250,   1250},

	{  -9,    1953,   2500,   2500},

	{  -8,    3906,  10000,   8330},

	{  -7,    7813,  20000,  16670},

	{  -6,   15625,  30000,  33330},

	{  -5,   31250,  40000,  41670},

	{  -4,   62500,  50000,  50000},

	{  -3,  125000,  60000,  66670},

	{  -2,  250000,  80000,  83330},

	{  -1,  500000, 100000, 100000},

	{   0, 1000000, 120000, 116670},

	{   1, 2000000, 130000, 133330}
*/



#include <windows.h>
#include "azure_wrapper.hpp"

body_wrapper::body_wrapper() {
	device = nullptr;
	device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	//tracker = nullptr;
	//tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	is_running = false;
	sensor_capture = nullptr;
	color_image = nullptr;
	depth_image = nullptr;
	transformed_depth_image = nullptr;
	transformation = nullptr;
}

bool body_wrapper::open_device(int gpu_id) {
	k4a_device_open(0, &device);
	k4a_device_set_color_control(device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, 7813);
	k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution,&sensor_calibration);
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	device_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	device_config.synchronized_images_only = true;
	//tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
	//tracker_config.gpu_device_id = gpu_id;
	//k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker);
	if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &calibration))
	{
		printf("Failed to get calibration\n");
		return false;
	}
	transformation = k4a_transformation_create(&calibration);
	if (k4a_device_start_cameras(device, &device_config) != K4A_RESULT_SUCCEEDED)
		return false;

	switch (device_config.depth_mode) {
	case k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED:
		depth_alpha = 255.0f / (3860.0f - 500.0f);
		depth_beta = -500.0f*depth_alpha;
		break;
	case k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_2X2BINNED:
		depth_alpha = 255.0f / (5460.0f - 500.0f);
		depth_beta = -500.0f*depth_alpha;
		break;
	case k4a_depth_mode_t::K4A_DEPTH_MODE_WFOV_UNBINNED:
		depth_alpha = 255.0f / (2210.0f - 250.0f);
		depth_beta = -250.0f*depth_alpha;
		break;
	case k4a_depth_mode_t::K4A_DEPTH_MODE_WFOV_2X2BINNED:
		depth_alpha = 255.0f / (2880.0f - 250.0f);
		depth_beta = -250.0f*depth_alpha;
		break;
	default:
		depth_alpha = 1.0f;
	}

	return true;
}

bool body_wrapper::start_capture() {
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		1920,
		1080,
		1920 * (int)sizeof(uint16_t),
		&transformed_depth_image))
	{
		std::cout << "Failed to create transformed depth image" << std::endl;
		return false;
	}
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM8,
		1920,
		1080,
		1920 * (int)sizeof(uint8_t),
		&body_index_map_image))
	{
		std::cout << "Failed to create body index map image" << std::endl;
		return false;
	}
	is_running = true;
	
	t_capture_images = new std::thread(&body_wrapper::get_images_data, &(*this));
	//t_capture_bodies = new std::thread(&body_wrapper::get_bodies_data, &(*this));
	t_capture_images->detach();
	//t_capture_bodies->detach();

	return true;
}

bool body_wrapper::get_all_data(cv::Mat &out_color_image, cv::Mat &out_transformed_depth_image, cv::Mat &out_index_map_image,
	 uint64_t &out_timestamp) {
	
	m_images.lock();
	if (color_image == nullptr || depth_image == nullptr) {
		m_images.unlock();
		return false;
	}
	int color_rows = k4a_image_get_height_pixels(color_image);
	int color_cols = k4a_image_get_width_pixels(color_image);
	//int depth_rows = k4a_image_get_height_pixels(depth_image);
	//int depth_cols = k4a_image_get_width_pixels(depth_image);

	out_color_image = cv::Mat(color_rows, color_cols, CV_8UC4, (void*)k4a_image_get_buffer(color_image), cv::Mat::AUTO_STEP).clone();
	//img_depth_res = cv::Mat(depth_rows, depth_cols, CV_16U, (void*)k4a_image_get_buffer(depth_image), cv::Mat::AUTO_STEP).clone();

	//transformed

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
	if(K4A_RESULT_SUCCEEDED!=k4a_transformation_depth_image_to_color_camera_custom(transformation, depth_image,
		body_index_map, transformed_depth_image, body_index_map_image,K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST, 0))
	{
		m_images.unlock();
		std::cout << "Failed to compute transformed depth image" << std::endl;
		return false;
	}
	m_images.unlock();
	
	out_transformed_depth_image = cv::Mat(color_rows, color_cols, CV_16U, k4a_image_get_buffer(transformed_depth_image), cv::Mat::AUTO_STEP).clone();

	out_timestamp = k4a_image_get_system_timestamp_nsec(color_image) / 1000000;
	//m_bodies.unlock();
	//out_index_map_image = cv::Mat(color_rows, color_cols, CV_8U, k4a_image_get_buffer(body_index_map_image), cv::Mat::AUTO_STEP).clone();
	//cv::convertScaleAbs(img_depth_res, img_depth_res, depth_ratio);
	//long t0 = cv::getTickCount();
	//cv::convertScaleAbs(out_transformed_depth_image, out_transformed_depth_image, depth_alpha, depth_beta);
	//long t1 = cv::getTickCount();
	//std::cout << "transform time: " << (t1 - t0) / cv::getTickFrequency() * 1000 << " ms" << std::endl;
	//m_bodies.lock();
	//out_body_identities = std::get<0>(bodies_data);
	//out_body_keypoints_3d = std::get<1>(bodies_data);
	//out_body_keypoints_2d = std::get<2>(bodies_data);
	//m_bodies.unlock();

	return true;
}

void body_wrapper::get_images_data() {
	while (is_running) {
		long t0 = cv::getTickCount();

		m_capture.lock();
		if(sensor_capture != nullptr)
			k4a_capture_release(sensor_capture);
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, -1);
		if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
			m_capture.unlock();
			std::cout << "Get depth capture returned error: " << get_capture_result << std::endl;
			continue;
		}

		m_images.lock();
		if(color_image != nullptr)
			k4a_image_release(color_image);
		if(depth_image != nullptr)
			k4a_image_release(depth_image);
		color_image = k4a_capture_get_color_image(sensor_capture);
		depth_image = k4a_capture_get_depth_image(sensor_capture);
		m_images.unlock();
		m_capture.unlock();
		
		Sleep(1);

		long t1 = cv::getTickCount();
		std::cout << "images time: " << (t1 - t0) / cv::getTickFrequency() * 1000 << " ms" << std::endl;
	}
}

//void body_wrapper::get_bodies_data() {
//	while (false) {
//		long t0 = cv::getTickCount();
//		m_capture.lock();
//		if (sensor_capture == nullptr) {
//			m_capture.unlock();
//			continue;
//		}
//		//k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, -1);
//		m_capture.unlock();
//		if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
//			std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
//			continue;
//		}
//		
//		//k4abt_frame_t body_frame = nullptr;
//		//k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, -1);
//		
//		//long t2 = cv::getTickCount();
//		//std::cout << "body step1 time: " << (t2 - t0) / cv::getTickFrequency() * 1000 << " ms" << std::endl;
//		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
//			body_keypoints_3d.clear();
//			body_keypoints_2d.clear();
//			body_identities.clear();
//
//			//k4a_capture_t original_capture = k4abt_frame_get_capture(body_frame);
//			//k4a_image_t index_map = k4abt_frame_get_body_index_map(body_frame);
//
//			uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
//			for (uint32_t i = 0; i < num_bodies; i++) {
//				k4abt_body_t body;
//				k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton);
//				body.id = k4abt_frame_get_body_id(body_frame, i);
//				body_identities.push_back(body.id);
//				for (auto &joint : body.skeleton.joints) {
//					const k4a_float3_t &jointPosition = joint.position;
//
//					k4a_float2_t joint_color_2d;
//					k4a_float3_t joint_color_3d;
//					int valid_joint;
//					k4a_calibration_3d_to_2d(&sensor_calibration, &jointPosition,
//						K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &joint_color_2d, &valid_joint);
//					k4a_calibration_3d_to_3d(&sensor_calibration, &jointPosition,
//						K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &joint_color_3d);
//
//					body_keypoints_3d.push_back(joint_color_3d.xyz.x);
//					body_keypoints_3d.push_back(joint_color_3d.xyz.y);
//					body_keypoints_3d.push_back(joint_color_3d.xyz.z);
//					body_keypoints_2d.push_back(joint_color_2d.xy.x);
//					body_keypoints_2d.push_back(joint_color_2d.xy.y);
//				}
//			}
//			
//			m_bodies.lock();
//
//			body_index_map = k4abt_frame_get_body_index_map(body_frame);
//			bodies_data = std::make_tuple(body_identities, body_keypoints_3d, body_keypoints_2d);
//			m_bodies.unlock();
//			k4abt_frame_release(body_frame);
//		}
//
//		long t1 = cv::getTickCount();
//		std::cout << "bodies time: " << (t1 - t0) / cv::getTickFrequency() * 1000 << " ms" << std::endl;
//
//		
//	}
//}

void body_wrapper::stop_capture() {
	is_running = false;
	//t_capture_images->join();
	//t_capture_bodies->join();
	
//	delete t_capture_bodies;
	if (sensor_capture != nullptr)
		k4a_capture_release(sensor_capture);
	if (color_image != nullptr)
		k4a_image_release(color_image);
	if (depth_image != nullptr)
		k4a_image_release(depth_image);
	if (transformed_depth_image != nullptr)
		k4a_image_release(transformed_depth_image);
	if (body_index_map != nullptr)
		k4a_image_release(body_index_map);
	if (body_index_map_image != nullptr)
		k4a_image_release(body_index_map_image);
	delete t_capture_images;
}

void body_wrapper::close_device() {
	//k4abt_tracker_shutdown(tracker);
	//k4abt_tracker_destroy(tracker);

	k4a_transformation_destroy(transformation);

	k4a_device_stop_cameras(device);
	k4a_device_close(device);
}

