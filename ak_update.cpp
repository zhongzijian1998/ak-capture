//
// Created by allen on 2020/1/2.
//

#include <iostream>
#include <azure_wrapper.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;
int main() {
	cv::Mat color_image, depth_image, transformed_depth_image, index_map_image;
	std::vector<uint32_t> body_identities;
	std::vector<float> body_keypoints_3d;
	std::vector<float> body_keypoints_2d;
	uint64_t timestamp;
	uint8_t image_save_flag = 0;
	uint64_t image_number = 0;
	char color_name[255], transformed_name[255], index_map_name[255];
	auto body_handle = new body_wrapper();
	body_handle->open_device(0);
	body_handle->start_capture();
	vector<Mat> save_color, save_depth;
	while (true) {
		//long t0 = cv::getTickCount();
		if (body_handle->get_all_data(color_image, transformed_depth_image, index_map_image, timestamp)) {

			/*			if (body_identities.size() > 0) {
							for (int i = 0; i < body_identities.size(); i++) {
								std::cout << body_identities[i] << " ";
							}
							std::cout << std::endl;
						}
						else
							std::cout << "false" << std::endl;
						for (int i = 0; i < body_keypoints_2d.size() / 2; i++) {
							cv::circle(color_image, cv::Point(body_keypoints_2d[i * 2], body_keypoints_2d[i * 2 + 1]), 3, cv::Scalar(0, 255, 0));
						}*/
			cv::imshow("show", color_image);
			//cv::imshow("transformed_depth_image", transformed_depth_image);
			//cv::imshow("index_map_image", index_map_image);
			if (image_save_flag)
			{
				save_color.emplace_back(color_image);
				save_depth.emplace_back(transformed_depth_image);
				image_number++;
				std::cout << image_number << std::endl;
				if (image_number == 800) break;
			}
			int key = cv::waitKey(1);
			if (key == 115 && image_save_flag == 0)
				image_save_flag = 1;//s保存
			else
				if (key == 27)  //esc
					break;
		}
		//long t1 = cv::getTickCount();
		//std::cout << "show time: " << (t1 - t0) / cv::getTickFrequency() * 1000 << " ms" << std::endl;
		//else
		//	std::cout << "false" << std::endl;
	}
	body_handle->stop_capture();
	body_handle->close_device();
	destroyAllWindows();
	if (image_save_flag)
		for (int n = 0; n < save_color.size(); n++)
		{
			sprintf_s(color_name, "D:\\ak_data\\scene2\\nobody2\\color\\%06d.png", n);
			sprintf_s(transformed_name, "D:\\ak_data\\scene2\\nobody2\\depth\\%06d.png", n);
			//sprintf_s(index_map_name, "c:\\ak_images\\index_map\\%05d.png", image_number);
			imwrite(color_name, save_color[n]);
			imwrite(transformed_name, save_depth[n]);
			//imwrite(index_map_name, index_map_image);
		}
	delete body_handle;
	std::cout << "exit normally" << std::endl;
	return 0;
}
