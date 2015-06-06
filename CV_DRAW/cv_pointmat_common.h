#pragma once
#include <opencv2\opencv.hpp>
#include <functional>

cv::Mat filter_pointmat(const cv::Mat& in, std::function<bool(const cv::Vec4f&, void*)> criteria, void* data);


//Take Care: don't use the z-value
void divide_pointmat_by_z(cv::Mat& pointmat);

//call divide_pointmat_by_z before calling this
void draw_pointmat_on_image(cv::Mat& image, const cv::Mat& pointmat, const cv::Vec3b& color, int ptsize=1);

cv::Mat pointvec_to_pointmat(std::vector<cv::Vec4f>& pointvec);
void display_pointmat(const std::string& window_name, int window_width, int window_height, const cv::Mat& camera_matrix, const cv::Mat& camera_pose_orig,
	const std::vector<cv::Mat>& pointmats, const std::vector<cv::Vec3b>& colors);
void display_pointmat(const std::string& window_name, int window_width, int window_height, const cv::Mat& camera_matrix, const cv::Mat& camera_pose_orig,
	const cv::Mat& pointmat, const cv::Vec3b& color);