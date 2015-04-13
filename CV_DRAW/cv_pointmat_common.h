#pragma once
#include <opencv2\opencv.hpp>
#include <functional>

cv::Mat filter_pointmat(const cv::Mat& in, std::function<bool(const cv::Vec4f&, void*)> criteria, void* data);
void divide_pointmat_by_z(cv::Mat& pointmat);

cv::Mat pointvec_to_pointmat(std::vector<cv::Vec4f>& pointvec);