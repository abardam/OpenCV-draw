#pragma once
#include <opencv2\opencv.hpp>

#define CLAMP(x, y, w, h)(0<=x&&x<w&&0<=y&&y<h)

struct CroppedMat{
	cv::Mat mMat;
	cv::Point2i mOffset;
	cv::Size2i mSize;

	CroppedMat() :
		mOffset(0, 0),
		mSize(0, 0){}
};

void draw_linesegmat_on_image(cv::Mat& image, const cv::Mat& linesegmat, const cv::Vec3b& color);

//convert to and from cropped mats
CroppedMat crop_mat(const cv::Mat& mat, const cv::Vec3b crop_color);
CroppedMat crop_mat(const cv::Mat& mat, const std::vector<cv::Vec3b>& crop_colors);
cv::Mat uncrop_mat(const CroppedMat& cropped_mat, const cv::Vec3b& color);

//serialization for cropped cv mat
void write(cv::FileStorage& fs, const std::string&, const CroppedMat& n);
void read(const cv::FileNode& node, CroppedMat& n, const CroppedMat& default_value = CroppedMat());


//convenience function
cv::Mat create_translation_mat(cv::Vec3f translation);

cv::Mat visualize_float(const cv::Mat& float_mat);