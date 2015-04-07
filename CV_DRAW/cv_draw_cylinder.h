#pragma once
#include <opencv2\opencv.hpp>

void cv_draw_cylinder(cv::Mat& image, float ellipse_x, float ellipse_z, float height, int numsegments, const cv::Mat& global_pose, const cv::Mat& camera_matrix, const cv::Vec3b& color);