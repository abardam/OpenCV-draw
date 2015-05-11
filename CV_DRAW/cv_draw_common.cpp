#include "cv_draw_common.h"



void draw_linesegmat_on_image(cv::Mat& image, const cv::Mat& linesegmat, const cv::Vec3b& color){
	for (int i = 0; i < linesegmat.cols; i+=2){
		int x1 = linesegmat.ptr<float>(0)[i];
		int y1 = linesegmat.ptr<float>(1)[i];
		int x2 = linesegmat.ptr<float>(0)[i+1];
		int y2 = linesegmat.ptr<float>(1)[i+1];

		cv::Scalar color_s(color(0), color(1), color(2));

		cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), color_s, 1);
	}
}

CroppedMat crop_mat(const cv::Mat& mat, const cv::Vec3b crop_color){
	unsigned int x = mat.cols,
		y = mat.rows,
		x2 = 0,
		y2 = 0;

	for (unsigned int _x = 0; _x < mat.cols; ++_x){
		for (unsigned int _y = 0; _y < mat.rows; ++_y){
			if (mat.ptr<cv::Vec3b>(_y)[_x] != crop_color){
				if (x > _x){
					x = _x;
				}
				if (y > _y){
					y = _y;
				}
				if (x2 < _x){
					x2 = _x;
				}
				if (y2 < _y){
					y2 = _y;
				}
			}
		}
	}

	CroppedMat out;

	if (x2 < x || y2 < y){
		//BAD END
	}
	else{
		out.mMat = mat(cv::Range(y, y2+1), cv::Range(x, x2+1)).clone();
		out.mOffset = cv::Point2i(x, y);
		out.mSize = mat.size();
	}
	
	return out;
}

CroppedMat crop_mat(const cv::Mat& mat, const std::vector<cv::Vec3b>& crop_colors){
	unsigned int x = mat.cols,
		y = mat.rows,
		x2 = 0,
		y2 = 0;

	for (unsigned int _x = 0; _x < mat.cols; ++_x){
		for (unsigned int _y = 0; _y < mat.rows; ++_y){

			bool is_crop_color = false;

			for (int i = 0; i < crop_colors.size(); ++i){
				if (mat.ptr<cv::Vec3b>(_y)[_x] == crop_colors[i]){
					is_crop_color = true;
					break;
				}
			}

			if (!is_crop_color){
				if (x > _x){
					x = _x;
				}
				if (y > _y){
					y = _y;
				}
				if (x2 < _x){
					x2 = _x;
				}
				if (y2 < _y){
					y2 = _y;
				}
			}
		}
	}

	CroppedMat out;

	if (x2 < x || y2 < y){
		//BAD END
	}
	else{
		out.mMat = mat(cv::Range(y, y2 + 1), cv::Range(x, x2 + 1)).clone();
		out.mOffset = cv::Point2i(x, y);
		out.mSize = mat.size();
	}

	return out;
}

cv::Mat uncrop_mat(const CroppedMat& cropped_mat, const cv::Vec3b& color){
	cv::Mat out(cropped_mat.mSize.height, cropped_mat.mSize.width, CV_8UC3, cv::Scalar(color(0), color(1), color(2)));

	if (out.cols == 0 || out.rows == 0) return out;

	unsigned int x, y, x2, y2;
	x = cropped_mat.mOffset.x;
	y = cropped_mat.mOffset.y;
	x2 = x + cropped_mat.mMat.cols;
	y2 = y + cropped_mat.mMat.rows;

	cropped_mat.mMat.copyTo(out(cv::Range(y, y2), cv::Range(x, x2)));

	return out;
	
}


void write(cv::FileStorage& fs, const std::string&, const CroppedMat& n){
	fs << "{" << "mat" << n.mMat
		<< "offset" << n.mOffset
		<< "size" << n.mSize
		<< "}";
}
void read(const cv::FileNode& node, CroppedMat& n, const CroppedMat& default_value){
	if (node.empty()){
		n = default_value;
	}
	else{
		node["mat"] >> n.mMat;
		node["offset"] >> n.mOffset;
		node["size"] >> n.mSize;
	}
}

cv::Mat create_translation_mat(cv::Vec3f translation){
	cv::Mat ret = cv::Mat::eye(4, 4, CV_32F);
	ret.ptr<float>(0)[3] = translation(0);
	ret.ptr<float>(1)[3] = translation(1);
	ret.ptr<float>(2)[3] = translation(2);
	return ret;
}

cv::Mat visualize_float(const cv::Mat& float_mat){
	cv::Mat norm_mat;
	cv::normalize(float_mat, norm_mat, 0, 255, cv::NORM_MINMAX, CV_8U);
	return norm_mat;
}