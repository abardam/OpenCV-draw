#include "cv_pointmat_common.h"

cv::Mat filter_pointmat(const cv::Mat& in, std::function<bool(const cv::Vec4f&, void*)> criteria, void* data){
	cv::Mat in_t = in.t();
	cv::Mat in_r = in_t.reshape(4, 1);
	std::vector<cv::Vec4f> in_filter_v;
	for (int i = 0; i < in_r.cols; ++i){
		if (criteria(in_r.ptr<cv::Vec4f>()[i], data)){
			in_filter_v.push_back(in_r.ptr<cv::Vec4f>()[i]);
		}
	}

	if (in_filter_v.size() == 0) return cv::Mat();

	cv::Mat in_filter_r(in_filter_v.size(), 1, CV_32FC4, in_filter_v.data());
	cv::Mat in_filter_t = in_filter_r.reshape(1, in_filter_v.size());
	return in_filter_t.t();
}


void divide_pointmat_by_z(cv::Mat& projected){
	cv::divide(projected(cv::Range(0, 1), cv::Range(0, projected.cols)),
		projected(cv::Range(2, 3), cv::Range(0, projected.cols)),
		projected(cv::Range(0, 1), cv::Range(0, projected.cols)));

	cv::divide(projected(cv::Range(1, 2), cv::Range(0, projected.cols)),
		projected(cv::Range(2, 3), cv::Range(0, projected.cols)),
		projected(cv::Range(1, 2), cv::Range(0, projected.cols)));

	cv::divide(projected(cv::Range(2, 3), cv::Range(0, projected.cols)),
		projected(cv::Range(2, 3), cv::Range(0, projected.cols)),
		projected(cv::Range(2, 3), cv::Range(0, projected.cols)));
}

cv::Mat pointvec_to_pointmat(std::vector<cv::Vec4f>& pointvec){

	cv::Mat pointmat_r(1, pointvec.size(), CV_32FC4, pointvec.data());
	cv::Mat pointmat_t = pointmat_r.reshape(1, pointvec.size());
	cv::Mat pointmat = pointmat_t.t();

	return pointmat;
}