#include "cv_pointmat_common.h"
#include "cv_draw_common.h"

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

void draw_pointmat_on_image(cv::Mat& image, const cv::Mat& pointmat, const cv::Vec3b& color, int size){
	for (int i = 0; i < pointmat.cols; ++i){
		int x = pointmat.ptr<float>(0)[i];
		int y = pointmat.ptr<float>(1)[i];
		cv::circle(image, cv::Point(x, y), size, cv::Scalar(color(0), color(1), color(2)), -1);
	}
}

#define ROTATE_VALUE 0.001
#define BIG_ZOOM_VALUE 0.1
#define ZOOM_VALUE 0.01

struct DisplayPointmatState{
	int mouse_x_start = 0, mouse_y_start = 0;
	int mouse_x_curr = 0, mouse_y_curr = 0;
	bool mouse_down = false;

};

static void display_pointmat_mouse_callback(int event, int x, int y, int, void* state){
	DisplayPointmatState * dps = (DisplayPointmatState*)state;

	if (event == cv::EVENT_LBUTTONDOWN){
		dps->mouse_x_start = dps->mouse_x_curr = x;
		dps->mouse_y_start = dps->mouse_y_curr = y;
		dps->mouse_down = true;
	}
	else if (event == cv::EVENT_MOUSEMOVE){
		if (dps->mouse_down){
			dps->mouse_x_curr = x;
			dps->mouse_y_curr = y;
		}
	}
	else if (event == cv::EVENT_LBUTTONUP){
		dps->mouse_down = false;
	}
}

void display_pointmat(const std::string& window_name, int window_width, int window_height, const cv::Mat& camera_matrix, const cv::Mat& camera_pose_orig, 
	const std::vector<cv::Mat>& pointmats, const std::vector<cv::Vec3b>& colors){

	bool draw_lines = true;
	int ptsize = 1;

	cv::namedWindow(window_name);
	cv::Mat camera_pose = camera_pose_orig.clone();

	DisplayPointmatState dps;
	cv::setMouseCallback(window_name, display_pointmat_mouse_callback, &dps);

	cv::Mat img(window_height, window_width, CV_8UC3, cv::Scalar(0,0,0));

	cv::Mat camera_temp_pose = cv::Mat::eye(4, 4, CV_32F);
	while (true){
		if (dps.mouse_down)
		{
			float angle_x = (dps.mouse_x_curr - dps.mouse_x_start) * ROTATE_VALUE;
			float angle_y = (dps.mouse_y_curr - dps.mouse_y_start) * ROTATE_VALUE;

			cv::Rodrigues(cv::Vec3f(angle_y, angle_x, 0), camera_temp_pose(cv::Range(0, 3), cv::Range(0, 3)));
		}
		else{
			camera_pose = camera_temp_pose * camera_pose;
			camera_temp_pose = cv::Mat::eye(4, 4, CV_32F);
		}

		cv::Mat img_clone = img.clone();

		if (draw_lines && pointmats.size() == 2 && pointmats[0].cols == pointmats[1].cols){
			std::vector<cv::Mat> ptmats_cam(pointmats.size());
			for (int i = 0; i < pointmats.size(); ++i){
				cv::Mat ptmat_cam = camera_matrix * camera_temp_pose * camera_pose * pointmats[i];
				divide_pointmat_by_z(ptmat_cam);
				ptmats_cam[i] = (ptmat_cam);
			}
			for (int i = 0; i < pointmats[0].cols; ++i){
				cv::line(img_clone,
					cv::Point(ptmats_cam[0].ptr<float>(0)[i], ptmats_cam[0].ptr<float>(1)[i]),
					cv::Point(ptmats_cam[1].ptr<float>(0)[i], ptmats_cam[1].ptr<float>(1)[i]),
					cv::Scalar(0xff, 0xff, 0xff));
			}
			for (int i = 0; i < pointmats.size(); ++i){
				draw_pointmat_on_image(img_clone, ptmats_cam[i], colors[i], ptsize);
			}
		}
		else{
			for (int i = 0; i < pointmats.size(); ++i){
				cv::Mat ptmat_cam = camera_matrix * camera_temp_pose * camera_pose * pointmats[i];
				divide_pointmat_by_z(ptmat_cam);
				draw_pointmat_on_image(img_clone, ptmat_cam, colors[i], ptsize);
			}
		}

		cv::imshow(window_name, img_clone);
		char q = cv::waitKey(20);

		cv::Mat trans_tmp = cv::Mat::eye(4, 4, CV_32F);

		switch (q){
		case 'W':
			trans_tmp.ptr<float>(2)[3] = -BIG_ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'w':
			trans_tmp.ptr<float>(2)[3] = -ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'S':
			trans_tmp.ptr<float>(2)[3] = BIG_ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 's':
			trans_tmp.ptr<float>(2)[3] = ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'A':
			trans_tmp.ptr<float>(0)[3] = -BIG_ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'a':
			trans_tmp.ptr<float>(0)[3] = -ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'D':
			trans_tmp.ptr<float>(0)[3] = BIG_ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'd':
			trans_tmp.ptr<float>(0)[3] = ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'Q':
			trans_tmp.ptr<float>(1)[3] = BIG_ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'q':
			trans_tmp.ptr<float>(1)[3] = ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'E':
			trans_tmp.ptr<float>(1)[3] = -BIG_ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'e':
			trans_tmp.ptr<float>(1)[3] = -ZOOM_VALUE;
			camera_temp_pose = trans_tmp * camera_temp_pose;
			break;
		case 'C':
		case 'c':
			//continue
			return;
			break;
		case 'R':
		case'r':
			camera_pose = camera_pose_orig.clone();
		case 'L':
		case'l':
			draw_lines = !draw_lines;
		case'[':
			--ptsize;
			if (ptsize < 0)ptsize = 0;
			break;
		case']':
			++ptsize;
			break;
		}
	}
}

void display_pointmat(const std::string& window_name, int window_width, int window_height, const cv::Mat& camera_matrix, const cv::Mat& camera_pose_orig,
	const cv::Mat& pointmat, const cv::Vec3b& color){
	std::vector<cv::Mat> v_m;
	v_m.push_back(pointmat);
	std::vector<cv::Vec3b> v_c;
	v_c.push_back(color);
	display_pointmat(window_name, window_width, window_height, camera_matrix, camera_pose_orig, v_m, v_c);
}