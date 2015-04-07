#include "cv_draw_cylinder.h"
#include "cv_draw_common.h"
#include "cv_pointmat_common.h"

template <typename T, int S>
struct Segment{
	cv::Vec<T, S> first;
	cv::Vec<T, S> second;

	Segment(){}
	Segment(cv::Vec<T, S> f, cv::Vec<T, S> s) :first(f), second(s){}

	bool operator==(Segment<T, S> s){
		return first == s.first && second == s.second;
	};
};

typedef Segment<float, 3> Segment3f;
typedef Segment<float, 2> Segment2f;


template <typename T, int S>
std::pair<Segment<T, S>, Segment<T, S>> subdivide_segment(Segment<T, S> s){
	cv::Vec<T, S> mid;
	for (int i = 0; i<S; ++i){
		mid[i] = (s.first[i] + s.second[i]) / 2;
	}
	Segment<T, S> a;
	Segment<T, S> b;
	a.first = s.first;
	a.second = mid;
	b.first = mid;
	b.second = s.second;

	return std::pair<Segment<T, S>, Segment<T, S>>(a, b);
};

//makes polygons more "circle-y"
void subdivide_polygon(std::vector<Segment3f>& polygon, const cv::Vec3f& center, float radius){

	int j = 0;
	while (j<polygon.size()){
		Segment3f segj = polygon[j];
		polygon.erase(polygon.begin() + j);
		auto seg2 = subdivide_segment(segj);
		seg2.first.first = center + radius * cv::normalize(seg2.first.first - center);
		seg2.second.first = center + radius * cv::normalize(seg2.second.first - center);
		seg2.first.second = center + radius * cv::normalize(seg2.first.second - center);
		seg2.second.second = center + radius * cv::normalize(seg2.second.second - center);
		polygon.insert(polygon.begin() + j, seg2.second);
		polygon.insert(polygon.begin() + j, seg2.first);
		j += 2;
	}
}

std::vector<Segment3f> cylinder_to_segments(float ellipse_x, float ellipse_z, float height, int numsegments){
	std::vector<Segment3f> ret;

	cv::Vec3f cylaxis(0, height, 0);
	cv::Vec3f cross_x(ellipse_x / 2, 0, 0);
	cv::Vec3f cross_z(0, 0, ellipse_z / 2);

	//draw the rect:
	cv::Vec3f a1 = ellipse_x * cross_x;
	cv::Vec3f a2 = -ellipse_x * cross_x;
	cv::Vec3f b1 = cylaxis + ellipse_x * cross_x;
	cv::Vec3f b2 = cylaxis - ellipse_x * cross_x;

	cv::Vec3f center_a(0, 0, 0);
	cv::Vec3f center_b = cylaxis;

	ret.push_back(Segment3f(a2, b2));
	ret.push_back(Segment3f(b1, a1));

	int ndiv = std::log(numsegments + 0.0) / CV_LOG2;

	//caps

	std::vector<Segment3f> ret_a;
	std::vector<Segment3f> ret_b;

	cv::Vec3f a3 = ellipse_z * cross_z;
	cv::Vec3f a4 = -ellipse_z * cross_z;
	cv::Vec3f b3 = cylaxis + ellipse_z * cross_z;
	cv::Vec3f b4 = cylaxis - ellipse_z * cross_z;

	ret_a.push_back(Segment3f(a1, a3));
	ret_a.push_back(Segment3f(a2, a3));
	ret_a.push_back(Segment3f(a1, a4));
	ret_a.push_back(Segment3f(a2, a4));
	ret_b.push_back(Segment3f(b1, b3));
	ret_b.push_back(Segment3f(b2, b3));
	ret_b.push_back(Segment3f(b1, b4));
	ret_b.push_back(Segment3f(b2, b4));

	//for (int i = 2; i<ndiv; ++i){
	//	subdivide_polygon(ret_a, center_a, radius);
	//	subdivide_polygon(ret_b, center_b, radius);
	//}

	for (int i = 0; i<ret_a.size(); ++i){
		ret.push_back(ret_a[i]);
		ret.push_back(ret_b[i]);
	}

	return ret;
}


void cv_draw_cylinder(cv::Mat& image, float ellipse_x, float ellipse_z, float height, int numsegments, const cv::Mat& global_pose, const cv::Mat& camera_matrix, const cv::Vec3b& color){
	std::vector<Segment3f> segments = cylinder_to_segments(ellipse_x, ellipse_z, height, numsegments);
	cv::Mat segments_m(4, segments.size() * 2, CV_32F);
	for (int i = 0; i < segments.size(); ++i){
		segments_m.ptr<float>(0)[2 * i] = segments[i].first(0);
		segments_m.ptr<float>(1)[2 * i] = segments[i].first(1);
		segments_m.ptr<float>(2)[2 * i] = segments[i].first(2);
		segments_m.ptr<float>(3)[2 * i] = 1;
		segments_m.ptr<float>(0)[2 * i + 1] = segments[i].second(0);
		segments_m.ptr<float>(1)[2 * i + 1] = segments[i].second(1);
		segments_m.ptr<float>(2)[2 * i + 1] = segments[i].second(2);
		segments_m.ptr<float>(3)[2 * i + 1] = 1;
	}

	cv::Mat projected = camera_matrix * global_pose * segments_m;
	divide_pointmat_by_z(projected);

	draw_linesegmat_on_image(image, projected, color);
}
