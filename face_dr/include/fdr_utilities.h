#ifndef FDR_UTILITIES_H
#define FDR_UTILITIES_H

#include <vector>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include "ros/ros.h"

#include "boost/filesystem.hpp"


std::vector<cv::Rect> detect_faces_cascade(const cv::Mat&, cv::CascadeClassifier);

void calc_hist(const cv::Mat&, cv::Mat&);

void read_ub_yml(std::string, std::string, cv::Mat&, cv::Mat&, std::vector<std::string>&);

void save_image(const cv::Mat&, std::string, std::string, int);

void append_to_csv(std::string, std::string, std::string, int);

int count_files(std::string);

void read_csv(const std::string&, std::vector<cv::Mat>&, std::vector<int>&, std::vector<std::string>&, char);

void write_vector(std::string, std::vector<std::string>);

void read_vector(std::string, std::vector<std::string>&);

cv::Mat extract_dress_color(const cv::Rect&, const cv::Mat&);

void append_to_yaml(std::string, std::string, cv::Mat, int);

void write_yaml(std::string, std::string, std::string, std::string);


template<typename T>
void read_yaml(std::string folder, std::string file, std::string node, T& value)
{
	using namespace boost::filesystem;
	path file_path(folder + file);
	if (!(exists(file_path) && is_regular_file(file_path)))
	   ROS_INFO("ERROR reading YAML file: %s", file.c_str());
	else
	{
	   cv::FileStorage fs(folder + file, cv::FileStorage::READ);
	   fs[node] >> value;
	   fs.release();
	}
}

#endif
