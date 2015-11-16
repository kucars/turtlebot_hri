#ifndef FDR_UTILITIES_H
#define FDR_UTILITIES_H

#include <vector>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include "ros/ros.h"

#include "boost/filesystem.hpp"

const int min_train_data = 10;

void calc_hist(const cv::Mat&, cv::Mat&);

void read_ub_yml(std::string, std::string, cv::Mat&, cv::Mat&, std::vector<std::string>&);

void save_image(const cv::Mat&, std::string, std::string, int);

void append_to_csv(std::string, std::string, std::string, int);

int count_files(std::string);

void read_csv(const std::string&, std::vector<cv::Mat>&, std::vector<int>&, std::vector<std::string>&, char);

void write_vector(std::string, std::vector<std::string>);

void read_vector(std::string, std::vector<std::string>&);

cv::Rect extract_dress_area(const cv::Rect&);

void append_to_yaml(std::string, std::string, cv::Mat, int);


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

template<typename T>
void write_yaml(std::string folder, std::string file, std::string node, const T& value)
{
	using namespace boost::filesystem;
	path folder_path(folder);
	if (!(exists(folder_path) && is_directory(folder_path)))
		create_directories(folder);
	cv::FileStorage fs(folder + file, cv::FileStorage::WRITE);
	fs << node << value;
	fs.release();
}


template<typename T1, typename T2>
void read_yaml_2(std::string folder, std::string file, std::string node_1, T1& value_1, std::string node_2, T2& value_2)
{
	using namespace boost::filesystem;
	path file_path(folder + file);
	if (!(exists(file_path) && is_regular_file(file_path)))
		ROS_INFO("ERROR reading YAML file: %s", file.c_str());
	else
	{
		cv::FileStorage fs(folder + file, cv::FileStorage::READ);
		fs[node_1] >> value_1;
		fs[node_2] >> value_2;
		fs.release();
	}
}

template<typename T1, typename T2>
void write_yaml_2(std::string folder, std::string file, std::string node_1, const T1& value_1, std::string node_2, const T2& value_2)
{
	using namespace boost::filesystem;
	path folder_path(folder);
	if (!(exists(folder_path) && is_directory(folder_path)))
		create_directories(folder);
	cv::FileStorage fs(folder + file, cv::FileStorage::WRITE);
	fs << node_1 << value_1;
	fs << node_2 << value_2;
	fs.release();
}



void save_avg_hists(const cv::Mat&, const cv::Mat&, std::vector<std::string>, std::string);

std::string classify_hue(const cv::Mat& hue_hist, std::string classifier_file);


#endif
