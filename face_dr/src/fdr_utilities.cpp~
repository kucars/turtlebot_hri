#include "fdr_utilities.h"

#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"


void calc_hist(const cv::Mat& dc_img, cv::Mat& hue_hist)
{
   cv::Mat hsv_img;
   cvtColor(dc_img, hsv_img, CV_BGR2HSV);
   std::vector<cv::Mat> hsv_planes;
   cv::split( hsv_img, hsv_planes );
   float norm_factor = hsv_img.total();
   int histSize = 256;

   float range[] = { 0, 256 } ;
   const float* histRange = { range };
   cv::calcHist(&hsv_planes[0], 1, 0, cv::Mat(), hue_hist, 1, &histSize, &histRange, true, false);
   for (int k = 0; k < 256; k++)
	   hue_hist.at<float>(k) = 100.0f * hue_hist.at<float>(k) / norm_factor;
}


void read_ub_yml(std::string hists_path, std::string dc_imgs_folder, cv::Mat& trainingData, cv::Mat& responseData, std::vector<std::string>& label_to_ub_name)
{
   int outer_counter = 0;
   boost::filesystem::directory_iterator end_iter;
   for (boost::filesystem::directory_iterator dir_iter(hists_path); dir_iter!=end_iter; ++dir_iter)
   {
      if (boost::filesystem::is_directory(dir_iter->status()))
      {
	 std::string name = dir_iter->path().filename().native();
	 int dir_count = count_files(dc_imgs_folder + name);
	 if (dir_count >= min_train_data)
         {
	         for (int i = 0; i < dir_count; i++)
	         {
       		    cv::Mat tmp(256, 1, CV_32F);
        	    std::string node = "Hue Histogram "+std::to_string(i + 1);
			    read_yaml<cv::Mat>(hists_path + name, "/dc_log.yml", node, tmp);
       		    if (trainingData.empty()) trainingData.push_back(tmp);
        	    else cv::hconcat(trainingData, tmp, trainingData);
        	    responseData.push_back(outer_counter);
        	 }
	         outer_counter++;
         	 label_to_ub_name.push_back(name);
         }
      }
   }
}

void save_image(const cv::Mat& img, std::string folder, std::string name, int dir_count)
{
	using namespace boost::filesystem;
	path dir_path(folder + name);
	if (!exists(dir_path)) create_directories(dir_path);
	if (!is_directory(dir_path)) create_directories(dir_path);
	std::string save_name = folder + name + "/" + name
		+ "_" + std::to_string(dir_count) + ".jpg";
	cv::imwrite(save_name, img);
}

void append_to_csv(std::string csv_file, std::string imgs_folder, std::string name, int count)
{
	std::ofstream ost(csv_file.c_str(), std::ios_base::app);
	ost << imgs_folder << name << "/" << name;
	ost << "_" << count << ".jpg;" << name;
	ost << std::endl;
	ost.close();
}

int count_files(std::string pathname)
{
	using namespace boost::filesystem;
	path dir_path(pathname);
	if (!(exists(dir_path) && is_directory(dir_path)))
	   return 0;
	else
	   return std::distance(directory_iterator(dir_path), directory_iterator());
}

void read_csv(const std::string& filename, std::vector<cv::Mat>& face_images_vector, std::vector<int>& class_labels, std::vector<std::string>& label_to_name, char separator)
{
	int current_label = -1;
	std::string prev_name = "!!isAnImpossibleName!!";
	std::ifstream file(filename.c_str());
	std::string line, path, person_name;
	while (getline(file, line))
	{
	   std::stringstream liness(line);
	   getline(liness, path, separator);
	   getline(liness, person_name);
	   if (!path.empty() && !person_name.empty())
	   {
	      face_images_vector.push_back(cv::imread(path, -1));
	      if (person_name != prev_name)
	      {
	         current_label++;
	         prev_name = person_name;
	         label_to_name.push_back(person_name);
	      }
	      class_labels.push_back(current_label);
	   }
	}
}

void write_vector(std::string pathname, std::vector<std::string> vec)
{
	using namespace std;
	ofstream ost(pathname.c_str(), ios_base::out);
	for (int i = 0; i < vec.size(); i++)
	   ost << vec[i] << endl;
	ost.close();
}

void read_vector(std::string pathname, std::vector<std::string>& vec)
{
	using namespace std;
	ifstream ist(pathname.c_str(), ios_base::in);
	string tmp;
	while (ist >> tmp)
	   vec.push_back(tmp);
	ist.close();
}


cv::Rect extract_dress_area(const cv::Rect& upper_body_rect)
{
	double dcx = upper_body_rect.x + upper_body_rect.width*0.3;
	double dcy = upper_body_rect.y + upper_body_rect.height*0.7;
	double dcwidth = upper_body_rect.width*0.4;
	double dcheight = upper_body_rect.height*0.3;
	cv::Rect dc_rect(dcx, dcy, dcwidth, dcheight);

	return dc_rect;
}


void append_to_yaml(std::string folder, std::string name, cv::Mat hist, int count)
{
	using namespace boost::filesystem;
	path dir_path(folder + name);
	
	if (!(exists(dir_path) && is_directory(dir_path)))
	   create_directories(dir_path);
	path pathname(folder + name + "/dc_log.yml");
	cv::FileStorage fs;
	if (!exists(pathname) || count == 0)
	   fs = cv::FileStorage(pathname.native(), cv::FileStorage::WRITE);
	else
	   fs = cv::FileStorage(pathname.native(), cv::FileStorage::APPEND);
	std::time_t rawtime; std::time(&rawtime);
	fs << "Time " + std::to_string(count) << std::asctime(std::localtime(&rawtime));
	fs << "Hue Histogram " + std::to_string(count) << hist;
	fs.release();
}


void save_avg_hists(const cv::Mat& trainingData, const cv::Mat& responseData, std::vector<std::string> label_to_name, std::string save_file)
{
	bool end_of_class = true;
	int steps = 0;
	int i = 0;
	cv::Mat respData;
	responseData.convertTo(respData, CV_32F);
	respData.push_back(-9999.00f);
	cv::Mat trainData;
	trainingData.convertTo(trainData, CV_32F);
	cv::Mat sums(cv::Size(label_to_name.size(), trainingData.size().height), CV_32F, 0.0f);
	cv::Mat averages(cv::Size(label_to_name.size(), trainingData.size().height), CV_32F, 0.0f);
	for (int j = 0; j < label_to_name.size(); j++)
	{
		steps = i;
		i = 0;
		do
		{
			for (int k = 0; k < 256; k++)
				sums.at<float>(k, j) = trainData.at<float>(k, i+steps) + sums.at<float>(k, j);

			int tmp = responseData.at<int>(i+steps);
			i++;
			if (tmp == responseData.at<int>(i+steps)) end_of_class = false;
			else end_of_class = true;
		} while (!end_of_class);
		for (int k = 0; k < 256; k++)
			averages.at<float>(k, j) = sums.at<float>(k, j) * (1.0f/i);

	}
	cv::FileStorage fs(save_file, cv::FileStorage::WRITE);
	fs << "average_hue_hists" << averages;
	fs << "class_names" << label_to_name;
	fs.release();
}

std::string classify_hue(const cv::Mat& hue_hist, std::string classifier_file)
{
	cv::Mat classifier;
	std::vector<std::string> label_to_name;
	cv::FileStorage fs(classifier_file, cv::FileStorage::READ);
	fs["average_hue_hists"] >> classifier;

	fs["class_names"] >> label_to_name;
	fs.release();
	int min_label = -1;
	float min_out = 1000000;
	std::vector<float> sum_sq_dist(label_to_name.size(), 0.0);
	for (int i = 0; i < label_to_name.size(); i++)
	{
		for (int k = 0; k < 256; k++)
		{

			float tmp = hue_hist.at<float>(k) - classifier.at<float>(k, i);
			sum_sq_dist[i] = sum_sq_dist[i] + tmp*tmp;

		}
		if (sum_sq_dist[i] < min_out)
		{
			min_out = sum_sq_dist[i];
			min_label = i;
		}
	}
	ROS_INFO("min out %f", min_out);
	if (min_out < 30) return label_to_name[min_label];
	else return "unknown";

}
