#pragma once
//
// Created by root on 3/2/18.
//

#ifndef SINGLETRACKER_SINGLETRACKER_H
#define SINGLETRACKER_SINGLETRACKER_H

#include <iostream>
#include <vector>
#include<unordered_map>
//#include <opencv2/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>

typedef struct ST{
	float x, y, w, h;
	int index;
	float blur_score;
	cv::Mat face_cropped;
} bbox_t;

class SimpleTracker {
private:
	int max_id;
	std::unordered_map<int, int> miss_frame;
	int allowed_miss_frame;
	float crop_scale;
	int index_length;
	std::vector<bbox_t> tracked_objects;
	int epoch;
private:
	void crop_a_face(bbox_t&, cv::Mat&);

public:

	SimpleTracker(int _allow_miss_frame = 15, float _crop_scale = 1.0);
	void flow_count_core(std::vector<int> &left_idx, std::vector<int> &available_idx);
	void merge_tracking_and_detection_results(std::vector<bbox_t>& detection_res, cv::Mat& cur_frame);
	float overlap(bbox_t bbox1, bbox_t bbox2);
	std::vector<bbox_t> getTrackerObjects();
	std::unordered_map<int, std::vector<bbox_t> > vehicles_center_path;
	float blurdetect(cv::Mat img);
	void clear_data();
	void quality_cmp();
	cv::Mat get_best_face_of_a_person(int index, int period);
	std::vector<int> get_object_indexs();
};


#endif //SINGLETRACKER_SINGLETRACKER_H
#pragma once
