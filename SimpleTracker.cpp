//
// Created by HWliaohh on 2/28/18.
//

#include "SimpleTracker.h"
#include"ColorList.h"
#include<memory>
#include<vector>
#include<algorithm>
#include<unordered_set>
#include<direct.h>
#include<io.h>
#include<sstream>//stringstream
#include<iomanip>//setprecision

SimpleTracker::SimpleTracker(int _allow_miss_frame, float _crop_scale)
{
	allowed_miss_frame = _allow_miss_frame;
	crop_scale = _crop_scale;
	index_length = 0;
	epoch = 0;
}

float SimpleTracker::overlap(bbox_t bbox1, bbox_t bbox2)
{
	float left = std::max(bbox1.x, bbox2.x);
	float right = std::min(bbox1.x + bbox1.w , bbox2.x + bbox2.w);
	float top = std::max(bbox1.y , bbox2.y);
	float down = std::min(bbox1.y + bbox1.h, bbox2.y + bbox2.h);
	if (left >= right || top >= down)
	{
		return 0;
	}
	else
	{
		float inter_area = (right - left) * (down - top);
		float union_area = bbox1.w * bbox1.h + bbox2.w * bbox2.h - inter_area;
		return inter_area / union_area;
	}
}

//void SimpleTracker::merge_tracking_and_detection_results(const std::vector<BBox>& detection_res) 
void SimpleTracker::merge_tracking_and_detection_results(std::vector<bbox_t>& detection_res, cv::Mat& cur_frame)
{
	//std::cout << "channels: " << cur_frame.channels() << std::endl;
	std::vector<int> unmatch_idx, left_idx, available_idx;
	std::unordered_set<int> match_idx;
	std::vector<bool> match_flag(tracked_objects.size(), false);
	int inter_size = 0;
	int trackSize = tracked_objects.size();
	int rows = cur_frame.rows;
	int cols = cur_frame.cols;
	for (int j = 0; j < detection_res.size(); ++j)
	{
		
		/*
		int _x1 = std::max(int(detection_res[j].x), 0);
		int _y1 = std::max(int(detection_res[j].y),0);
		int _x2 = std::min(int(detection_res[j].x + detection_res[j].w - 1), cur_frame.cols - 1);
		int _y2 = std::min(int(detection_res[j].y + detection_res[j].h - 1), cur_frame.rows - 1);
		//detection_res[j].face_cropped.clear();
		cur_frame(cv::Rect(_x1, _y1, _x2 - _x1 + 1, _y2 - _y1 + 1)
		).copyTo(detection_res[j].face_cropped);
		
		*/
		crop_a_face(detection_res[j], cur_frame);
		detection_res[j].blur_score = blurdetect(detection_res[j].face_cropped);
		
		float max_iou = 0;
		int best_idx = -1;
		for (int k = 0; k < tracked_objects.size(); ++k)
		{
			//if (match_flag[k] || detection_res[j].y > tracked_objects[k].y + tracked_objects[k].h) continue;
			//if (detection_res[j].y + detection_res[j].h < tracked_objects[k]) break;
			float iou = overlap(detection_res[j], tracked_objects[k]);
			if (iou > max_iou) max_iou = iou, best_idx = k;
		}
		if (max_iou > 0.3)
		{
			detection_res[j].index = tracked_objects[best_idx].index;
			miss_frame[tracked_objects[best_idx].index] = 0;

			vehicles_center_path[tracked_objects[best_idx].index].push_back(detection_res[j]);
			match_idx.insert(tracked_objects[best_idx].index);
			match_flag[best_idx] = true;
			++inter_size;
		}
		else
		{
			unmatch_idx.push_back(j);
		}
	}

	index_length = std::max(int(detection_res.size() + tracked_objects.size() - inter_size + 1), index_length);
	for (int j = 0; j < index_length; ++j)
	{
		if (match_idx.find(j) != match_idx.end()) continue;
		left_idx.push_back(j);
	}
	flow_count_core(left_idx, available_idx);

	for (int j = 0; j < unmatch_idx.size(); j++)
	{
		detection_res[unmatch_idx[j]].index = available_idx[j];
		std::vector<bbox_t> tmp{ detection_res[unmatch_idx[j]] };
		vehicles_center_path[available_idx[j]] = tmp;
		miss_frame[available_idx[j]] = 0;
	}
	clear_data();
	trackSize = tracked_objects.size();
	for (int i = 0; i < tracked_objects.size(); ++i)
	{
		//printf("tracked_object %f %f %f %f %d\n", tracked_objects[i].x, tracked_objects[i].y, tracked_objects[i].w, tracked_objects[i].h, tracked_objects[i].index);
		
	}

	//printf("addr for test is %p", &(tracked_objects[0].x));
	int a = 1;
}

void SimpleTracker::clear_data()
{
	tracked_objects.clear();
	for (auto &x : vehicles_center_path)
	{
		int lp = x.second.size() - 1;
		tracked_objects.push_back(x.second[lp]);
	}
	//sort(tracked_objects.begin(), tracked_objects.end(), cmp);
}
/*
bool SimpleTracker::cmp(const bbox_t &bbox1, const bbox_t& bbox2)
{
return bbox1.y < bbox2.y;
}
*/

void SimpleTracker::flow_count_core(std::vector<int> &left_idx, std::vector<int> &available_idx)
{
	for (int j = 0; j < left_idx.size(); j++)
	{
		if (vehicles_center_path.find(left_idx[j]) != vehicles_center_path.end() && miss_frame[left_idx[j]] >= allowed_miss_frame)
		{
			//std::vector<bbox_t> tmp = vehicles_center_path[left_idx[j]];
			//int lt = tmp.size();
			//if(lt>=5 && (tmp[lt-1].y))

			vehicles_center_path[left_idx[j]].clear();
			vehicles_center_path.erase(left_idx[j]);
			miss_frame.erase(left_idx[j]);
			available_idx.push_back(left_idx[j]);
		}
		else if (vehicles_center_path.find(left_idx[j]) != vehicles_center_path.end() && miss_frame[left_idx[j]]<allowed_miss_frame)
		{
			miss_frame[left_idx[j]] += 1;
		}
		else
		{
			available_idx.push_back(left_idx[j]);
		}
	}
}

std::vector<bbox_t> SimpleTracker::getTrackerObjects()
{
	//std::cout << "in getTrackerObjects()" << std::endl;
	for (int i = 0; i < tracked_objects.size(); ++i)
	{
		//printf("tracked_object %f %f %f %f %d\n", tracked_objects[i].x, tracked_objects[i].y, tracked_objects[i].w, tracked_objects[i].h, tracked_objects[i].index);
	}
	return tracked_objects;
}



float SimpleTracker::blurdetect(cv::Mat img)
{
	cv::cvtColor(img, img, CV_BGR2GRAY);
	cv::Mat laplacianImage;
	cv::Laplacian(img, laplacianImage, CV_8U); // CV_8U

	cv::Mat laplacianImage8bit;
	laplacianImage.convertTo(laplacianImage8bit, CV_8UC1);
	laplacianImage.release();

	cv::Mat finalImage;
	cv::cvtColor(laplacianImage8bit, finalImage, CV_GRAY2BGRA);
	laplacianImage8bit.release();

	/*imshow("tmp", finalImage);
	cv::waitKey();*/

	int rows = finalImage.rows;
	int cols = finalImage.cols;
	char *pixels = reinterpret_cast<char *>(finalImage.data);
	int maxLap = -16777216;
	int sumLap = 0;
	for (int i = 0; i < (rows*cols); i++)
	{
		if (pixels[i] > maxLap)
			maxLap = pixels[i];
		sumLap = sumLap + pixels[i];
	}
	float score = 1.0 * sumLap / (rows*cols);

	return score;
}

cv::Mat SimpleTracker::get_best_face_of_a_person(int index, int period)
{
	assert(vehicles_center_path.find(index) != vehicles_center_path.end());

	int sz = vehicles_center_path[index].size();
	
	float max_score = - 2000;
	float best_index = -1;
	
	for (int i = 0; i < period; ++i)
	{
		if (sz - 1 - i < 0)
		{
			return vehicles_center_path[index][best_index].face_cropped;
		}
		float score_tmp = vehicles_center_path[index][sz - 1 - i].blur_score;
		if (max_score < score_tmp)
		{
			max_score = score_tmp;
			best_index = sz -1 - i;
		}
		
	}
	
	return vehicles_center_path[index][best_index].face_cropped;
	
}

void SimpleTracker::crop_a_face(bbox_t& _bbox_t, cv::Mat& cur_frame)
{
	int _x1 = _bbox_t.x - (crop_scale-1)*0.5*_bbox_t.w;
	int _y1 = _bbox_t.y - (crop_scale-1)*0.5*_bbox_t.h;
	int _w = _bbox_t.w*crop_scale;
	int _h = _bbox_t.h*crop_scale;

	_x1 = std::max(_x1, 0);
	_y1 = std::max(_y1, 0);
	int _x2 = std::min(_x1+_w, cur_frame.cols - 1);
	int _y2 = std::min(_y1+_h, cur_frame.rows - 1);
	
	cur_frame(cv::Rect(_x1, _y1, _x2 - _x1 + 1, _y2 - _y1 + 1)
	).copyTo(_bbox_t.face_cropped);

	//std::cout << "in crop a face" << std::endl;
	//std::cout << cur_frame.channels() << std::endl;
	//std::cout << _bbox_t.face_cropped.channels() << std::endl;
}
std::vector<int> SimpleTracker::get_object_indexs()
{
	std::vector<int> indexs;
	for (std::unordered_map<int, std::vector<bbox_t>>::iterator it = vehicles_center_path.begin(); it != vehicles_center_path.end(); ++it)
		indexs.push_back(it->first);
	return indexs;
}
void SimpleTracker::quality_cmp() 
{
	printf("epoch %d --------------------- \n", epoch);
	for (std::unordered_map<int, std::vector<bbox_t> >::iterator it = vehicles_center_path.begin(); it != vehicles_center_path.end(); ++it)
	{
		printf("score candidate of index %d: \n", it->first);
		
		std::string dir = "D:\\data\\quality_cmp\\" + std::to_string(it->first);
		int flag = _mkdir(dir.c_str());

		
		
		float max_score = -2000;
		int best_idx = -1;
		
		int sz = (it->second).size();
		for (int i = 0; i < 30; i++)
		{
			if (sz - 1 - i < 0)
				break;
			
			float score_tmp = (it->second)[sz - 1 - i].blur_score;

			if ( max_score < score_tmp ) {
				
				max_score =score_tmp;
				best_idx = sz - 1 - i;
			}

			std::stringstream  ss;
			ss << std::fixed << std::setprecision(1) << score_tmp;
			cv::imwrite(dir + "\\" + "epoch_" + std::to_string(epoch) + "_" + std::to_string(sz - i - 1) + "_" + ss.str() + ".jpg", (it->second)[sz - 1 - i].face_cropped);
			
			printf("%.1f  ", score_tmp);
			std::cout << "*** channels: " << (it->second)[sz - 1 - i].face_cropped.channels() << std::endl;
		}

		printf("\n");

		printf("max_score: %.1f\n", max_score);

		std::stringstream  ss;
		ss << std::fixed << std::setprecision(1) << max_score;

		cv::imwrite(dir + "\\" + "epoch_" + std::to_string(epoch) + "_" + "best_" + ss.str()+".jpg", (it->second)[best_idx].face_cropped);

	}
	epoch += 1;

}

