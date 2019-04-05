#include<opencv2/core/utility.hpp>
#include<opencv2/videoio/videoio.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/tracking/tracker.hpp>
#include<iostream>
#include<cstring>
#include<facedetect-dll.h>
#include"SimpleTracker.h"
#include"ColorList.h"
#define DETECT_BUFFER_SIZE 0x20000

int main(int argc, char** argv) {
	/*
	cv::Rect2d rect1=cv::Rect2d(1, 1, 3, 3);
	cv::Rect2d rect2=cv::Rect2d(3, 3, 6, 6);
	SimpleTracker tracker;
	std::cout << "iou" << tracker.IOU(rect1, rect2);
	return 0;
	*/
	
	if (argc < 2) {
		std::cout << "need input video file" << std::endl;
		return 0;
	}
	
	cv::VideoCapture cap(argv[1]);
	if (!cap.isOpened()) {
		std::cout << "can not read video file " << argv[1] << std::endl;
	}
	int *pResults = NULL;
	unsigned char* pBuffer = (unsigned char*)std::malloc(DETECT_BUFFER_SIZE);
	if (!pBuffer) {
		fprintf(stderr, "Can not alloc buffer\n");
	}
	int doLandmark = 0;
	int frame_cnt = 0;
	SimpleTracker tracker(15,1.5);
	//std::vector<BBox> face_bbox;
	std::vector<bbox_t> face_bbox;
	std::vector<cv::Scalar> colorlist;
	colorlist.push_back(cv::Scalar(255, 0, 0));
	colorlist.push_back(cv::Scalar(0, 255, 0));
	colorlist.push_back(cv::Scalar(0, 0, 255));
	colorlist.push_back(cv::Scalar(255, 255, 255));
	colorlist.push_back(cv::Scalar(0, 0, 0));
	cv::namedWindow("face");
	while (true) {
		if (frame_cnt == 241)
		{
			printf("hello");
		}
		cv::Mat frame, gray;
		cap.read(frame);
		if (frame.empty()) {
			return 0;
		}
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		cv::Mat result_frontal = frame.clone();
		pResults= facedetect_multiview(pBuffer, (unsigned char*)(gray.ptr(0)), gray.cols, gray.rows, (int)gray.step,
			1.2f, 2, 48, 250, doLandmark);

		for (int i = 0; i < (pResults ? *pResults : 0); i++)
		{
			short * p = ((short*)(pResults + 1)) + 142 * i;
			bbox_t roi;
			roi.x = p[0];
			roi.y = p[1];
			roi.w = p[2];
			roi.h = p[3];
			int neighbors = p[4];
			int angle = p[5];

			//printf("face_rect=[%d, %d, %d, %d], neighbors=%d, angle=%d\n", p[0], p[1], p[2], p[3], neighbors, angle);
			face_bbox.push_back(roi);
		}
		//std::cout << "------------" << std::endl;
		//std::cout << "face detected: " << face_bbox.size() << std::endl;
		std::vector<bbox_t> track_res;

		tracker.merge_tracking_and_detection_results(face_bbox,frame);

		track_res = tracker.getTrackerObjects();
		
	    //std::cout << "tracking objects " << track_res.size() << std::endl;
	
		for (int i = 0; i < track_res.size(); ++i)
		{
			//rectangle(result_frontal, cv::Rect(track_res[i].x,track_res[i].y,track_res[i].w,track_res[i].h), cv::Scalar(0, 255, 0), 2);
			rectangle(result_frontal, cv::Rect(track_res[i].x, track_res[i].y, track_res[i].w, track_res[i].h),colorlist[track_res[i].index%colorlist.size()], 4);
			/*
			CV_EXPORTS_W void putText(InputOutputArray img, const String& text, Point org,
				int fontFace, double fontScale, Scalar color,
				int thickness = 1, int lineType = LINE_8,
				bool bottomLeftOrigin = false);
				*/
			cv::putText(result_frontal, std::to_string(track_res[i].index), cv::Point(track_res[i].x, track_res[i].y), cv::FONT_HERSHEY_PLAIN, 2, colorlist[track_res[i].index%colorlist.size()],4);
			if (frame_cnt % 100 == 0)
			{
				cv::Mat face_best = tracker.get_best_face_of_a_person(track_res[i].index, 30);
				cv::imshow("face", face_best);
				//cv::imwrite("best_face_period_" + std::to_string(frame_cnt / 30)+ "_index_" + std::to_string(track_res[i].index) + ".jpg", face_best);
			}
		}
		if (frame_cnt % 30 == 0)
		{
			tracker.quality_cmp();
		}
		imshow("Results_frontal", result_frontal);
		++frame_cnt;
		//std::cout << "frame count: " << frame_cnt << std::endl;
		cv::waitKey(1);
		//std::cout << "frame number: " << frame_cnt << std::endl;

		face_bbox.clear();
	}
	free(pBuffer);
	
}
/*
using namespace std;
using namespace cv;
int main(int argc, char** argv) {
	std::string trackingAlg = "KCF";
	if (argc > 2)
		trackingAlg = argv[2];
	MultiTracker trackers(trackingAlg);
	vector<Rect2d> objects;

	std::string video = argv[1];
	VideoCapture cap(video);

	Mat frame;

	cap >> frame;
	selectROI("tracker", frame, objects);

	if (objects.size() < 1)
		return 0;

	trackers.add(frame, objects);

	printf("Start the tracking process, press ESC to quit.\n");
	double t1 = 0, t2 = 0, time = 0;
	printf("frame height %d, width %d\t ", frame.rows, frame.cols);
	for (;;) {
		t1 = (double)cv::getTickCount();
		cap >> frame;
		if (frame.rows == 0 || frame.cols == 0)
			break;
		trackers.update(frame);
		t2 = (double)cv::getTickCount();
		time = (t2 - t1) / getTickFrequency()*1000.0;
		printf("time used %f ms\n", time);

		
		for (unsigned i = 0; i < trackers.objects.size(); i++) {
		rectangle(frame, trackers.objects[i], Scalar(255, 0, 0), 2, 1);
		}
		imshow("tracker", frame);
		if (waitKey(1) == 27) break;
		
	}
}
*/



/*
int main(int argc, char** argv) {
if (argc < 2) {
return 0;
}
//设置默认跟踪算法
std::string trackingAlg = "KCF";
//使用参数设置跟踪算法
if (argc > 2)
trackingAlg = argv[2];
//创建tracker
MultiTracker trackers;
//跟踪目标矩形框列表
vector<Rect2d> objects;
//设置输入视频
std::string video = argv[1];
VideoCapture cap(video);
Mat frame;
//设置bounding box
cap >> frame;
vector<Rect> ROIs;
//opencv 3.4.0  https://docs.opencv.org/trunk/d5/d07/tutorial_multitracker.html
//使用 selctROIs("tracker",frame,ROIs) tracker is windowName
while (true) {
Rect2d roi = selectROI("tracker", frame);
ROIs.push_back(roi);
if (roi.width == 0 && roi.height == 0)
break;
}
if (ROIs.size() < 1)
return 0;
std::vector<Ptr<Tracker>> algorithms;
for (size_t i = 0; i < ROIs.size(); i++) {
algorithms.push_back(createTrackerByName());
objects.push_back(ROIs[i]);
}
trackers.add(algorithms, frame, objects);
for (unsigned i = 0; i<trackers.getObjects().size(); i++)
rectangle(frame, trackers.getObjects()[i], Scalar(255, 0, 0), 2, 1);

// show image with the tracked object
imshow("tracker", frame);

//quit on ESC button
if (waitKey(1) == 27)
break;
}

return 0;

}
*/

