#include<opencv2\core\utility.hpp>
#include<opencv2\videoio\videoio.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\tracking\tracker.hpp>
#include<iostream>
#include<cstring>
using namespace std;
using namespace cv;
// opencv version: 3.1.0  
// usage: D:\code\cpp\obj_tracking_opencv\x64\Debug>obj_tracking_opencv.exe D:\\data\\faceocc2.webm
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

		//draw the tracked objects
		/*
		for (unsigned i = 0; i < trackers.objects.size(); i++) {
			rectangle(frame, trackers.objects[i], Scalar(255, 0, 0), 2, 1);
		}
		imshow("tracker", frame);
		if (waitKey(1) == 27) break;
		*/
	}
}



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

