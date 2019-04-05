#include<opencv2\core\utility.hpp>
#include<opencv2\videoio\videoio.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\tracking\tracker.hpp>
#include<iostream>
#include<cstring>
using namespace std;
using namespace cv;

/*
Point origin;//用于保存鼠标选择第一次单击时点的位置
Rect selection;//用于保存鼠标选择的矩形框
int trackObject = 0;
*/


/*
int main(int argc, char** argv) {
	Rect2d roi;
	Mat frame;
	Ptr<Tracker> tracker = Tracker::create("KCF");
	std::string video = argv[1];
	VideoCapture cap(video);
	cap >> frame;
	roi = selectROI("tracker", frame);
	if (roi.width == 0 || roi.height == 0)
		return 0;
	
	tracker->init(frame, roi);
	printf("Start the tracking process, press ESC to quit.\n");
	double t1=0, t2=0;
	double time=0;
	for (;;) {
		
		t1 = (double)cv::getTickCount();
		cap >> frame;
		printf("frame height %d, width %d\t ",frame.rows,frame.cols);
		if (frame.rows == 0 || frame.cols == 0)
			break;
		tracker->update(frame, roi);
		t2 = (double)cv::getTickCount();
		time = (t2 - t1) / getTickFrequency()*1000.0;
		printf("time used %f ms\n",time);
		//rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
		//imshow("tracker", frame);
		//if (waitKey(1) == 27)
		//	break;
	}
	return 0;
}
*/



