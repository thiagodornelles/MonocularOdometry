#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <thread>
#include "interface.h"
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

enum Matcher {KNN, Optical};

class Odometry
{
public:
    Odometry(int argc, char *argv[], bool showTrajectory3d);

    //Calcula o 'Scale' de uma imagem para outra
    double getAbsoluteScale(int frame_id, char *address, Point2d &gt, Point3f &gt3d);

    //Feature Tracking usando OpticalFlow
    void featureTrackingOpticalFlow(Mat img1, Mat img2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);

    //Feature Detection usando FAST
    void featureDetection(Mat img_1, vector<Point2f>& points1);

    void getTransformationsBetween2Frames(Mat frame1, Mat frame2, Matcher type_matcher, vector< vector<DMatch> >& good, Mat& R_f, Mat& t_f, vector<KeyPoint>& kps1, vector<KeyPoint>& kps2);

    Mat meanTranslation(Mat t, Mat t_hist);

    Mat meanRotation(Mat R, Mat R_hist);

    void Run();

    void initialize();



    VideoCapture cap;
    char *address;
    int type_match, stepFrames;

    Mat frame1, frame2, frame_hist;
    Mat traj, R, t, R_hist, t_hist;

    vector< vector<DMatch> > good;
    vector<KeyPoint> kps1, kps2, kps_hist;
    vector<Mat> allframes, allR, allt;
    vector<Point3f> alltrajectory, alltrajGT;
    Point2d p0, gt;
    Point3f gt3d;

    int max_hist;
    int num_frame;
    int cont_hist;
    float scale_hist;
    thread* VOthread;
    thread* Interf;

    interface* ginter;

};

#endif // ODOMETRY_H








