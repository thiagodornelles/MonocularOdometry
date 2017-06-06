#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    Ptr<Feature2D> orb = ORB::create(200);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    VideoCapture cap("../../../video2.avi");
//    481.20,	0,          319.50
//    0,        -480.00,	239.50
//    0,        0,          1
    Mat frame1, frame2, output;
    Mat traj = Mat::zeros(500, 500, CV_8UC3);;
    Mat desc1, desc2;
    vector<KeyPoint> kps1, kps2;
    vector<Point2f> points1, points2;
    vector< vector<DMatch> > matches;
    vector< vector<DMatch> > good;
    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat::zeros(3, 1, CV_64F);
    Mat E;
    Point2d p0 = Point2d(250, 250);
    cap >> frame1;
    while(cap.isOpened()){
        for (int i = 0; i < 10 && cap.isOpened(); ++i) {
            cap >> frame2;
        }
        if(frame2.empty()) break;

        orb->detectAndCompute(frame1, noArray(), kps1, desc1, false);
        orb->detectAndCompute(frame2, noArray(), kps2, desc2, false);
//        drawKeypoints(frame1, kps1, frame1);
//        drawKeypoints(frame2, kps2, frame2);
        matcher->knnMatch(desc1, desc2, matches, 2);

        //Pegando matchings mais confiáveis
        for (int i = 0; i < matches.size(); ++i) {
            DMatch d1  = matches[i][0];
            DMatch d2  = matches[i][1];            
            if (d1.distance < 0.8 * d2.distance) {
                vector<DMatch> v;
                v.push_back(d1);
                good.push_back(v);
                KeyPoint kt1 = kps1.at(d1.queryIdx);
                points1.push_back(kt1.pt);
                KeyPoint kt2 = kps2.at(d1.trainIdx);
                points2.push_back(kt2.pt);
            }
        }
        if(points1.size() < 10){
            frame2.copyTo(frame1);
            good.clear();
            matches.clear();
            kps1.clear();
            kps2.clear();
            points1.clear();
            points2.clear();            
            continue;
        }
//        drawMatches(frame1, kps1, frame2, kps2, good, output);
//        Point2d pp = Point2d(319.50f, 239.50f);
        Point2d pp = Point2d(0, 0);
        E = findEssentialMat(points1, points2, 1.f/-480.f, pp, RANSAC, 0.9999999);
        Mat t_f = Mat::zeros(3,1, CV_64F);
        Mat R_f;
        recoverPose(E, points1, points2, R_f, t_f);
        cout << R << endl;
        cout << t << endl;
        t = t + 8.f*(R_f*t_f);
        R = R_f*R;

//        traj.setTo(0);

        Point2d p1 = Point2d(-t.at<double>(0)+250, t.at<double>(1)+250);
//        Point2d p2 = Point2d(10*t.at<double>(1)+50, 10*t.at<double>(2)+50);
        circle(traj, p1, 1, cvScalar(0,0,255), 2);
        line(traj, p0, p1, cvScalar(0,255,255),2);
        p0 = p1;
//        cout << p1.x << " " << p1.y << endl;
//        line(traj, p1, p2, (0,255,255),2);


//        for (int i = 0; i < good.size(); ++i) {
//            vector<DMatch> v = good.at(i);
//            DMatch d = v.at(0);
//            KeyPoint k1 = kps1.at(d.queryIdx);
//            KeyPoint k2 = kps2.at(d.trainIdx);
//            cout << "x1 " << k1.pt.x << " ";
//            cout << "y1 " << k1.pt.y << " ";
//            cout << "x2 " << k2.pt.x << " ";
//            cout << "y2 " << k2.pt.y << "\n";
//        }

//        imshow("Frame t", frame1);
        frame2.copyTo(frame1);
        for (int i = 0; i < good.size(); ++i) {
            DMatch d1  = good[i][0];            
            KeyPoint kt1 = kps1.at(d1.queryIdx);
            KeyPoint kt2 = kps2.at(d1.trainIdx);
            circle(frame2, kt1.pt, 1, cvScalar(255,0,0));
            line(frame2, kt1.pt, kt2.pt, cvScalar(0,255,0));
            circle(frame2, kt2.pt, 1, cvScalar(0,0,255));
        }

        imshow("Frame t+1", frame2);
//        imshow("Output", output);
        imshow("trajetoria", traj);
        if(waitKey(0) == 'q') cap.release();
        good.clear();
        matches.clear();
        kps1.clear();
        kps2.clear();
        points1.clear();
        points2.clear();
    }
    cap.release();
    return 0;
}

