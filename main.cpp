#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ccalib.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    Ptr<Feature2D> orb = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    VideoCapture cap("../../../video1.avi");
//    481.20,	0,          319.50
//    0,        -480.00,	239.50
//    0,        0,          1
    Mat frame1, frame2, output;    
    Mat desc1, desc2;
    vector<KeyPoint> kps1, kps2;
    vector<Point2f> points1, points2;
    vector< vector<DMatch> > matches;
    vector< vector<DMatch> > good;
    cap >> frame1;
    while(cap.isOpened()){
        for (int i = 0; i < 10 && cap.isOpened(); ++i) {
            cap >> frame2;
        }
        if(frame2.empty()) break;

        orb->detectAndCompute(frame1, noArray(), kps1, desc1);
        orb->detectAndCompute(frame2, noArray(), kps2, desc2);
//        drawKeypoints(frame1, kps1, frame1);
//        drawKeypoints(frame2, kps2, frame2);
        matcher->knnMatch(desc1, desc2, matches, 2);

        //Pegando matchings mais confiáveis
        for (int i = 0; i < matches.size(); ++i) {
            DMatch d1  = matches[i][0];
            DMatch d2  = matches[i][1];            
            if (d1.distance < 0.6 * d2.distance) {
                vector<DMatch> v;
                v.push_back(d1);
                good.push_back(v);
                KeyPoint kt1 = kps1.at(d1.queryIdx);
                points1.push_back(kt1.pt);
                KeyPoint kt2 = kps2.at(d1.trainIdx);
                points2.push_back(kt2.pt);
            }
        }
        drawMatches(frame1, kps1, frame2, kps2, good, output);
//        KeyPoint::convert(kps1, points1, vector<int>());
//        KeyPoint::convert(kps2, points2, vector<int>());
//        cout << points1.size() << " " << points2.size() << endl;
        Mat E = findEssentialMat(points1, points2, -480.00f);
//        cout << E << endl;
        Mat R, t;
        recoverPose(E, points1, points2, R, t, -480.00f);
//        cout << R << endl;
        cout << t << endl;

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
//        imshow("Frame t+1", frame2);
        imshow("Output", output);
        if(waitKey(0) == 'q') cap.release();
        frame2.copyTo(frame1);
        good.clear();
        matches.clear();
        kps1.clear();
        kps2.clear();
    }
    cap.release();
    return 0;
}
