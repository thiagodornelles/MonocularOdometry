#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
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

enum Matcher {KNN, Optical};


double euclidianDistance(Point2d gt, Point2d est){
    double er = sqrt(pow(gt.x-est.x,2) + pow(gt.y-est.y,2));
    cerr << "Error:" << er << endl;
    return er;
}

Point2d getGroundTruth(int frame_id, char *address){
    string line;
    int i = 0;
    ifstream myfile (address);
    double x = 0, y = 0, z = 0;
    if (myfile.is_open()) {
        while (( getline (myfile,line) ) && (i <= frame_id)) {
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)  {
                in >> z ;
                if (j==7) y=z;
                if (j==3)  x=z;
            }
            i++;
        }        
        myfile.close();
    }
    else {
        cout << "Unable to open file";
    }
    return Point2d(x,-z);
}

Point2d getGroundTruthICL(int frame_id, char *address){
    string line;
    int i = 0;
    ifstream myfile (address);
    double x = 0, y = 0, z = 0;
    if (myfile.is_open()) {
        while (( getline (myfile,line) ) && (i <= frame_id)) {
            std::istringstream in(line);
            for (int j=0; j<4; j++)  {
                in >> z ;
                if (j==2) y=z*100;
                if (j==1)  x=z*100;
                if (j==3)  z=z*100;
            }
            i++;
        }
        myfile.close();
    }
    else {
        cout << "Unable to open file";
    }
    return Point2d(-x,-z);
}

double getAbsoluteScaleICL(int frame_id, char *address){
    string line;
    int i = 0;
    ifstream myfile (address);
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open()){
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<=3; j++)  {
                in >> z ;
                if (j==2) y=z*100;
                if (j==1)  x=z*100;
                if (j==3)  z=z*100;
            }
            i++;
        }
        myfile.close();
    }
    else {
        cout << "Unable to open file";
        return 0;
    }
    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev));
}


//Calcula o 'Scale' de uma imagem para outra
double getAbsoluteScale(int frame_id, char *address){
    string line;
    int i = 0;
    ifstream myfile (address);
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open()){
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)  {
                in >> z ;
                if (j==7) y=z;
                if (j==3)  x=z;
            }

            i++;
        }
        myfile.close();
    }
    else {
        cout << "Unable to open file";
        return 0;
    }

    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
}

//Feature Tracking usando OpticalFlow
void featureTrackingOpticalFlow(Mat img1, Mat img2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status){
    vector<float> err;

    Size winSize=Size(21,21);
    TermCriteria termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,30,0.01);

    calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    int indexCorrection = 0;

    for(int i = 0; i < status.size(); i++){
        Point2f pt1 = points1.at(i - indexCorrection);
        Point2f pt2 = points2.at(i - indexCorrection);
        float ptdx = pt2.x - pt1.x;
        float ptdy = pt2.y - pt1.y;
        float dist = sqrt(ptdx*ptdx + ptdy*ptdy);
        if((status.at(i) == 0) || (pt2.x < 0) || (pt2.y < 0) || dist > img1.rows/2){
            if((pt2.x < 0) || (pt2.y < 0))
                status.at(i) = 0;
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}

//Feature Detection usando FAST
void featureDetection(Mat img_1, vector<Point2f>& points1)	{   //USA FAST
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}

int main(int argc, char *argv[]) {
    /*-------COMANDOS----------
    * argv[1] - endereco do video
    * argv[2] - endereco do ground-truth
    * argv[3] - tipo do matcher
   */

    VideoCapture cap(argv[1]);
    if(!cap.isOpened()){
        cout<<"Erro ao abrir o video. Verifique o endereco informado"<<endl;
        return 0;
    }

    Matcher type_matcher = (!strcmp(argv[3],"KNN"))? KNN:Optical; //selecionando o tipo do matcher (KNN ou Optical)

    Ptr<Feature2D> orb = ORB::create(600);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    Mat frame1, frame2;
    Mat traj = Mat::zeros(1500, 1500, CV_8UC3);
    traj.setTo(0xFFFFFF);
    Mat desc1, desc2;
    vector<KeyPoint> kps1, kps2;
    vector<Point2f> points1, points2;
    vector< vector<DMatch> > matches;
    vector< vector<DMatch> > good;
    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat::zeros(3, 1, CV_64F);
    Mat E;
    const int deslx = 600;
    const int desly = 500;
    Point2d p0 = Point2d(deslx, desly);
    vector<uchar> status;
    int num_frame = 1;

    //Comecando a analisar o video
    cap >> frame1;
    while(cap.isOpened()){
        if (strcmp(argv[5], "ICL") == 0){
            for (int i = 0; i < 5 && cap.isOpened(); ++i) {
                cap >> frame2;
            }
        }
        else {
            cap >> frame2;
        }
        if(frame2.empty()) break;
        if(type_matcher == KNN){
            orb->detectAndCompute(frame1, noArray(), kps1, desc1, false);
            orb->detectAndCompute(frame2, noArray(), kps2, desc2, false);            
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
        }
        else{
            if (strcmp(argv[3], "KNNOPT") == 0) {
                orb->detectAndCompute(frame1, noArray(), kps1, desc1, false);
                for (int i = 0; i < kps1.size(); ++i) {
                    KeyPoint k  = kps1[i];
                    points1.push_back(k.pt);
                }
            }
            else {
                Mat edges;
                Canny(frame1, edges, 90, 100);
                int step = 5;
                for (int row = 0; row < edges.rows; row+=step) {
                    unsigned char *data = edges.ptr(row);
                    for (int col = 0; col < edges.cols; col+=step) {
                        int value = *data;
                        if (value > 150){
                            points1.push_back(Point2f(col, row));
                        }
                        data+=step;
                    }
                }
            }
            featureTrackingOpticalFlow(frame1, frame2, points1, points2, status);
            //imshow("bordas", edges);
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
        //    481.20,	0,          319.50
        //    0,        -480.00,	239.50
        //    0,        0,          1

        //Informacoes da camera
        Point2d pp;
        float focal;
        if (strcmp(argv[5],"ICL") == 0) {
            pp = Point2d(319.50f, 239.50f);
            focal = -480.f;
        }
        else {
            pp = Point2d(607.1928, 185.2157);
            focal = 718.8560;
        }

        Mat t_f = Mat::zeros(3,1, CV_64F);
        Mat R_f;

        E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.9999999);
        recoverPose(E, points2, points1, R_f, t_f, focal, pp);
        float scale = 0;
        if (strcmp(argv[5],"ICL") == 0) {
            for (int i = 0; i < 5 && cap.isOpened(); ++i) {
                scale += getAbsoluteScaleICL(num_frame++, argv[2]);
            }
        }
        else {
            scale = getAbsoluteScale(num_frame++, argv[2]);
        }
        bool validFrame = false;
        //ESSE IF FAZ MUITA DIFERENCA NA ESTIMATIVA
        if ((scale > 0.1) &&
            (t_f.at<double>(2) > t_f.at<double>(0)) &&
            (t_f.at<double>(2) > t_f.at<double>(1))) {

            t = t + scale*(R*t_f);
            R = R_f*R;
            validFrame = true;
        }

        float angle = 80.1;
        //float angle = 78.9;
        float rot[2][2] = {cos(angle),-sin(angle),sin(angle),cos(angle)}; //Rotacao do ponto da trajetoria
        Point2d p1 = Point2d((int)(t.at<double>(2)*rot[0][0]+t.at<double>(0)*rot[0][1]),
                (int)(t.at<double>(2)*rot[1][0]+t.at<double>(0)*rot[1][1])); //Ponto rotacionado
        p1 = Point2d(p1.x+deslx, p1.y+desly); //Transladando o ponto
        //        Point2d p2 = Point2d(10*t.at<double>(1)+50, 10*t.at<double>(2)+50);
        putText(traj, "AZUL: ALGORITMO", Point(10,20), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(255,0,0), 2);
        putText(traj, "VERMELHO: GROUND TRUTH", Point(10,40), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0,0,255), 2);
        line(traj, p0, p1, cvScalar(255,0,0),2);
        p0 = p1;
        Point2d pg1, pg2;
        if (strcmp(argv[5], "ICL") == 0){
            pg1 = getGroundTruthICL(num_frame-1, argv[2]);
            pg2 = getGroundTruthICL(num_frame, argv[2]);
        }
        else {
            pg1 = getGroundTruth(num_frame-1, argv[2]);
            pg2 = getGroundTruth(num_frame, argv[2]);
        }
        pg1.x += deslx;
        pg1.y += desly;
        pg2.x += deslx;
        pg2.y += desly;
        line(traj, pg1, pg2, cvScalar(0,0,255), 2);

        //Medindo o erro
        double dist = euclidianDistance(pg2, p1);
        //cerr << dist << endl;

        frame2.copyTo(frame1);        
        if (type_matcher == KNN) {
            for (int i = 0; i < good.size(); ++i) {
                DMatch d1  = good[i][0];
                KeyPoint kt1 = kps1.at(d1.queryIdx);
                KeyPoint kt2 = kps2.at(d1.trainIdx);
                circle(frame2, kt1.pt, 1, cvScalar(255,0,0));
                line(frame2, kt1.pt, kt2.pt, cvScalar(0,255,0));
                circle(frame2, kt2.pt, 1, cvScalar(0,0,255));
            }
        }
        else {
            for (int i = 0; i < points2.size(); ++i) {
                Point2f p1 = points1[i];
                Point2f p2 = points2[i];
                circle(frame2, Point(p1.x, p1.y), 2, cvScalar(255,0,0),-1);
                line(frame2, Point(p1.x, p1.y), Point(p2.x, p2.y),
                     cvScalar(0,0,255), 1);
                circle(frame2, Point(p2.x, p2.y), 2, cvScalar(0,255,0),-1);
            }
        }
        imshow("trajetoria", traj);
        imshow("Frame t+1", frame2);
        int k = waitKey(1);
        if(k == 'q') cap.release();
        if(k == 'p') waitKey(0);
        good.clear();
        matches.clear();
        kps1.clear();
        kps2.clear();
        points1.clear();
        points2.clear();
    }
    cap.release();
    waitKey(0);
    return 0;
}
