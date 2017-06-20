#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdlib>
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

//Calcula o 'Scale' de uma imagem para outra
double getAbsoluteScale(int frame_id, char *address){

  string line;
  int i = 0;

  ifstream myfile (address);
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
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
        Point2f pt = points2.at(i - indexCorrection);
        if((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)){
            if((pt.x < 0) || (pt.y < 0))
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


void getTransformationsBetween2Frames(Mat frame1, Mat frame2, Matcher type_matcher, vector< vector<DMatch> >& good, Mat& R_f, Mat& t_f, vector<KeyPoint>& kps1, vector<KeyPoint>& kps2){
    Mat desc1, desc2;

    vector<Point2f> points1, points2;
    vector< vector<DMatch> > matches;

    Mat E;
    vector<uchar> status;

    Point2d pp = Point2d(607.1928, 185.2157);
    float focal = 718.8560;


    if(type_matcher == KNN){
        Ptr<Feature2D> orb = ORB::create(600);
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

        orb->detectAndCompute(frame1, noArray(), kps1, desc1, false);
        orb->detectAndCompute(frame2, noArray(), kps2, desc2, false);

        matcher->knnMatch(desc1, desc2, matches, 2);

        float sum_good = 0;
        //Pegando matchings mais confiáveis
        for (int i = 0; i < matches.size(); ++i) {
            DMatch d1  = matches[i][0];
            DMatch d2  = matches[i][1];
//            cout<<"D1:"<<d1.distance<<endl<<"D2:"<<d2.distance<<endl<<endl;

            if (d1.distance < 0.8 * d2.distance) {
                vector<DMatch> v;
                v.push_back(d1);
                sum_good += d1.distance;
                good.push_back(v);
                KeyPoint kt1 = kps1.at(d1.queryIdx);
                points1.push_back(kt1.pt);
                KeyPoint kt2 = kps2.at(d1.trainIdx);
                points2.push_back(kt2.pt);
            }
        }
        cout<<"SUM:"<<sum_good<<endl;
    }else{
        featureDetection(frame1,points1);
        featureTrackingOpticalFlow(frame1, frame2, points1, points2, status);
    }
    if(points1.size() < 10){
        frame2.copyTo(frame1);
        good.clear();
        matches.clear();
        kps1.clear();
        kps2.clear();
        points1.clear();
        points2.clear();
        R_f = 0;
        t_f = 0;
    }else{
        E = findEssentialMat(points2, points1, /*1.f/-480.f*/focal, pp, RANSAC, 0.9999999);
        recoverPose(E, points2, points1, R_f, t_f, focal, pp);
    }
}

int main(int argc, char *argv[]) {
   /*-------COMANDOS----------
    * argv[1] - endereco do video
    * argv[2] - endereco do ground-truth
    * argv[3] - tipo do matcher
    * argv[4] - quantidade de frames que serão pulados
   */

    Mat frame1, frame2, frame_hist;
    Mat traj = Mat::zeros(500, 500, CV_8UC3);

    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat::zeros(3, 1, CV_64F);
    Mat R_hist = Mat::eye(3, 3, CV_64F);
    Mat t_hist = Mat::zeros(3, 1, CV_64F);

    vector< vector<DMatch> > good;
    vector<KeyPoint> kps1, kps2, kps_hist;
    vector<Mat> allframes, allR, allt;
    Point2d p0 = Point2d(150, 350);

    int max_hist = 0;
    int num_frame = 1;
    int cont_hist = 0;

    if(argc != 5){
        cout<<"Confira os parametros necessarios para a execucao do programa.\n<Endereco do video> <Endereco do Ground-Truth> <Tipo do Matcher> <Step dos frames>"<<endl;
        return 0;
    }

    //-----------CONFIGURANDO PARAMETROS-----------
    VideoCapture cap(argv[1]);
    if(!cap.isOpened()){
        cout<<"Erro ao abrir o video. Verifique o endereco informado"<<endl;
        return 0;
    }

    //Selecionando o tipo do matcher (KNN ou Optical)
    Matcher type_matcher = (!strcmp(argv[3],"KNN"))? KNN:Optical;
    int stepFrames = atoi(argv[4]);

//    481.20,	0,          319.50
//    0,        -480.00,	239.50
//    0,        0,          1


    //Comecando a analisar o video, capturando o primeiro frame
    cap >> frame1;
    frame1.copyTo(frame_hist);

    Mat t_f, R_f, t_f_hist, R_f_hist;

    while(cap.isOpened()){
        for (int i = 0; i < stepFrames && cap.isOpened(); ++i) {
            cap >> frame2;
            allframes.push_back(frame2);
        }
        if(frame2.empty()){
            cout<<"Erro na leitura do frame2"<<endl;
            break;
        }

//        drawMatches(frame1, kps1, frame2, kps2, good, output);
//        Point2d pp = Point2d(319.50f, 239.50f);
//        Point2d pp = Point2d(0, 0);




        //Transformacao entre t e t+1
        getTransformationsBetween2Frames(frame1, frame2, type_matcher, good, R_f, t_f, kps1, kps2); //ESSA FUNCAO CALCULA R E t

        if(cont_hist >= max_hist)
            //Transformacao entre t-n e t+1
            getTransformationsBetween2Frames(frame_hist, frame2, type_matcher, good, R_f_hist, t_f_hist, kps_hist, kps2); //ESSA FUNCAO CALCULA R E t

        float scale = getAbsoluteScale(num_frame, argv[2]);
        num_frame = num_frame+stepFrames+1;//Atualizando o índice do valor de escala

        //ESSE IF FAZ MUITA DIFERENCA NA ESTIMATIVA
         if ((scale>0.1)&&(t_f.at<double>(2) > t_f.at<double>(0)) && (t_f.at<double>(2) > t_f.at<double>(1))) {
            t = t + scale*(R*t_f);
            R = R_f*R;

            allt.push_back(t);
            allR.push_back(R);

            if(cont_hist >= max_hist){
                t_hist = t_hist + scale*(R_hist*t_f_hist);
                R_hist = R_f_hist*R_hist;

                cout<<"CONT:"<<cont_hist<<endl;
                cout<<"T:"<<t<<endl;
                cout<<"T_hist:"<<t_hist<<endl;
                //COMPARAR AS DUAS ESTIMATIVAS (t,R com t_hist, R_hist)
                //Atualizando os parametros
                cont_hist = 0;
                allframes.front().copyTo(frame_hist);
                t_hist = allt.front();
                R_hist = allR.front();

                //Removendo os primeiros elementos que ja foram usados
                allframes.erase(allframes.begin());
                allt.erase(allt.begin());
                allR.erase(allR.begin());

            }
            cont_hist++; //Atualizando a quantidade de frames pra depois calcular o hist
        }

        float rot[2][2] = {cos(80),-sin(80),sin(80),cos(80)}; //Rotacao 2D do ponto da trajetoria
        Point2d p1 = Point2d((int)(t.at<double>(2)*rot[0][0]+t.at<double>(0)*rot[0][1]),
                             (int)(t.at<double>(2)*rot[1][0]+t.at<double>(0)*rot[1][1])); //Ponto rotacionado
        p1 = Point2d(p1.x+150, p1.y+350); //Transladando o ponto

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
        kps1.clear();
        kps2.clear();
        kps_hist.clear();

    }
    cap.release();
    return 0;
}
