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
double getAbsoluteScale(int frame_id, char *address, Point2d& gt){

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
            //      cout << line << '\n';
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
    //  cout<<"PONTO DO GT:"<<endl;
    //  cout<<"X:"<<x_prev<<" Y:"<<y_prev<<" Z:"<<z_prev<<endl;

    float rot[2][2] = {cos(80),-sin(80),sin(80),cos(80)}; //Rotacao 2D do ponto do GT
    gt = Point2d((int)(z_prev*rot[0][0]+x_prev*rot[0][1])+150,
            (int)(z_prev*rot[1][0]+x_prev*rot[1][1])+350); //Ponto rotacionado e transladado

    //  gt = Point2d(x_prev+150, z_prev+350);
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

            if (d1.distance < 0.5 * d2.distance) {
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
        //        cout<<"SUM:"<<sum_good<<endl;
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

Mat meanTranslation(Mat t, Mat t_hist){
    Mat t_mean = Mat::zeros(3, 1, CV_64F);

    t_mean.at<double>(0) = (t.at<double>(0)+t_hist.at<double>(0))/2.0;
    t_mean.at<double>(1) = (t.at<double>(1)+t_hist.at<double>(1))/2.0;
    t_mean.at<double>(2) = (t.at<double>(2)+t_hist.at<double>(2))/2.0;

    //    return t_mean;
    return t;
}

Mat meanRotation(Mat R, Mat R_hist){
    Mat R_mean = Mat::eye(3, 3, CV_64F);

    for(int i = 0; i < R_mean.cols; i++){
        for(int j = 0; j < R_mean.rows; j++)
            R_mean.at<double>(i,j) = atan2(sin(R.at<double>(i,j))+sin(R_hist.at<double>(i,j)),
                                           cos(R.at<double>(i,j))+cos(R_hist.at<double>(i,j)));
    }
    //    return R_mean;
    return R;
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
    Point2d gt;
    int max_hist = 2;
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

    float scale_hist = 0;

    while(cap.isOpened()){

        for (int i = 0; i < stepFrames && cap.isOpened(); ++i) {
            cap >> frame2;

        }
        if(frame2.empty()){
            cout<<"Erro na leitura do frame2"<<endl;
            break;
        }


        //        drawMatches(frame1, kps1, frame2, kps2, good, output);
        //        Point2d pp = Point2d(319.50f, 239.50f);
        //        Point2d pp = Point2d(0, 0);

        Mat t_f, R_f;
        //Transformacao entre t e t+1
        getTransformationsBetween2Frames(frame1, frame2, type_matcher, good, R_f, t_f, kps1, kps2); //ESSA FUNCAO CALCULA R E t

        float scale = getAbsoluteScale(num_frame, argv[2], gt);

        //ESSE IF FAZ MUITA DIFERENCA NA ESTIMATIVA. IF MILAGROSO!!!!!!!!
        if ((scale>0.1)&&(t_f.at<double>(2) > t_f.at<double>(0)) && (t_f.at<double>(2) > t_f.at<double>(1))) {
            Mat temp;
            frame2.copyTo(temp);
            allframes.push_back(temp);


            scale_hist += scale;
            num_frame = num_frame+stepFrames;//Atualizando o índice do valor de escala

            t = t + scale*(R*t_f);
            R = R_f*R;

            cont_hist++; //Atualizando a quantidade de frames pra depois calcular o hist

            if(cont_hist >= max_hist){
                //Transformacao entre t-n e t+1
                Mat t_f_hist, R_f_hist;
                vector< vector<DMatch> > good_hist;
                vector<KeyPoint> kps2_hist, kps_hist;

                getTransformationsBetween2Frames(frame_hist, frame2, type_matcher, good_hist, R_f_hist, t_f_hist, kps_hist, kps2_hist); //ESSA FUNCAO CALCULA R E t

                Mat t_mean = t;
                Mat R_mean = R;

                scale_hist /= (float)cont_hist; //A escala do "frame_hist" é a media das escalas entre o "frame_hist" e o "frame2"
                if(!t_f_hist.empty() && !R_f_hist.empty()){
                    t_hist = t_hist + scale_hist*(R_hist*t_f_hist);
                    R_hist = R_f_hist*R_hist;

                    //ESSE IF FAZ MUITA DIFERENCA NA ESTIMATIVA. IF MILAGROSO!!!!!!!!
                    if ((scale_hist>0.1)&&(t_f_hist.at<double>(2) > t_f_hist.at<double>(0)) && (t_f_hist.at<double>(2) > t_f_hist.at<double>(1))) {

                        t_mean = meanTranslation(t, t_hist); //DO JEITO QUE EU DEIXEI, NAO ESTA CALCULANDO A MEDIA, SO RETORNANDO O VALOR DE t
                        R_mean = meanRotation(R, R_hist); //DO JEITO QUE EU DEIXEI, NAO ESTA CALCULANDO A MEDIA, SO RETORNANDO O VALOR DE R

                        //                    cout<<endl<<"T:"<<t<<endl;
                        //                    cout<<"T_hist:"<<t_hist<<endl;
                        //                    cout<<"T_mean:"<<t_mean<<endl<<endl;
                        //                    cout<<endl<<"R:"<<R<<endl;
                        //                    cout<<"R_hist:"<<R_hist<<endl;
                        //                    cout<<"R_mean:"<<R_mean<<endl<<endl;


                        //Desenhando as features da transformacao entre "frame_hist" e "frame2"
                        for (int i = 0; i < good_hist.size(); ++i) {
                            DMatch d1  = good_hist[i][0];
                            KeyPoint kt1 = kps_hist.at(d1.queryIdx);
                            KeyPoint kt2 = kps2_hist.at(d1.trainIdx);
                            circle(frame_hist, kt1.pt, 1, cvScalar(255,0,0));
                            line(frame_hist, kt1.pt, kt2.pt, cvScalar(0,255,0));
                            circle(frame_hist, kt2.pt, 1, cvScalar(0,0,255));
                        }
                    }
                    //Atualizando o contador da janela do "hist"
                    cont_hist--;

                    //Atualizando os valores das variaveis "hist"
                    allframes.front().copyTo(frame_hist);
                    t_hist = allt.front();
                    R_hist = allR.front();

                    //Removendo os primeiros elementos que ja foram usados
                    allframes.erase(allframes.begin());
                    allt.erase(allt.begin());
                    allR.erase(allR.begin());

                    //Atualizando o valor do r e do T
                    t = t_mean;
                    R = R_mean;

                }
                scale_hist = 0;
            }

            //Inserindo os valores de t e R nos vectors de controle
            Mat aux;
            t.copyTo(aux);
            allt.push_back(aux);
            R.copyTo(aux);
            allR.push_back(aux);
        }else{ //SE NAO ENTROU NO IF MILAGROSO, DESCONSIDERO O FRAME2
            continue;
        }

        float rot[2][2] = {cos(80),-sin(80),sin(80),cos(80)}; //Rotacao 2D do ponto da trajetoria
        Point2d p1 = Point2d((int)(t.at<double>(2)*rot[0][0]+t.at<double>(0)*rot[0][1]),
                (int)(t.at<double>(2)*rot[1][0]+t.at<double>(0)*rot[1][1])); //Ponto rotacionado
        p1 = Point2d(p1.x+150, p1.y+350); //Transladando o ponto

        //        Point2d p2 = Point2d(10*t.at<double>(1)+50, 10*t.at<double>(2)+50);
        circle(traj, p1, 1, cvScalar(0,0,255), 2); //ponto da trajetória
        circle(traj, gt, 1, cvScalar(255,0,255), 2); //ponto do ground-truth
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
