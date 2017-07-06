#ifndef INTERFACE_H
#define INTERFACE_H
#include <pangolin/display/device/display_glut.h>
#include <GL/glut.h>
#include <pangolin/pangolin.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace pangolin;
using namespace cv;

struct CustomType
{
  CustomType()
    : x(0), y(0.0f) {}

  CustomType(int x, float y, std::string z)
    : x(x), y(y), z(z) {}

  int x;
  float y;
  std::string z;
};

class interface
{
public:
    interface(vector<Point3f>* alt, vector<Point3f>* allGT);

    void Run();
    void drawSquad(Point3f point, float color[]);
    void drawLine(Point3f point1, Point3f point2, float color[]);
    void drawCubeFace(Point3f center, int side, float color[]);

    void drawVO();
    void setImageData(unsigned char * imageArray, int size);


    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    vector<Point3f>* alltraj;
    vector<Point3f>* allGT;

    bool exemplo;

};

#endif // INTERFACE_H
