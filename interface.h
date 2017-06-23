#ifndef INTERFACE_H
#define INTERFACE_H
#include <pangolin/display/device/display_glut.h>
#include <GL/glut.h>
#include <pangolin/pangolin.h>


#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace pangolin;

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
    interface();

    void Run();
    void drawGT();
    void drawVO();


    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

};

#endif // INTERFACE_H
