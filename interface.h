#ifndef INTERFACE_H
#define INTERFACE_H
#include <pangolin/display/device/display_glut.h>
#include <GL/glut.h>
#include <pangolin/pangolin.h>
#include <odometry.h>

#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace pangolin;

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
