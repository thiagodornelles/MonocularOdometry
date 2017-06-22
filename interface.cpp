#include "interface.h"

interface::interface()
{
    mViewpointX = 0;
    mViewpointY = -10;
    mViewpointZ = -0.1;
    mViewpointF = 2000;
    mKeyFrameSize = 0.03;
    mKeyFrameLineWidth = 1;
    mGraphLineWidth = 1;
    mPointSize = 2;
    mCameraSize = 0.15;
    mCameraLineWidth = 2;

}


void interface::Run(){
    CreateWindowAndBind("Trajectory", 1024, 768);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("MENU").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

    Var<bool> option("TESTE", true, true);
    Var<bool> option2("TESTE2", false, true);

    OpenGlRenderState s_cam(ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
                            ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

    View& d_cam = CreateDisplay().SetBounds(0.0, 1.0, Attach::Pix(175), 1.0, -1024.0f/768.0f)
                                 .SetHandler(new Handler3D(s_cam));

    OpenGlMatrix Twc;
    Twc.SetIdentity();


    while(1){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.7f,0.7f,0.7f,1.0f);

        pangolin::FinishFrame();
    }
}
