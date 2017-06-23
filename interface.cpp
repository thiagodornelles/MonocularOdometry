#include "interface.h"

interface::interface(vector<Point3f> *allt, vector<Point3f> *allG)
{
    mViewpointX = 0;
    mViewpointY = -10;
    mViewpointZ = -0.1;
    mViewpointF = 200;
    mKeyFrameSize = 0.03;
    mKeyFrameLineWidth = 1;
    mGraphLineWidth = 1;
    mPointSize = 2;
    mCameraSize = 0.15;
    mCameraLineWidth = 2;

    exemplo = true;
    alltraj = allt;
    allGT = allG;

}


void interface::Run(){
    pangolin::CreateWindowAndBind("Visual Odometry",1024,768);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuGT("menu.Ground-Truth",false,true);
    pangolin::Var<bool> menuTRAJ("menu.Trajectory",false,true);
    pangolin::Var<bool> menuFLL("menu.Follow",false,true);

    // Define Camera Render Object (for view / scene browsing)
    OpenGlRenderState s_cam(ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                            ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));


    // Add named OpenGL viewport to window and provide 3D Handler
    View& d_cam1 = CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetAspect(640.0f/480.0f).SetHandler(new Handler3D(s_cam));

    // Default hooks for exiting (Esc) and fullscreen (tab).

    OpenGlMatrix Twc;
    Twc = ProjectionMatrix(1024, 768, 20, 20, 320, 240, 0.1, 1000);
    int t = 0;
    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //            Twc = ProjectionMatrix(0,0,30, 30, 30, 30, 1,1);

        glColor3f(1.0,1.0,1.0);
        if(menuFLL){
            s_cam.SetModelViewMatrix(ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0,0,0,0,-1.0,0));
            //                s_cam.Follow(Twc);
        }
        t++;
        d_cam1.Activate(s_cam);
        //            pangolin::glDrawColouredCube();
        if(menuTRAJ){
            for(int i = 0; i < alltraj->size(); i++)
                //                    cout<<"TESTE"<<alltraj->at(i).x<<endl;
                drawSquad(alltraj->at(i), false);
        }

        if(menuGT){
            for(int i = 0; i < allGT->size(); i++)
                //                    cout<<"TESTE"<<alltraj->at(i).x<<endl;
                drawSquad(allGT->at(i), true);
        }


        pangolin::FinishFrame();
    }

}


void interface::setImageData(unsigned char * imageArray, int size){
    for(int i = 0 ; i < size;i++) {
        imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
    }
}

void interface::drawSquad(Point3f point, bool gt){
    glColor3f(1,0,0);
    glBegin(GL_QUADS);
    glVertex3f(point.x+0,point.y+0,0+point.z);
    glVertex3f(point.x+0,point.y+1,0+point.z);
    glVertex3f(point.x+1,point.y+1,0+point.z);
    glVertex3f(point.x+1,point.y+0,0+point.z);
    glEnd();

    (gt)?glColor3f(0,1,0):glColor3f(1,1,0);
    glBegin(GL_QUADS);
    glVertex3f(point.x+0,point.y+0,0+point.z);
    glVertex3f(point.x+0,point.y+0,1+point.z);
    glVertex3f(point.x+1,point.y+0,1+point.z);
    glVertex3f(point.x+1,point.y+0,0+point.z);
    glEnd();

    glColor3f(0,0,1);
    glBegin(GL_QUADS);
    glVertex3f(point.x+0,point.y+0,1+point.z);
    glVertex3f(point.x+0,point.y+1,1+point.z);
    glVertex3f(point.x+1,point.y+1,1+point.z);
    glVertex3f(point.x+1,point.y+0,1+point.z);
    glEnd();

    glColor3f(0,1,1);
    glBegin(GL_QUADS);
    glVertex3f(point.x+1,point.y+1,1+point.z);
    glVertex3f(point.x+0,point.y+1,1+point.z);
    glVertex3f(point.x+0,point.y+1,0+point.z);
    glVertex3f(point.x+1,point.y+1,0+point.z);
    glEnd();

    glColor3f(1,0,1);
    glBegin(GL_QUADS);
    glVertex3f(point.x+1,point.y+1,1+point.z);
    glVertex3f(point.x+1,point.y+0,1+point.z);
    glVertex3f(point.x+1,point.y+0,0+point.z);
    glVertex3f(point.x+1,point.y+1,0+point.z);
    glEnd();

    glColor3f(1,1,1);
    glBegin(GL_QUADS);
    glVertex3f(point.x+0,point.y+1,1+point.z);
    glVertex3f(point.x+0,point.y+0,1+point.z);
    glVertex3f(point.x+0,point.y+0,0+point.z);
    glVertex3f(point.x+0,point.y+1,0+point.z);
    glEnd();

}
