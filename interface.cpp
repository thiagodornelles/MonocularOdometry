#include "interface.h"

interface::interface(vector<Point3f> *allt, vector<Point3f> *allG, vector<float> *err)
{
    mViewpointX = 0;
    mViewpointY = -100;
    mViewpointZ = -0.1;
    mViewpointF = 2000;
    mKeyFrameSize = 0.03;
    mKeyFrameLineWidth = 1;
    mGraphLineWidth = 1;
    mPointSize = 2;
    mCameraSize = 0.15;
    mCameraLineWidth = 2;

    exemplo = true;
    alltraj = allt;
    allGT = allG;
    error = err;
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
    pangolin::Var<bool> menuError("menu.Error",false,true);

    // Define Camera Render Object (for view / scene browsing)
    OpenGlRenderState s_cam(ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                            ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));


    // Add named OpenGL viewport to window and provide 3D Handler
    View& d_cam1 = CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            /*.SetAspect(640.0f/480.0f)*/.SetHandler(new Handler3D(s_cam));

    // Default hooks for exiting (Esc) and fullscreen (tab).

    OpenGlMatrix Twc;
    Twc.SetIdentity();

    int t = 0;
    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //            Twc = ProjectionMatrix(0,0,30, 30, 30, 30, 1,1);

        glColor3f(1.0,1.0,1.0);
        if(menuFLL){
            //Media pontual entre a traj estimada e o GT
            float x = (alltraj->at(alltraj->size()-1).x + allGT->at(allGT->size()-1).x)/2.0;
            float y = (alltraj->at(alltraj->size()-1).y + allGT->at(allGT->size()-1).y)/2.0;
            float z = (alltraj->at(alltraj->size()-1).z + allGT->at(allGT->size()-1).z)/2.0;
            cout<<"X:"<<x<<" Y:"<<y<<endl;
            s_cam.SetModelViewMatrix(ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, x,y,z,0,-.5,0));
//            s_cam.Follow(Twc);
        }
        t++;
        d_cam1.Activate(s_cam);
        //            pangolin::glDrawColouredCube();
        if(menuTRAJ){
            float color[4] = {0, 0.8, 0, 1};
            drawSquad(alltraj->at(0), color);
            for(int i = 1; i < alltraj->size(); i++){
                drawSquad(alltraj->at(i), color);
                drawLine(alltraj->at(i-1), alltraj->at(i),color);
                cout<<"ERROR:"<<error->at(i)<<endl;
            }
        }

        if(menuGT){
            float color[4] = {0.8, 0, 0, 1};
            drawSquad(allGT->at(0), color);
            for(int i = 1; i < allGT->size(); i++){
                drawSquad(allGT->at(i), color);
                drawLine(allGT->at(i-1), allGT->at(i),color);
            }
        }
        if(menuError){
            for(int i = 0; i < alltraj->size(); i++){
                float color[4] = {0, 0, 1, 1};
                drawLine(alltraj->at(i), allGT->at(i), color);
            }
        }


        pangolin::FinishFrame();
    }

}


void interface::setImageData(unsigned char * imageArray, int size){
    for(int i = 0 ; i < size;i++) {
        imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
    }
}

void interface::drawSquad(Point3f center,float color[4]){

    glBegin(GL_QUADS);

        //Bottom
        drawCubeFace(center, 0, color);

        //Top
        drawCubeFace(center, 1, color);

        //Back
        drawCubeFace(center, 2, color);

        //Front
        drawCubeFace(center, 3, color);

        //Right
        drawCubeFace(center, 4, color);

        //Left
        drawCubeFace(center, 5, color);

    glEnd();  // End of drawing color-cube

}

void interface::drawCubeFace(Point3f center, int side, float color[4])
{
    float size = 1;
    float factor = 0.2;

    switch(side) {
        //Bottom
        case 0:
            glColor4f(color[0]-(2*factor), color[1]-(2*factor), color[2]-(2*factor), color[3]);
            glVertex3f(center.x+size, center.y+size, center.z);
            glVertex3f(center.x, center.y+size, center.z);
            glVertex3f(center.x, center.y+size, center.z+size);
            glVertex3f(center.x+size, center.y+size, center.z+size);
        break;

        //Top
        case 1:
            glColor4f(color[0]+factor, color[1]+factor, color[2]+factor, color[3]);
            glVertex3f(center.x+size, center.y, center.z+size);
            glVertex3f(center.x, center.y, center.z+size);
            glVertex3f(center.x, center.y, center.z);
            glVertex3f(center.x+size, center.y, center.z);
        break;

        //Back
        case 2:
            glColor4f(color[0]-factor, color[1]-factor, color[2]-factor, color[3]);
            glVertex3f(center.x+size, center.y+size, center.z+size);
            glVertex3f(center.x, center.y+size, center.z+size);
            glVertex3f(center.x, center.y, center.z+size);
            glVertex3f(center.x+size, center.y, center.z+size);
        break;

        //Front
        case 3:
            glColor4f(color[0]-factor, color[1]-factor, color[2]-factor, color[3]);
            glVertex3f(center.x+size, center.y, center.z);
            glVertex3f(center.x, center.y, center.z);
            glVertex3f(center.x, center.y+size, center.z);
            glVertex3f(center.x+size, center.y+size, center.z);
        break;

        //Right
        case 4:
            glColor4f(color[0], color[1], color[2], color[3]);
            glVertex3f(center.x+size, center.y+size, center.z);
            glVertex3f(center.x+size, center.y+size, center.z+size);
            glVertex3f(center.x+size, center.y, center.z+size);
            glVertex3f(center.x+size, center.y, center.z);
        break;

        //Left
        case 5:
            glColor4f(color[0], color[1], color[2], color[3]);
            glVertex3f(center.x, center.y+size, center.z+size);
            glVertex3f(center.x, center.y+size, center.z);
            glVertex3f(center.x, center.y, center.z);
            glVertex3f(center.x, center.y, center.z+size);
        break;
    }
}


void interface::drawLine(Point3f point1, Point3f point2, float color[4]){
    glColor4f(color[0], color[1], color[2], color[3]);
    glLineWidth(6);
    glBegin(GL_LINES);
    glVertex3f(point1.x+.5, point1.y+.5, point1.z+.5);
    glVertex3f(point2.x+.5, point2.y+.5, point2.z+.5);
    glEnd();
}
