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
//    CreateWindowAndBind("Trajectory", 1024, 768);

//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    glEnable(GL_DEPTH_TEST);

//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//    pangolin::CreatePanel("MENU").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
//    Var<bool> option("TESTE", true, true);
//    Var<bool> option2("TESTE2", false, true);

//    OpenGlRenderState s_cam(ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
//                            ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

//    View& d_cam = CreateDisplay().SetBounds(0.0, 1.0, Attach::Pix(175), 1.0, -1024.0f/768.0f)
//                                 .SetHandler(new Handler3D(s_cam));

//    OpenGlMatrix Twc;
//    Twc.SetIdentity();


//    while(!ShouldQuit()){
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        glClearColor(0.7f,0.7f,0.7f,1.0f);

//        pangolin::FinishFrame();
//    }





      CreateWindowAndBind("Main",640,480);

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      // 3D Mouse handler requires depth testing to be enabled
      glEnable(GL_DEPTH_TEST);

      // Issue specific OpenGl we might need
      glEnable (GL_BLEND);
      glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      // Data logger object
      DataLog log;

      // Optionally add named labels
      vector<string> labels;
      labels.push_back(string("sin(t)"));
      labels.push_back(string("cos(t)"));
      labels.push_back(string("sin(t)+cos(t)"));
      log.SetLabels(labels);

      const float tinc = 0.01f;

      // OpenGL 'view' of data. We might have many views of the same data.
      Plotter plotter(&log,0.0f,4.0f*(float)M_PI/tinc,-4.0f,4.0f,(float)M_PI/(4.0f*tinc),0.5f);
      plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
      plotter.Track("$i");

      // Add some sample annotations to the plot
      plotter.AddMarker(Marker::Vertical,   -1000, Marker::LessThan, Colour::Blue().WithAlpha(0.2f) );
      plotter.AddMarker(Marker::Horizontal,  100, Marker::GreaterThan, Colour::Red().WithAlpha(0.2f) );
      plotter.AddMarker(Marker::Horizontal,    10, Marker::Equal, Colour::Green().WithAlpha(0.2f) );

      DisplayBase().AddDisplay(plotter);

      float t = 0;

      // Default hooks for exiting (Esc) and fullscreen (tab).
      while( !ShouldQuit() )
      {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        log.Log(sin(t),cos(t),sin(t)+cos(t));
        t += tinc;

        // Render graph, Swap frames and Process Events
        FinishFrame();
      }
}
