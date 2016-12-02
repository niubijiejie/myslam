/*#include <limits>
#include <iostream>
#include <pangolin/pangolin.h>

void setImageData(unsigned char * imageArray, int size){
  for(int i = 0 ; i < size;i++) {
    imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
  }
}

int main( int /*argc*//*, char* argv[] )
{
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("Main",640,480);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
    pangolin::ModelViewLookAt(-1,1,-1, 0,0,0, pangolin::AxisY)
  );

  // Aspect ratio allows us to constrain width and height whilst fitting within specified
  // bounds. A positive aspect ratio makes a view 'shrink to fit' (introducing empty bars),
  // whilst a negative ratio makes the view 'grow to fit' (cropping the view).
  pangolin::View& d_cam = pangolin::Display("cam")
      .SetBounds(0,1.0f,0,1.0f,-640/480.0)
      .SetHandler(new pangolin::Handler3D(s_cam));

  // This view will take up no more than a third of the windows width or height, and it
  // will have a fixed aspect ratio to match the image that it will display. When fitting
  // within the specified bounds, push to the top-left (as specified by SetLock).
  pangolin::View& d_image = pangolin::Display("image")
      .SetBounds(2/3.0f,1.0f,0,1/3.0f,640.0/480)
      .SetLock(pangolin::LockLeft, pangolin::LockTop);

  std::cout << "Resize the window to experiment with SetBounds, SetLock and SetAspect." << std::endl;
  std::cout << "Notice that the cubes aspect is maintained even though it covers the whole screen." << std::endl;

  const int width =  64;
  const int height = 48;

  unsigned char* imageArray = new unsigned char[3*width*height];
  pangolin::GlTexture imageTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while(!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);

    glColor3f(1.0,1.0,1.0);
    pangolin::glDrawColouredCube();

    //Set some random image data and upload to GPU
    setImageData(imageArray,3*width*height);
    imageTexture.Upload(imageArray,GL_RGB,GL_UNSIGNED_BYTE);

    //display the image
    d_image.Activate();
    glColor3f(1.0,1.0,1.0);
    imageTexture.RenderToViewport();

    pangolin::FinishFrame();
  }

  delete[] imageArray;

  return 0;
}*/
#include <iostream>
#include <pangolin/pangolin.h>

/*void setImageData(unsigned char * imageArray, int size){
  for(int i = 0 ; i < size;i++) {
    imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
  }
}

int main( int /*argc*//*, /*char* argv[] )
{
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("Main",640,480);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000);
  pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(1,0.5,-2,0,0,0, pangolin::AxisY) );
  pangolin::OpenGlRenderState s_cam2(proj, pangolin::ModelViewLookAt(0,0,-2,0,0,0, pangolin::AxisY) );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam1 = pangolin::Display("cam1")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::View& d_cam2 = pangolin::Display("cam2")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam2));

  pangolin::View& d_cam3 = pangolin::Display("cam3")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::View& d_cam4 = pangolin::Display("cam4")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam2));

  pangolin::View& d_img1 = pangolin::Display("img1")
    .SetAspect(640.0f/480.0f);

  pangolin::View& d_img2 = pangolin::Display("img2")
    .SetAspect(640.0f/480.0f);

  // LayoutEqual is an EXPERIMENTAL feature - it requires that all sub-displays
  // share the same aspect ratio, placing them in a raster fasion in the
  // viewport so as to maximise display size.
  pangolin::Display("multi")
      .SetBounds(1.0, 0.0, 0.0, 1.0)
      .SetLayout(pangolin::LayoutEqual)
      .AddDisplay(d_cam1)
      .AddDisplay(d_img1)
      .AddDisplay(d_cam2)
      .AddDisplay(d_img2)
      .AddDisplay(d_cam3)
      .AddDisplay(d_cam4);

  const int width =  64;
  const int height = 48;
  unsigned char* imageArray = new unsigned char[3*width*height];
  pangolin::GlTexture imageTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() )
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Generate random image and place in texture memory for display
    setImageData(imageArray,3*width*height);
    imageTexture.Upload(imageArray,GL_RGB,GL_UNSIGNED_BYTE);

    glColor3f(1.0,1.0,1.0);

    d_cam1.Activate(s_cam);
    pangolin::glDrawColouredCube();

    d_cam2.Activate(s_cam2);
    pangolin::glDrawColouredCube();

    d_cam3.Activate(s_cam);
    pangolin::glDrawColouredCube();

    d_cam4.Activate(s_cam2);
    pangolin::glDrawColouredCube();

    d_img1.Activate();
    glColor4f(1.0f,1.0f,1.0f,1.0f);
    imageTexture.RenderToViewport();

    d_img2.Activate();
    glColor4f(1.0f,1.0f,1.0f,1.0f);
    imageTexture.RenderToViewport();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  delete[] imageArray;

  return 0;
}*/

void setImageData(unsigned char * imageArray, int size)
{
    for(int i=0;i<size;i++)
    {
        imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
    }
}

int main(int /*argc*/, char** /*argv*/)
{

    pangolin::CreateWindowAndBind("Main", 640, 480);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000);
    pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(1,0.5,-2,0,0,0, pangolin::AxisY));
    pangolin::OpenGlRenderState s_cam2(proj, pangolin::ModelViewLookAt(0,0,-2,0,0,0, pangolin::AxisY));

    pangolin::View & d_cam1 = pangolin::Display("cam1")
            .SetAspect(640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View & d_cam2 = pangolin::Display("cam2")
            .SetAspect(640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam2));
    pangolin::View & d_cam3 = pangolin::Display("cam3")
            .SetAspect(640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View & d_cam4 = pangolin::Display("cam4")
            .SetAspect(640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam2));

    pangolin::View& d_img1 = pangolin::Display("img1")
      .SetAspect(640.0f/480.0f);

    pangolin::View& d_img2 = pangolin::Display("img2")
      .SetAspect(640.0f/480.0f);

    pangolin:: Display("multi")
            .SetBounds(1.0, 0.0, 0.0, 1.0)
            .SetLayout(pangolin::LayoutEqual)
            .AddDisplay(d_cam1)
            .AddDisplay(d_img1)
            .AddDisplay(d_cam2)
            .AddDisplay(d_img2)
            .AddDisplay(d_cam3)
            .AddDisplay(d_cam4);

    const int width =  64;
    const int height = 48;

    unsigned char* imageArray=new unsigned char[3*width*height];
    pangolin::GlTexture imageTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setImageData(imageArray, 3*width*height);
        imageTexture.Upload(imageArray,GL_RGB,GL_UNSIGNED_BYTE);

        glColor3f(1.0,1.0,1.0);

        d_cam1.Activate(s_cam);
        pangolin::glDrawColouredCube();

        d_cam2.Activate(s_cam2);
        pangolin::glDrawColouredCube();

        d_cam3.Activate(s_cam);
        pangolin::glDrawColouredCube();

        d_cam4.Activate(s_cam2);
        pangolin::glDrawColouredCube();

        d_img1.Activate(s_cam);
        glColor4f(1.0f,1.0f,1.0f,1.0f);
        imageTexture.RenderToViewport();

        d_img2.Activate();
        glColor4f(1.0f,1.0f,1.0f,1.0f);
        imageTexture.RenderToViewport();




        // Swap frames and Process Events
        pangolin::FinishFrame();

    }

    delete[] imageArray;
    return 0;
}
