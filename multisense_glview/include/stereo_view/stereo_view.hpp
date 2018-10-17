#include <iostream>
#include <fstream>      // std::ifstream
#include <stdio.h>

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <poll.h>

#include <zlib.h>
#include <glib.h>

#include <lcm/lcm.h>
#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/bot_core_images_t.h>
#include <pthread.h>
#include <png.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <stdbool.h>
#include <math.h>

#include "jpeg_utils/jpeg-utils-ijg.hpp"

#define DEPTH_VAL 8192


class StereoView;
static StereoView* currentInstance;




class StereoView{
  public:

    StereoView();
    
    ~StereoView(){
    }    



    // Explicit declaration
    void write_png_file(char* file_name, int width, int height, png_bytep* row_pointers);

    static void DrawGLScene(){ currentInstance->DrawGLSceneInternal(); }
    static void ReSizeGLScene(int Width, int Height){ currentInstance->ReSizeGLSceneInternal(Width, Height); }
    static void keyPressed(unsigned char key, int x, int y){ currentInstance->keyPressedInternal(key, x, y); }

    void DrawGLSceneInternal();
    void keyPressedInternal(unsigned char key, int x, int y);
    void ReSizeGLSceneInternal(int Width, int Height);
    void keyPressedInternal(int Width, int Height);

    void InitGL(int Width, int Height);

static void on_frame(const lcm_recv_buf_t *rbuf __attribute__((unused)),
    const char * channel __attribute__((unused)), const bot_core_images_t * msg,
    void * user)
{
  StereoView * app = (StereoView *) user;
  app->on_frame2(msg);
}

    void on_frame2(const bot_core_images_t* msg);

  private:


int window;

// 800x800 - vrc
// 1024x544 - drc (real sensor)
// 1024x1024 - extended HFOV real sensor
int width;
int height;

bool rotateImg;

GLuint gl_leftpane_tex;
int32_t left_color_format;
uint8_t* left_img_data;


GLuint gl_rightpane_tex;
int rightpane_is_depth; // false = depth | true = color

int32_t right_color_format;
uint8_t* right_img_data;

uint8_t* depth_img_data;
uint8_t* uncompress_buffer;


lcm_t* lcm;// = NULL;

int doOutput;
char outputPath[1024];

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

int pngFileIndex;

uint16_t t_gamma[DEPTH_VAL];

};
