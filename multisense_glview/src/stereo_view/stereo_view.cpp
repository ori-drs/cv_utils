#include <stereo_view/stereo_view.hpp>




StereoView::StereoView(){
  // defaults
  width = 1024;
  height = 1024;
  rotateImg = false;
  doOutput = 0;
  pngFileIndex = 0;




  // commandline
  char channelName[512];
//  strcpy(channelName, "MULTISENSE_CAMERA");
  strcpy(channelName, "REALSENSE_FORWARD_CAMERA");
  char *lcm_url = NULL;


  // constructor

  lcm = lcm_create(lcm_url);

  int npixels = width*height;

  depth_img_data = (uint8_t*) malloc(npixels*3);
  uncompress_buffer = (uint8_t*) malloc(npixels*sizeof(uint16_t));
  left_img_data = (uint8_t*) malloc(npixels*3);
  right_img_data = (uint8_t*) malloc(npixels*3);
  rightpane_is_depth = 0;

  int i;
  for (i=0; i<DEPTH_VAL; i++) {
    float v = i/(float)DEPTH_VAL;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }


  char *myargv [1];
  int myargc=1;
  myargv [0]=strdup ("Myappname");
  glutInit(&myargc, myargv);

//  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  //glutInitWindowSize(width*2, height);
  glutInitWindowSize(width, height/2);
  glutInitWindowPosition(0, 0);

  window = glutCreateWindow("Multisense LCM viewer");

  currentInstance = this;

//  glutDisplayFunc(& (StereoView::DrawGLScene) );
  ::glutDisplayFunc(StereoView::DrawGLScene);

  glutIdleFunc(&DrawGLScene);

//  glutReshapeFunc(&ReSizeGLScene);
  glutReshapeFunc(StereoView::ReSizeGLScene);

  glutKeyboardFunc(StereoView::keyPressed);

  InitGL(width*2, height);


  bot_core_images_t_subscribe(lcm, channelName, on_frame, this);


  std::cout << "Finished setting up\n";


  glutMainLoop();
  free(depth_img_data);
  free(uncompress_buffer);
  free(left_img_data);
  lcm_destroy(lcm);







}




void StereoView::DrawGLSceneInternal(){

  struct pollfd pfd = { lcm_get_fileno(lcm), POLLIN, 0 };
  int status = poll (&pfd, 1, 10);
  if (status > 0) {
    lcm_handle(lcm);
  }

  pthread_mutex_lock( &mutex1 );

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, gl_leftpane_tex);
  if ((left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) ||
      (left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG )){
    glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, left_img_data);
  }else if (left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY){
    glTexImage2D(GL_TEXTURE_2D, 0, 1, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, left_img_data);
  }

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0); glVertex3f(0,0,0);
  glTexCoord2f(1, 0); glVertex3f(width,0,0);
  glTexCoord2f(1, 1); glVertex3f(width,height,0);
  glTexCoord2f(0, 1); glVertex3f(0,height,0);
  glEnd();

  glBindTexture(GL_TEXTURE_2D, gl_rightpane_tex);
  if (rightpane_is_depth){
  glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_img_data);
  } else{

    if ((right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) ||
        (right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG )){
      glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, right_img_data);
    }else if (right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY){
      glTexImage2D(GL_TEXTURE_2D, 0, 1, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, right_img_data);
    }

  }

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0); glVertex3f(width,0,0);
  glTexCoord2f(1, 0); glVertex3f(width*2,0,0);
  glTexCoord2f(1, 1); glVertex3f(width*2,height,0);
  glTexCoord2f(0, 1); glVertex3f(width,height,0);
  glEnd();

  glutSwapBuffers();

  pthread_mutex_unlock( &mutex1 );
}








void StereoView::keyPressedInternal(unsigned char key, int x, int y)
{
	if (key == 27) {
		glutDestroyWindow(window);
	}
}

void StereoView::ReSizeGLSceneInternal(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, width*2, height, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void StereoView::InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);

  if (rotateImg) {
    glMatrixMode(GL_TEXTURE);
    glTranslatef(1.0f, 0.0f, 0.0f);
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
  }

	glGenTextures(1, &gl_rightpane_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rightpane_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glGenTextures(1, &gl_leftpane_tex);
	glBindTexture(GL_TEXTURE_2D, gl_leftpane_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ReSizeGLScene(Width, Height);
}




void
StereoView::on_frame2(const bot_core_images_t* msg)
{
  png_bytep* row_pointers;
  // TODO check image width, height
  width = msg->images[0].width;
  height = msg->images[0].height;

  if(msg->images[0].size) {
    left_color_format =  msg->images[0].pixelformat;
    if(left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
      memcpy(left_img_data, msg->images[0].data, width * height * 3);
    } else if(left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG) {
      jpegijg_decompress_8u_rgb (msg->images[0].data, msg->images[0].size,
            left_img_data, width, height, width * 3);
    }else if (left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
      memcpy(left_img_data, msg->images[0].data, width * height);
    }else{
      printf("First Image Format Not Understood\n");
    }
  }

  if( msg->image_types[1] == BOT_CORE_IMAGES_T_RIGHT ){
    rightpane_is_depth = 0;

    right_color_format =  msg->images[1].pixelformat;
    if(right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
      memcpy(right_img_data, msg->images[1].data, width * height * 3);
    } else if(right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG) {
      jpegijg_decompress_8u_rgb (msg->images[1].data, msg->images[1].size,
            right_img_data, width, height, width * 3);
    }else if (right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
      memcpy(right_img_data, msg->images[1].data, width * height);
    }else{
      printf("Second Image Format Not Understood\n");
    }

  }else if ( (msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_M || msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_M_ZIPPED) ||
             ( (msg->image_types[1] == BOT_CORE_IMAGES_T_DISPARITY || msg->image_types[1] == BOT_CORE_IMAGES_T_DISPARITY_ZIPPED) ||
               (msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_MM  || msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_MM_ZIPPED) )){
    rightpane_is_depth = 1;

    int i;
    const uint16_t* depth = NULL;
    const short* depth_short = NULL;
    const float* depth_float = NULL;
    int scaling=2;
    bool disparity_values = false;
    bool depth_m_values = false;
    bool depth_mm_values = false;
    if(msg->image_types[1] == BOT_CORE_IMAGES_T_DISPARITY) {
      depth = (uint16_t*) msg->images[1].data;
      disparity_values = true;
    }else if (msg->image_types[1] == BOT_CORE_IMAGES_T_DISPARITY_ZIPPED ) {
      unsigned long dlen = width*height*2;
      uncompress(uncompress_buffer, &dlen, msg->images[1].data, msg->images[1].size);
      depth = (uint16_t*) uncompress_buffer;
      disparity_values = true;
    }else if (msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_MM ) {
      depth_short = (short*) msg->images[1].data;
      depth_mm_values = true;
    }else if (msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_MM_ZIPPED ) {
      unsigned long dlen = width*height*2;
      uncompress(uncompress_buffer, &dlen, msg->images[1].data, msg->images[1].size);
      depth_short = (short*) uncompress_buffer;
      depth_mm_values = true;
    }else if (msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_M ) {
      depth_float = (float*) msg->images[1].data;
      depth_m_values = true;
    }else if (msg->image_types[1] == BOT_CORE_IMAGES_T_DEPTH_M_ZIPPED ) {
      unsigned long dlen = width*height*4;
      uncompress(uncompress_buffer, &dlen, msg->images[1].data, msg->images[1].size);
      depth_float = (float*) uncompress_buffer;
      depth_m_values = true;
    }else{
      printf("Second Image Format Not Understood [B]\n");
    }

    pthread_mutex_lock( &mutex1 );
    int npixels = width * height;

    for (i=0; i<npixels; i++) {

      int d = 0;
      // Invert the disparity values and scale to reasonable range
      if (disparity_values){
        if (depth[i]==0){
          d=0;
        }else{
          d=round(600000.0/ (float)depth[i]) ;
        }
      }else if(depth_mm_values){
        if (depth_short[i]==0){
          d=0;
        }else{
          d=round(depth_short[i]);
        }
      }else if(depth_m_values){
        if (depth_float[i]==0){
          d=0;
        }else{
          d=round((float)2000*depth_float[i]);
        }
      }

      if ( scaling*d >= DEPTH_VAL ) {
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 0;
        depth_img_data[3*i+2] = 0;
        continue;
      }

      int pval = t_gamma[scaling*d];
      int lb = pval & 0xff;
      switch (pval>>8) {
      case 0:
        depth_img_data[3*i+0] = 255;
        depth_img_data[3*i+1] = 255-lb;
        depth_img_data[3*i+2] = 255-lb;
        break;
            case 1:
        depth_img_data[3*i+0] = 255;
        depth_img_data[3*i+1] = lb;
        depth_img_data[3*i+2] = 0;
        break;
            case 2:
        depth_img_data[3*i+0] = 255-lb;
        depth_img_data[3*i+1] = 255;
        depth_img_data[3*i+2] = 0;
        break;
            case 3:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 255;
        depth_img_data[3*i+2] = lb;
        break;
            case 4:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 255-lb;
        depth_img_data[3*i+2] = 255;
        break;
            case 5:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 0;
        depth_img_data[3*i+2] = 255-lb;
        break;
            default:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 0;
        depth_img_data[3*i+2] = 0;
        break;
      }
    }

    row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
    for ( i = 0; i < height; i++ ) {
      row_pointers[i] = (png_bytep) malloc(3*width*2);
      memcpy(row_pointers[i] + 0*3*width, depth_img_data + i*3*width, width*3);
      memcpy(row_pointers[i] + 1*3*width, depth_img_data + i*3*width, width*3);
    }
    if ( doOutput ) {
      char filename[1024];
      sprintf(filename, "%s/img%08i.png", outputPath, pngFileIndex++);
      write_png_file(filename, width*2, height, row_pointers);
    }

    for ( i = 0; i < height; i++ ) {
      free(row_pointers[i]);
    }
    free(row_pointers);

    pthread_mutex_unlock( &mutex1 );
  }else{
      printf("Second Image Format Not Understood [C]\n");
  }
}



void abort_(const char * s, ...)
{
        va_list args;
        va_start(args, s);
        vfprintf(stderr, s, args);
        fprintf(stderr, "\n");
        va_end(args);
        abort();
}


void StereoView::write_png_file(char* file_name, int width, int height, png_bytep* row_pointers )
{
  png_byte color_type = PNG_COLOR_TYPE_RGB;
  png_byte bit_depth = 8;

  /* create file */
  FILE *fp = fopen(file_name, "wb");
  if (!fp)
    abort_("[write_png_file] File %s could not be opened for writing", file_name);


  /* initialize stuff */
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!png_ptr)
    abort_("[write_png_file] png_create_write_struct failed");

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr)
    abort_("[write_png_file] png_create_info_struct failed");

  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during init_io");

  png_init_io(png_ptr, fp);


  /* write header */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during writing header");

  png_set_IHDR(png_ptr, info_ptr, width, height,
	       bit_depth, color_type, PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);


  /* write bytes */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during writing bytes");

  png_write_image(png_ptr, row_pointers);


  /* end write */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during end of write");

  png_write_end(png_ptr, NULL);

  /*
  for (y=0; y<height; y++)
    free(row_pointers[y]);
  free(row_pointers);
*/

  fclose(fp);
}




