// ----------------------------------------------
// Informatique Graphique 3D & Réalité Virtuelle.
// Travaux Pratiques
// Shaders
// Copyright (C) 2015 Tamy Boubekeur
// All rights reserved.
// ----------------------------------------------

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/glew.h>
#endif

#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cmath>

#include "Vec3.h"
#include "Camera.h"
#include "Mesh.h"

using namespace std;

static const unsigned int DEFAULT_SCREENWIDTH = 1024;
static const unsigned int DEFAULT_SCREENHEIGHT = 768;
static const string DEFAULT_MESH_FILE ("../bunny.obj");

static string appTitle ("Informatique Graphique & Realite Virtuelle - Travaux Pratiques - Shaders");
static GLint window;
static unsigned int FPS = 0;
static bool fullScreen = false;

static Camera camera;
static Mesh mesh;

// lorsque laplacianMode == false && selectMode == true, on définit la région d'intérêt
// et si selectMode == false, on définira le handle
// si laplacianMode == true, on applique la transformation chaque fois on clique sur l'écran
bool selectMode;
bool laplacianMode;

void printUsage () {
  std::cerr << std::endl 
	    << appTitle << std::endl
	    << "Author: Tamy Boubekeur" << std::endl << std::endl
	    << "Usage: ./main [<file.off>]" << std::endl
	    << "Commands:" << std::endl 
	    << "------------------" << std::endl
	    << " ?: Print help" << std::endl
	    << " w: Toggle wireframe mode" << std::endl
	    << " <drag>+<left button>: rotate model" << std::endl 
	    << " <drag>+<right button>: move model" << std::endl
	    << " <drag>+<middle button>: zoom" << std::endl
	    << " q, <esc>: Quit" << std::endl << std::endl; 
}

void init (const char * modelFilename) {
  #ifndef __APPLE__
  glewInit();
  #endif
  glCullFace (GL_BACK);     // Specifies the faces to cull (here the ones pointing away from the camera)
  glEnable (GL_CULL_FACE); // Enables face culling (based on the orientation defined by the CW/CCW enumeration).
  glDepthFunc (GL_LESS); // Specify the depth test for the z-buffer
  glEnable (GL_DEPTH_TEST); // Enable the z-buffer in the rasterization
  glLineWidth (2.0); // Set the width of edges in GL_LINE polygon mode
  glClearColor (0.0f, 0.0f, 0.0f, 1.0f); // Background color
  glClearColor (0.0f, 0.0f, 0.0f, 1.0f);
	
  // add some lighting
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  GLfloat lightPosition[] = {0.0, 1.0, 1.0, 0.0};   // w=0.0
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

  glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
glEnable ( GL_COLOR_MATERIAL );
  camera.resize (DEFAULT_SCREENWIDTH, DEFAULT_SCREENHEIGHT); // Setup the camera
  mesh.loadOBJ (modelFilename); // Load a mesh file

  selectMode = true;
  laplacianMode = false;
}

void drawScene () {
  glBegin (GL_TRIANGLES);
  Vertex hv = Vertex(Vec3f(0,0,0),Vec3f(0,0,0));
  for (unsigned int i = 0; i < mesh.T.size (); i++) 
    for (unsigned int j = 0; j < 3; j++) {
      const Vertex & v = mesh.V[mesh.T[i].v[j]];
      if(v.isHandle){
        glColor3f(1,0,0);
      }
      else if(v.isSelected){
        glColor3f(0,1,0);
      }
      else{
        glColor3f(1,1,1);
      }
      glNormal3f (v.n[0], v.n[1], v.n[2]); // Specifies current normal vertex   
      glVertex3f (v.p[0], v.p[1], v.p[2]); // Emit a vertex (one triangle is emitted each time 3 vertices are emitted)
    }

   glEnd (); 
}

void reshape(int w, int h) {
  camera.resize (w, h);
}

void display () {
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  camera.apply (); 
  drawScene ();
  glFlush ();
  glutSwapBuffers (); 
}

Vec3f screenTo3D(int x, int y){
  GLdouble modelview[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

  GLdouble projection[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection);

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);

  y = viewport[3] - y;
  float z;
  glReadPixels(x,y,1,1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);

  GLdouble objx,objy,objz;
  gluUnProject(x, y, z, modelview, projection, viewport, &objx,&objy,&objz);
  cout << "[Debug] Click point in the model: (" << objx << "," << objy << "," << objz<<")" << endl;
  if(!laplacianMode){
    mesh.selectPart(Vec3f(objx,objy,objz),0.1,selectMode);  
  }
  else{
    mesh.laplacianTransform(Vec3f(objx,objy,objz));
  }
  
}
void key (unsigned char keyPressed, int x, int y) {
  switch (keyPressed) {
  case 'f':
    if (fullScreen) {
      glutReshapeWindow (camera.getScreenWidth (), camera.getScreenHeight ());
      fullScreen = false;
    } else {
      glutFullScreen ();
      fullScreen = true;
    }      
    break;
  case 'q':
  case 27:
    exit (0);
    break;
  case 's':
    cout << "Selection Mode" << endl;
    selectMode = true;
    laplacianMode = false;
    break;
  case 'h':
    cout << "Handle mode" << endl;
    selectMode = false;
    laplacianMode = false;
    break;
  case 'l':
    laplacianMode = true;
    break;
  case 'w':
    GLint mode[2];
    glGetIntegerv (GL_POLYGON_MODE, mode);
    glPolygonMode (GL_FRONT_AND_BACK, mode[1] ==  GL_FILL ? GL_LINE : GL_FILL);
    break;
    break;
  default:
    printUsage ();
    break;
  }
}

void mouse (int button, int state, int x, int y) {
  camera.handleMouseClickEvent (button, state, x, y);
  if(state == GLUT_DOWN){
    screenTo3D(x,y);
  }
  
  
}

void motion (int x, int y) {
  camera.handleMouseMoveEvent (x, y);

}

void idle () {
  static float lastTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
  static unsigned int counter = 0;
  counter++;
  float currentTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
  if (currentTime - lastTime >= 1000.0f) {
    FPS = counter;
    counter = 0;
    static char winTitle [128];
    unsigned int numOfTriangles = mesh.T.size ();
    sprintf (winTitle, "Number Of Triangles: %d - FPS: %d", numOfTriangles, FPS);
    glutSetWindowTitle (winTitle);
    lastTime = currentTime;
  }
  glutPostRedisplay (); 
}

int main (int argc, char ** argv) {
  if (argc > 2) {
    printUsage ();
    exit (1);
  }
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize (DEFAULT_SCREENWIDTH, DEFAULT_SCREENHEIGHT);
  window = glutCreateWindow (appTitle.c_str ());
  init (argc == 2 ? argv[1] : DEFAULT_MESH_FILE.c_str ());
  glutIdleFunc (idle);
  glutReshapeFunc (reshape);
  glutDisplayFunc (display);
  glutKeyboardFunc (key);
  glutMotionFunc (motion);
  glutMouseFunc (mouse);
  printUsage ();  
  glutMainLoop ();
  return 0;
}

