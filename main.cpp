#include <iostream>
#include <vector>
#include <Eigen/Dense>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

using namespace Eigen;

std::vector<Vector3f> points;
float counter = 0;

void myDisplay() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  
  GLfloat light_position[] = { 1.0, 1.0, -1.0, 0.0 };
  glLoadIdentity();
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glLoadIdentity();
  glTranslatef(points.at(counter).x(), points.at(counter).y(), points.at(counter).z());
  glutSolidSphere(0.05,70,10);

  counter += 1;
  if (counter >= points.size()) {
    counter = 0;
  }
  //glutSolidSphere(0.05, 70, 10);
  glFlush();
  glutSwapBuffers();
  glutPostRedisplay();
}

void getPoints() {
  float a = sin(3.14159/2);
  for (float i = 0; i <= 2*3.14159; i+=0.0005) {
    float x = 0.75 * cos(i);
    float y = 0.75 * sin(i);
    points.push_back(Vector3f(x,y,0));
  }
}

int main( int argc, char** argv )
{
	glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );

	glutInitWindowPosition( 0, 0 );
  glutInitWindowSize( 600, 600 );
  glutCreateWindow( "Window 1" );
	
  getPoints();
  //glutKeyboardFunc( KeyPressFunc );
	//glutSpecialFunc( SpecialKeyFunc );

	//glutReshapeFunc( ResizeWindow );
  glutDisplayFunc(myDisplay);
	
	glutMainLoop(  );
  return(0);
}