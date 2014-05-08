#include <iostream>
#include <vector>

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

#include "Eigen/Dense"
#include "base.h"

using namespace Eigen;


float PI = 3.14159f;

std::vector<Vector3f> points;
Arm arm;
Point prev_point;
float counter = 0;
GLUquadric* qobj;


void init() {
  glClearColor(0,0,0,0);
  qobj = gluNewQuadric();
  gluQuadricNormals(qobj, GLU_SMOOTH);

  /*
  GLfloat light_position[] = { 1.0, 1.0, -20.0, 1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  */
  
}

void myReshape(int w, int h) {
  glViewport (0, 0, (GLsizei) w, (GLsizei) h); // Set the viewport
  glMatrixMode (GL_PROJECTION); 	// Set the Matrix mode
  glLoadIdentity (); 
  gluPerspective(75, (GLfloat) w /(GLfloat) h , 0.10, 100.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt (1, 1, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

float angle = 0.0f;
void myDisplay() {
  // Draw the positive side of the lines x,y,z
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glBegin(GL_LINES);
  glColor3f (0.0, 1.0, 0.0); // Green for x axis
  glVertex3f(0,0,0);
  glVertex3f(1,0,0);
  glColor3f(1.0,0.0,0.0); // Red for y axis
  glVertex3f(0,0,0);
  glVertex3f(0,1,0);
  glColor3f(0.0,0.0,1.0); // Blue for z axis
  glVertex3f(0,0,0); 
  glVertex3f(0,0,1);
  glEnd();

  glPushMatrix();
  glTranslatef(points.at(counter).x(), points.at(counter).y(), points.at(counter).z());
  glutSolidSphere(0.05,70,10);
  glPopMatrix();


  arm.drawPolygon(points.at(counter));
  prev_point = points.at(counter);

  counter += 1;
  if (counter >= points.size()) {
    counter = 0;
  }

  /*glRotatef(angle,0,1,0); 
  angle += 0.4f;
  if (angle >= 360) {
    angle -= 360;
  }*/
  //gluCylinder(qobj, 0.1, 0.001, 0.5, 60, 60);
  //gluCylinder(qobj, 0.5, 0.0001, 0.4, 20,20);
  //glTranslatef(0.75,0,0);

  /*
  glLoadIdentity();
  glTranslatef(points.at(counter).x(), points.at(counter).y(), points.at(counter).z());

  glLoadIdentity();


  glLoadIdentity();
  //arm.drawPolygon(40.0f * PI/180.0f);
  //glutSolidSphere(0.05, 70, 10);
  */
  glFlush();
  glutSwapBuffers();
  glutPostRedisplay();
}

void getPoints() {
  float a = sin(3.14159/2);
  for (float i = 0; i <= 2*3.14159; i+=0.005) {
    float y = 0.75 * cos(i + 90 * PI/180);
    float z = 0.75 * sin(i + 90 * PI/180);
    points.push_back(Vector3f(0,y,z));
  }
}

int main( int argc, char** argv )
{
	glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );

	glutInitWindowPosition( 0, 0 );
  glutInitWindowSize( 600, 600 );
  glutCreateWindow( "Window 1" );
	 
  init();
  getPoints();
  
  std::vector<Link> links;
  links.push_back(Link(0.4));
  links.push_back(Link(0.4));
  arm = Arm(Point(0,0,0), &links, 0.05);
  prev_point = points.at(counter);
  counter = 1;
  //glutKeyboardFunc( KeyPressFunc );
	//glutSpecialFunc( SpecialKeyFunc );

	glutReshapeFunc(myReshape );
  glutDisplayFunc(myDisplay);
	//glutReshapeFunc(myReshape);

	glutMainLoop();
  return(0);
}
