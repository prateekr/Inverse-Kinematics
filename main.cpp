#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "arm.h"

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

#define NUM_ARMS 2
#define STEP_SIZE 0.01
#define ERROR_MARGIN 0.1
#define ZERO_MARGIN 0.00001

using namespace Eigen;
using namespace std;

vector<Vector3f> points;
float counter = 0;

vector<Arm> arms;
vector<Affine3f> Rs; // vector of the summation of rotations of R determined from the previous arms
vector<Affine3f> Xs; // transformations determined from the previous arms
vector<Matrix3f> Js; // Jacobians for each corresponding arm

void init() {
  // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  // glMatrixMode(GL_PROJECTION);
  // glLoadIdentity();
  // glOrtho(-1, 1, -1, 1, -100, 100);

  // glMatrixMode(GL_MODELVIEW);
  // glLoadIdentity();

  // GLfloat light_position[] = { 1.0, 1.0, -1.0, 0.0 };
  // glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  // glEnable(GL_LIGHTING);
  // glEnable(GL_LIGHT0);

  // Initialize the data structures for the arms
  // Simple case with 1 arm
  // arms = new vector<Arm>();
  // Rs = new vector<Affine3f>();
  // Xs = new vector<Affine3f>();
  // Js = new vector<Matrix3f>();
  Rs.push_back(Affine3f::Identity());
  Rs.push_back(Affine3f::Identity());

  Xs.push_back(Affine3f::Identity());
  Xs.push_back(Affine3f::Identity());
  
  Matrix3f tmp2 = Matrix3f::Zero();
  Js.push_back(tmp2);
  Js.push_back(tmp2);

  arms.push_back(Arm(Vector3f(0,0,0), Vector3f(0,0,0), 10));
  arms.push_back(Arm(Vector3f(0,10,0), Vector3f(0,0,0), 5));
}

void myDisplay() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glLoadIdentity();
  glTranslatef(points.at(counter).x(), points.at(counter).y(), points.at(counter).z());
  glutSolidSphere(0.1,70,10);

  counter += 5;
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

//****************************************************
// Inverse Kinematics
//****************************************************

Affine3f getAngleAxis(Vector3f v) {
  if (v == Vector3f(0,0,0)) {
    return Affine3f::Identity();
  }

  float mag = v.norm(); 
  v.normalize();
  return (Affine3f) AngleAxisf(mag, v);
}

Matrix3f getCrossProductMatrix(Vector3f v) {
  Matrix3f m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

// Given world coordinate of p, get local p with respect to the number of arms
Vector3f getLocalP(Vector3f p) {
  Affine3f m = Affine3f::Identity();
  for (int i = (arms.size() - 2); i >= 0; i--) {
    m = m * getAngleAxis(arms[i].R_world_to_body) * Translation3f(arms[i].X_world_to_body);
  }
  return m * p;
}

void updateRs() {
  // By default, the first entry of Rs is the identity
  Affine3f m;
  for (int i = 1; i < Rs.size(); i++) {
    m = Rs[i-1] * getAngleAxis(arms[i-1].R_body_to_world);
    Rs[i] = m;
  }
}

void updateXs() {
  Affine3f m;
  for (int i = (Xs.size() - 2); i >= 0; i--) {
    m = Matrix3f::Identity(3,3);
    for (int j = i; j < arms.size(); j++) {
      m = m * Translation3f(arms[j].X_body_to_world);
    }
    Xs[i] = m;
  }
}

void updateJacobian(Vector3f localP) {
  // cout << "-------" << endl;
  for (int i = 0; i < Js.size(); i++) {
    // cout << Rs[i].affine() << endl;   
    Js[i] = Rs[i] * getCrossProductMatrix(-(Xs[i] * localP));
  }
}

void updateArms(Vector3f p) {
  int count = 10;

  while ((count > 0) && (ERROR_MARGIN < (p - arms[arms.size() - 1].getPoint()).norm())) {
    cout << "====" << count << endl;
    cout << arms[arms.size() - 1].getPoint() << endl;
    // cout << "====\n";
    // cout << (p - arms[0].getPoint()) << endl;
    cout << "====\n";
    
    Vector3f localP = getLocalP(p);
    updateRs();
    updateXs();
    updateJacobian(localP);

    // Generate and get the jacobian
    Matrix<float, 3, NUM_ARMS * 3> jacobian;
    for (int i = (Js.size() - 1); i >= 0; i--) {
      jacobian.block(0,i*3,3,3) = Js[i];
    }

    // cout << jacobian << endl;

    // Get the pseudo-inverse
    Matrix<float, NUM_ARMS * 3, 3> jacobian_inverse;
    JacobiSVD<MatrixXf> svd(jacobian, ComputeThinU | ComputeThinV);

    VectorXf s_vals = svd.singularValues();
    // cout << s_vals << endl;

    for (int i = 0; i < s_vals.size(); i++) {
      if (abs(s_vals(i)) > ZERO_MARGIN) {
        s_vals(i) = 1 / s_vals(i);
      }
      else {
        s_vals(i) = 0;
      }
    }

    cout << jacobian << endl;
    // cout << s_vals << endl;

    MatrixXf d = DiagonalMatrix<float, Dynamic, Dynamic>(s_vals);

    // cout << svd.matrixV() << endl;
    // cout << svd.matrixU() << endl;
    // cout << d << endl;

    jacobian_inverse = svd.matrixV() * d * svd.matrixU().transpose();

    cout << jacobian_inverse << endl;

    Matrix<float, NUM_ARMS * 3, 1> dd;
    dd = jacobian_inverse * (p - arms[arms.size() - 1].getPoint());

    // cout << dd << endl;
    for (int i = 0; i < arms.size(); i++) {
      arms[i].addToR(STEP_SIZE * Vector3f(dd[i*3], dd[(i*3)+1], dd[(i*3)+2]));
    }
    
    count--;
  }
}

int main( int argc, char** argv )
{
	// glutInit(&argc,argv);
 //  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );

	// glutInitWindowPosition( 0, 0 );
 //  glutInitWindowSize( 600, 600 );
 //  glutCreateWindow( "Window 1" );
	 
  init();
 //  getPoints();
 //  //glutKeyboardFunc( KeyPressFunc );
	// //glutSpecialFunc( SpecialKeyFunc );

	// //glutReshapeFunc( ResizeWindow );
 //  glutDisplayFunc(myDisplay);
	
	// glutMainLoop();

  updateArms(Vector3f(1, 14.5, 0));

  // Vector3f a(2,2,0);
  // Vector3f b(1,0,0);
  // // cout << a*b.transpose() << endl;
  // Matrix3f m;
  // Affine3f m2;  
  // AngleAxisf aa = AngleAxisf(3.14, a);
  // Translation3f tt = Translation3f(Vector3f(0,0,1));
  // m = aa;
  // m2 = m * tt;

  // cout << m2.affine() << endl;
  // cout << m2 * b << endl;

  // Matrix3f a;
  // a << 1,2,3,4,5,6,7,8,9;
  // Matrix<float, 3, 6> m;

  // m.block(0,0,3,3) = a;
  // cout << m << endl;

  // Matrix3f a = Matrix3f::Random(3,3);
  // JacobiSVD<MatrixXf> svd(a, ComputeThinU | ComputeThinV);
  // // cout << svd.matrixU() << endl;
  // // cout << svd.matrixV() << endl;
  // Matrix3f x = Matrix3f::Identity(3,3);
  // Vector3f tmp = svd.singularValues();
  // // MatrixXf t = x * svd.singularValues();
  // cout << tmp << endl;  
  // for (int i = 0; i < tmp.size(); i++) {
  //   if (tmp(i) != 0) {
  //     tmp(i) = 1 / tmp(i);
  //   }
  // }
  // cout << tmp << endl;
  // MatrixXf d = DiagonalMatrix<float, Dynamic, Dynamic>(tmp);
  // cout << d << endl;
  
  // cout << svd.matrixV() * d * svd.matrixU().transpose() << endl;
  // cout << 1 / (svd.singularValues()) << endl;
  
  
  
  return(0);
}
