#ifndef BASE_H
#define BASE_H

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

using namespace Eigen;

typedef Vector3f Point;

class Link {
  public:
    float length;
    float current_theta, current_phi;
    
    Link(float l) {
      length = l;
      current_theta = 0;
      current_phi = 0;
    }
};

class Arm {
  public:
    std::vector<Link> *links;
    Point origin, curr_position;
    float width;
    float total_length;

    Arm() { }

    Arm(Point o, std::vector<Link> *link_list, float w) {
      links = link_list;
      origin = o;
      width = w;

      total_length = 0;
      curr_position = o;
      for (int i = 0; i < link_list->size(); i++) {
        curr_position(1) += link_list->at(i).length;
        total_length += link_list->at(i).length;
      }
    }

    MatrixXf getJacobian() {
      MatrixXf jacobian(3, links->size() * 2);
      float theta_sum = 0, phi_sum = 0;
      std::vector<float> theta_sums, phi_sums;

      //Holds partial sums, theta1, theta1+theta2,...
      for (int i = 0; i < links->size(); i++) {
        theta_sum += links->at(i).current_theta, phi_sum += links->at(i).current_phi;
        theta_sums.push_back(theta_sum);
        phi_sums.push_back(phi_sum);
      }

      for (int i = 0; i < links->size(); i++) {
        Link link = links->at(i);
        jacobian(0, i*2) = 0, jacobian(0, i*2 + 1) = 0;
        jacobian(1, i*2) = 0, jacobian(1, i*2 + 1) = 0;
        jacobian(2, i*2) = 0, jacobian(2, i*2 + 1) = 0;
        for (int j = i; j < links->size(); j++) {
          jacobian(0, i*2) += links->at(j).length * cos(phi_sums.at(j)) * cos(theta_sums.at(j));
          jacobian(0, i*2+1) += -links->at(j).length * sin(phi_sums.at(j)) * sin(theta_sums.at(j));
          jacobian(1, i*2) += -links->at(j).length * cos(phi_sums.at(j)) * sin(theta_sums.at(j)); 
          jacobian(1, i*2+1) += -links->at(j).length * sin(phi_sums.at(j)) * cos(theta_sums.at(j)); 
          jacobian(2, i) += 0;
          jacobian(2, i*2+1) += links->at(j).length * cos(phi_sums.at(j));
        }
      }
      return jacobian;
    }

    MatrixXf getSvDPeudoInverse(Vector3f error) {
      MatrixXf jacobian = getJacobian();
      Eigen::JacobiSVD<MatrixXf> svd (jacobian,ComputeThinU | ComputeThinV);
      
      //return svd.solve(Vector2f(error.x(), error.y()));
      return svd.solve(Vector3f(error.x(), error.y(), error.z()));
    }

    void drawPolygon(Point goal) {
      if ((goal - origin).norm() > total_length - 0.0005) {
        goal = goal * (total_length)/goal.norm();
      }
      
      Vector3f error = goal - curr_position;
      MatrixXf matrix = getSvDPeudoInverse(error);

      Point location = origin;
      float theta = 0, phi = 0;
      for (int i = 0; i < links->size(); i++) {
        float length = links->at(i).length;
        links->at(i).current_theta += matrix(i*2,0);
        links->at(i).current_phi += matrix(i*2+1,0);
        theta += links->at(i).current_theta, phi += links->at(i).current_phi;

        Point end_point = Point(location.x() + length * cos(phi) * sin(theta), location.y() + length * cos(phi) * cos(theta), location.z() + length * sin(phi));
        // Point top_origin = Point(location.x() - width * cos(theta), location.y() + width * sin(theta), location.z());
        // Point bottom_origin = Point(location.x() + width * cos(theta), location.y() - width * sin(theta), location.z());
        Point A = Point(location.x() - width * cos(theta), location.y() + width * sin(theta), location.z());
        Point C = Point(location.x() + width * cos(theta), location.y() - width * sin(theta), location.z());

        Vector3f diff = A - C;
        float d = diff.cwiseAbs().sum();
        Point D = Point(location.x(), location.y(), location.z() - (d / 2));
        Point B = Point(location.x(), location.y(), location.z() + (d / 2));

        glPushMatrix();
        glColor3f(0.5f, 0.0f, 0.5f);
        glTranslatef(location.x(), location.y(), location.z());
        glutSolidSphere(width,30,10);
        glPopMatrix();

        // top_origin -  --  - - - - - end
        //     |                      |
        // bottom_origin - - - - - - (bottom_origin - location) + end

        Point A_prime = (A - location) + end_point;
        Point B_prime = (B - location) + end_point;
        Point C_prime = (C - location) + end_point;
        Point D_prime = (D - location) + end_point;

        Vector3f n;

        glPushMatrix();
        glBegin(GL_QUADS);

        glColor3f(0.5f, 0.5f, 0.0f);
        // Bottom face
        n = (D - A).cross(B - A);
        n.normalize();
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(D.x(), D.y(), D.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(C.x(), C.y(), C.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(B.x(), B.y(), B.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(A.x(), A.y(), A.z());

        // Top face
        n = (B_prime - A_prime).cross(D_prime - A_prime);
        n.normalize();
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(A_prime.x(), A_prime.y(), A_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(B_prime.x(), B_prime.y(), B_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(C_prime.x(), C_prime.y(), C_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(D_prime.x(), D_prime.y(), D_prime.z());

        // Long faces
        n = (B - A).cross(A_prime - A);
        n.normalize();
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(A.x(), A.y(), A.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(A_prime.x(), A_prime.y(), A_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(B_prime.x(), B_prime.y(), B_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(B.x(), B.y(), B.z());
        

        n = (C - B).cross(B_prime - B);
        n.normalize();
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(B.x(), B.y(), B.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(B_prime.x(), B_prime.y(), B_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(C_prime.x(), C_prime.y(), C_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(C.x(), C.y(), C.z());
        
        n = (D - C).cross(C_prime - C);
        n.normalize();
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(C.x(), C.y(), C.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(C_prime.x(), C_prime.y(), C_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(D_prime.x(), D_prime.y(), D_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(D.x(), D.y(), D.z());

        n = (A - D).cross(D_prime - D);
        n.normalize();
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(D.x(), D.y(), D.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(D_prime.x(), D_prime.y(), D_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(A_prime.x(), A_prime.y(), A_prime.z());
        glNormal3f(n.x(), n.y(), n.z());
        glVertex3f(A.x(), A.y(), A.z());
        
        glEnd();
        glPopMatrix();

        // glBegin(GL_TRIANGLES);
        // glVertex3f(top_origin.x(), top_origin.y(), top_origin.z());
        // glVertex3f(bottom_origin.x(), bottom_origin.y(), bottom_origin.z());
        // glVertex3f(end_point.x(), end_point.y(), end_point.z());
        // glEnd();

        location = end_point;
      }
      curr_position = location;
    }
};
#endif
