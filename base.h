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

#include "Eigen\Dense"

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

    Arm() { }

    Arm(Point o, std::vector<Link> *link_list, float w) {
      links = link_list;
      origin = o;
      width = w;

      curr_position = o;
      for (int i = 0; i < link_list->size(); i++) {
        curr_position(1) += link_list->at(i).length;
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
      std::cout << jacobian << std::endl;
      Eigen::JacobiSVD<MatrixXf> svd (jacobian,ComputeThinU | ComputeThinV);
      
      //return svd.solve(Vector2f(error.x(), error.y()));
      return svd.solve(Vector3f(error.x(), error.y(), error.z()));
    }

    void drawPolygon(Point goal) {
      Vector3f error = goal - curr_position;
      MatrixXf matrix = getSvDPeudoInverse(error);
      std::cout << matrix << std::endl;

      Point location = origin;
      float theta = 0, phi = 0;
      for (int i = 0; i < links->size(); i++) {
        float length = links->at(i).length;
        links->at(i).current_theta += matrix(i*2,0);
        links->at(i).current_phi += matrix(i*2+1,0);
        theta += links->at(i).current_theta, phi += links->at(i).current_phi;

        Point end_point = Point(location.x() + length * cos(phi) * sin(theta), location.y() + length * cos(phi) * cos(theta), location.z() + length * sin(phi));
        Point top_origin = Point(location.x() - width * cos(theta), location.y() + width * sin(theta), location.z());
        Point bottom_origin = Point(location.x() + width * cos(theta), location.y() - width * sin(theta), location.z());
 
        glBegin(GL_POLYGON);
        glVertex3f(top_origin.x(), top_origin.y(), top_origin.z());
        glVertex3f(bottom_origin.x(), bottom_origin.y(), bottom_origin.z());
        glVertex3f(end_point.x(), end_point.y(), end_point.z());
        glEnd();

        location = end_point;
      }
      curr_position = location;
    }
};
#endif
