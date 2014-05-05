#ifndef ARM_H
#define ARM_H

#include "Eigen/Dense"

using namespace Eigen;

class Arm {
  public:
    Vector3f R_body_to_world;
    Vector3f R_world_to_body;
    
    // translations to the default initial location of the arm
    Vector3f X_body_to_world;
    Vector3f X_world_to_body;

    // assume length goes upward in y-axis from location of the arm
    float length;

    Arm() { }
    Arm(Vector3f x, Vector3f r, float l);

    void drawArm();
};

#endif