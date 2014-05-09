#ifndef ARM_H
#define ARM_H

#include <vector>
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace Eigen;
using namespace std;

class Arm {
  public:
    Vector3f R_body_to_world;
    Vector3f R_world_to_body;
    
    // translations to the default initial location of the arm
    Vector3f X_body_to_world;
    Vector3f X_world_to_body;

    // assume length goes upward in y-axis from location of the arm
    Vector3f length;

    // Stores all the previous arms
    vector<Arm> prev_arms;

    Arm() { }
    Arm(Vector3f x, Vector3f r, float l, vector<Arm> prev_arms_arg);

    void drawArm();
    void addToR(Vector3f r);
    Vector3f getEndPoint(); // point that this arm is at
    Affine3f getLocalTransformation(); // Get the transformation that will put this arm in local perspective.

  private:
    Affine3f getAngleAxis(Vector3f v);
};

#endif