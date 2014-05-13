#include "arm.h"

#include <iostream>

Arm::Arm(Vector3f x, Vector3f r, float l) {
  X_body_to_world = x;
  R_body_to_world = r;
  length = Vector3f(0,l,0);

  X_world_to_body = -1 * X_body_to_world;
  R_world_to_body = -1 * R_body_to_world;
}

void Arm::addToR(Vector3f r) {
  R_body_to_world = R_body_to_world + r;
  R_world_to_body = -1 * R_body_to_world;
}

Vector3f Arm::getPoint() {
  return getAngleAxis(R_body_to_world) * length;
}
void Arm::drawArm() {
  // doesn't do anything yet
}

Affine3f Arm::getAngleAxis(Vector3f v) {
  if (v == Vector3f(0,0,0)) {
    return Affine3f::Identity();
  }

  float mag = v.norm(); 
  v.normalize();
  return (Affine3f) AngleAxisf(mag, v);
}