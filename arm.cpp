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
  return Translation3f(X_body_to_world) * getAngleAxis(R_body_to_world) * length;
}
void Arm::drawArm() {
  // glBegin(GL_LINES);
  // glVertex3f(0.0, 0.0, 0.0);
  // glVertex3f(length.x(), length.y(), length.z());
  // glEnd();

  // glRotatef(R_body_to_world.norm(), R_body_to_world.x(), R_body_to_world.y(), R_body_to_world.z());
}

Affine3f Arm::getAngleAxis(Vector3f v) {
  if (v == Vector3f(0,0,0)) {
    return Affine3f::Identity();
  }

  float mag = v.norm(); 
  v.normalize();
  return (Affine3f) AngleAxisf(mag, v);
}