#include "arm.h"

Arm::Arm(Vector3f x, Vector3f r, float l) {
  X_body_to_world = x;
  R_body_to_world = r;
  length = l;

  X_world_to_body = -1 * X_body_to_world;
  R_world_to_body = -1 * R_body_to_world;
}

void Arm::drawArm() {
  // doesn't do anything yet
}