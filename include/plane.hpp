#ifndef PLANE_H
#define PLANE_H

#include <float.h>
#include <vecmath.h>

#include <cmath>

#include "object3d.hpp"

// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
 public:
  Plane() {}

  Plane(const Vector3f &normal, float d, Material *material)
      : Object3D(material), normal(normal.normalized()), d(d) {
    uaxis = Vector3f::cross(Vector3f::UP, normal);
  }

  ~Plane() override = default;

  bool intersect(const Ray &r, Hit &h, float tmin = 0) override {
    float cos = normal.dot(normal, r.getDirection());
    if (cos > -1e-6) return false;  // 平行
    float t = (d - normal.dot(normal, r.getOrigin())) / cos;
    if (t < tmin || t > h.getT()) return false;
    Vector3f p = r.pointAtParameter(t);
    float u = p.dot(p - d * normal, uaxis), v = p.y();
    h.set(t, material, normal, material->getColor(u, v), p);
    return true;
  }

  Vector3f min() const override {
    return -INF * Vector3f(fabs(normal.x()) < 1 - FLT_EPSILON,
                           fabs(normal.y()) < 1 - FLT_EPSILON,
                           fabs(normal.z()) < 1 - FLT_EPSILON) +
           normal * d;
  }

  Vector3f max() const override {
    return INF * Vector3f(fabs(normal.x()) < 1 - FLT_EPSILON,
                          fabs(normal.y()) < 1 - FLT_EPSILON,
                          fabs(normal.z()) < 1 - FLT_EPSILON) +
           normal * d;
  }

 protected:
  Vector3f normal, uaxis;
  float d;
};

#endif  // PLANE_H
