#ifndef SPHERE_H
#define SPHERE_H

#include <float.h>
#include <vecmath.h>

#include <cmath>

#include "object3d.hpp"
#include "utils.hpp"

class Sphere : public Object3D {
 public:
  Vector3f center;
  float radius;

  Sphere() {
    // unit ball at the center
    this->center = Vector3f(0, 0, 0);
    this->radius = 1;
  }

  Sphere(const Vector3f &center, float radius, Material *material)
      : Object3D(material), center(center), radius(radius) {}

  ~Sphere() override = default;

  bool intersect(const Ray &r, Hit &h, float tmin = 0) override {
    Vector3f l = center - r.getOrigin();
    float tp = l.dot(l, r.getDirection().normalized());
    float tmp = radius * radius - l.squaredLength() + tp * tp;
    if (tmp < 0) return false;
    float t1 = tp - sqrt(tmp), t2 = tp + sqrt(tmp);
    float t;
    if (t1 >= tmin && t1 <= h.getT())
      t = t1;
    else if (t2 >= tmin && t2 <= h.getT())
      t = t2;
    else
      return false;
    Vector3f n = (r.pointAtParameter(t) - center).normalized();
    float u = 0.5 + atan2(n.x(), n.z()) / (2 * PI), v = 0.5 - asin(n.y()) / PI;
    h.set(t, material, n, material->getColor(u, v), r.pointAtParameter(t));
    return true;
  }

  Ray shine() const override {
    float u = RND, v = RND;
    float rSquared = u * u + v * v;
    while (rSquared >= 1) {
      u = RND;
      v = RND;
      rSquared = u * u + v * v;
    }
    Vector3f dir = Vector3f(2 * u * sqrtf(1 - rSquared),
                            2 * v * sqrt(1 - rSquared), 1 - 2 * rSquared)
                       .normalized();
    return Ray(center + radius * dir, dir);
  }

  Vector3f min() const override { return center - radius; }

  Vector3f max() const override { return center + radius; }
};

#endif
