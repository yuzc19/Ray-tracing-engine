#pragma once

#include <vecmath.h>

#include <vector>

#include "ray.hpp"
using namespace std;

class AABB {
 public:
  Vector3f bounds[2];

  AABB() { bounds[0] = Vector3f(INF), bounds[1] = -bounds[0]; }

  AABB(const Vector3f &min, const Vector3f &max) {
    bounds[0] = min, bounds[1] = max;
  }

  void update(const Vector3f &v) {
    bounds[0] = minV(bounds[0], v), bounds[1] = maxV(bounds[1], v);
  }

  bool intersect(const Ray &r, float &tmin) {
    Vector3f o(r.getOrigin()), invdir(1 / r.getDirection());
    bool negx = invdir.x() < 0, negy = invdir.y() < 0, negz = invdir.z() < 0;
    tmin = INF;
    // 六个面
    float txmin = (bounds[negx].x() - o.x()) * invdir.x(),
          txmax = (bounds[negx ^ 1].x() - o.x()) * invdir.x(),
          tymin = (bounds[negy].y() - o.y()) * invdir.y(),
          tymax = (bounds[negy ^ 1].y() - o.y()) * invdir.y(),
          tzmin = (bounds[negz].z() - o.z()) * invdir.z(),
          tzmax = (bounds[negz ^ 1].z() - o.z()) * invdir.z();
    if (txmin > tymax || tymin > txmax) return false;
    txmin = max(txmin, tymin), txmax = min(txmax, tymax);
    if (txmin > tzmax || tzmin > txmax) return false;
    // 真正的交点
    txmin = max(txmin, tzmin);
    tmin = txmin;
    return true;
  }
};