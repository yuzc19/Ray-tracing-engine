#ifndef HIT_H
#define HIT_H

#include <vecmath.h>

#include "ray.hpp"
class Material;

const float INIT_RSQUARED = 0.0001;

class Hit {
 public:
  float t, rSquared = INIT_RSQUARED;  // sppm
  Material *material;
  Vector3f normal, fluxLight = Vector3f::ZERO, flux = Vector3f::ZERO,
                   color = Vector3f::ZERO, power = Vector3f(1, 1, 1),
                   p;  // flux是流量，power是辐射率，p是点
  int n = 0;

  // constructors
  Hit() {
    material = nullptr;
    t = INF;
    normal = Vector3f::ZERO;
  }

  Hit(float _t, Material *_m, const Vector3f &_n) {
    t = _t;
    material = _m;
    normal = _n;
  }

  float getT() const { return t; }

  Material *getMaterial() const { return material; }

  const Vector3f &getNormal() const { return normal; }

  void set(float _t, Material *_m, const Vector3f &_n, const Vector3f &_c,
           const Vector3f &_p) {
    t = _t;
    material = _m;
    normal = _n;
    color = _c;
    p = _p;
  }
};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
  os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
  return os;
}

#endif  // HIT_H