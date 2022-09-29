#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "curve.hpp"
#include "object3d.hpp"

const int NEWTON_STEPS = 5;
const float NEWTON_EPS = 1e-4;

class RevSurface : public Object3D {
  Curve *curve;
  AABB aabb;

 public:
  RevSurface(Curve *curve, Material *material)
      : curve(curve),
        Object3D(material),
        aabb(Vector3f(-curve->radius, curve->ymin - 3, -curve->radius),
             Vector3f(curve->radius, curve->ymax + 3, curve->radius)) {
    // Check flat.
    for (const auto &cp : curve->getControls()) {
      if (cp.z() != 0.0) {
        printf("Profile of revSurface must be flat on xy plane.\n");
        exit(0);
      }
    }
  }

  ~RevSurface() override { delete curve; }

  bool intersect(const Ray &r, Hit &h, float tmin = 0) override {
    // (PA3 optional TODO): implement this for the ray-tracing routine using G-N
    // iteration.
    float t, v;
    if (!aabb.intersect(r, t) || t > h.getT()) return false;
    Vector3f tmpPoint = r.pointAtParameter(t);
    v = atan2(-tmpPoint.z(), tmpPoint.x()) + PI;
    float u = (curve->ymax - tmpPoint.y()) / (curve->ymax - curve->ymin);
    bool flag = false;
    Vector3f normal, point;
    for (int i = 0; i < NEWTON_STEPS; ++i) {
      if (v < 0) v += 2 * PI;
      if (v >= 2 * PI) v = fmod(v, 2 * PI);
      if (u >= 1) u = 1.0 - FLT_EPSILON;
      if (u <= 0) u = FLT_EPSILON;
      Quat4f rot;  // 旋转
      rot.setAxisAngle(v, Vector3f::UP);
      Matrix3f rotMat = Matrix3f::rotation(rot);
      CurvePoint cp = curve->getPoint(u);
      point = rotMat * cp.V;
      Vector3f du = rotMat * cp.T;
      Vector3f dv = Vector3f(-cp.V.x() * sin(v), 0, -cp.V.x() * cos(v));
      Vector3f diff = r.pointAtParameter(t) - point;
      normal = Vector3f::cross(du, dv);
      if (diff.squaredLength() < NEWTON_EPS) {  // 找到交点
        flag = true;
        break;
      }
      // 牛顿迭代
      float D = Vector3f::dot(r.direction, normal);
      t += Vector3f::dot(du, Vector3f::cross(diff, dv)) / D;
      u += Vector3f::dot(r.direction, Vector3f::cross(diff, dv)) / D;
      v -= Vector3f::dot(r.direction, Vector3f::cross(diff, du)) / D;
    }
    if (!flag || t < tmin || t > h.getT() || u < curve->low || u > curve->high)
      return false;
    h.set(t, material, normal.normalized(), material->getColor(v / (2 * PI), u),
          point);  // v是环面
    return true;
  }
};

#endif  // REVSURFACE_HPP
