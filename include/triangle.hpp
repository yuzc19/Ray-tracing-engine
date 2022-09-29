#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <float.h>
#include <vecmath.h>

#include <algorithm>
#include <cmath>
#include <iostream>

#include "object3d.hpp"
#include "utils.hpp"
using namespace std;

class Triangle : public Object3D {
 public:
  Vector3f normal;
  Vector3f vertices[3];
  Vector3f center;
  Vector3f bound[2];
  float d;
  Vector3f an = Vector3f::ZERO, bn = Vector3f::ZERO, cn = Vector3f::ZERO;
  bool nSet = false;
  Vector2f at, bt, ct;
  bool tSet = false;

  Triangle() = delete;

  // a b c are three vertex positions of the triangle
  Triangle(const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m)
      : Object3D(m) {
    normal = a.cross(b - a, c - a).normalized();
    vertices[0] = a, vertices[1] = b, vertices[2] = c;
    center = (a + b + c) / 3;
    bound[0] = minV(minV(a, b), c), bound[1] = maxV(maxV(a, b), c);
    d = -normal.dot(normal, a);
  }

  bool intersect(const Ray& ray, Hit& hit, float tmin = 0) override {
    Vector3f pv =
        Vector3f::cross(ray.getDirection(), vertices[2] - vertices[0]);
    float det = Vector3f::dot(vertices[1] - vertices[0], pv);
    if (fabs(det) < FLT_EPSILON) return false;  // 防止平行
    Vector3f tv = ray.getOrigin() - vertices[0];
    float u = Vector3f::dot(tv, pv) / det;
    if (u < 0 || u > 1) return false;
    float v = Vector3f::dot(ray.getDirection(),
                            Vector3f::cross(tv, vertices[1] - vertices[0])) /
              det;
    if (v < 0 || u + v > 1) return false;
    float t = -(d + normal.dot(normal, ray.getOrigin())) /
              (normal.dot(normal, ray.getDirection()));
    if (t < tmin || t > hit.getT()) return false;
    bool in = false;
    Vector3f pr = ray.pointAtParameter(t);
    Vector3f n1 = normal.cross(vertices[0] - pr, vertices[1] - pr),
             n2 = normal.cross(vertices[1] - pr, vertices[2] - pr),
             n3 = normal.cross(vertices[2] - pr, vertices[0] - pr);
    if (normal.dot(normal, n1) >= 0 && normal.dot(normal, n2) >= 0 &&
        normal.dot(normal, n3) >= 0)
      in = true;
    if (!in) return false;
    float ar = n2.length(), br = n3.length(), cr = n1.length();
    Vector3f n = normal;
    if (nSet) n = (ar * an + br * bn + cr * cn).normalized();  // 法向量插值
    if (tSet) {
      Vector2f uv = (ar * at + br * bt + cr * ct) / (ar + br + cr);
      u = uv.x();
      v = uv.y();
    }
    hit.set(t, material, n, material->getColor(u, v), pr);
    return true;
  }

  void setVNorm(const Vector3f& _an, const Vector3f& _bn, const Vector3f& _cn) {
    an = _an;
    bn = _bn;
    cn = _cn;
    nSet = true;
  }

  void setVT(const Vector2f& _at, const Vector2f& _bt, const Vector2f& _ct) {
    at = _at;
    bt = _bt;
    ct = _ct;
    tSet = true;
  }

  Vector3f min() const override { return bound[0]; }

  Vector3f max() const override { return bound[1]; }

 protected:
  Ray shine() const override {
    float r1 = RND01, r2 = RND01;
    if (r1 + r2 > 1) {
      r1 = 1 - r1;
      r2 = 1 - r2;
    }
    return Ray(
        r1 * vertices[1] + r2 * vertices[2] + (1 - r1 - r2) * vertices[0],
        diffDir(normal));
  }
};

#endif  // TRIANGLE_H
