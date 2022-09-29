#ifndef MATERIAL_H
#define MATERIAL_H

#include <vecmath.h>

#include <cassert>
#include <iostream>

#include "hit.hpp"
#include "ray.hpp"
#include "texture.hpp"

class Material {
 public:
  Vector3f emission;  // 发光系数
  float refr;         // 折射率
  Vector3f type;      // 种类
  Texture texture;    // 纹理

  explicit Material(const Vector3f &d_color,
                    const Vector3f &s_color = Vector3f::ZERO, float s = 0,
                    const Vector3f &e_color = Vector3f::ZERO, float r = 1,
                    Vector3f t = Vector3f(1, 0, 0))
      : diffuseColor(d_color),
        specularColor(s_color),
        shininess(s),
        emission(e_color),
        refr(r),
        type(t) {}

  virtual ~Material() = default;

  Vector3f getColor(float u, float v) {
    if (!texture.image)
      return diffuseColor;
    else
      return texture.getColor(u, v);
  }

  Vector3f Shade(const Ray &ray, const Hit &hit, const Vector3f &dirToLight,
                 const Vector3f &lightColor) {
    Vector3f shaded = Vector3f::ZERO;
    Vector3f n = hit.getNormal();
    Vector3f v = -ray.getDirection();
    Vector3f r =
        2 * (n.dot(n, dirToLight.normalized())) * n - dirToLight.normalized();
    shaded =
        lightColor * (clamp(n.dot(n, dirToLight.normalized())) * hit.color +
                      pow(clamp(v.dot(v, r)), shininess) * specularColor);
    return shaded;
  }

 protected:
  Vector3f diffuseColor;   // 颜色
  Vector3f specularColor;  // 镜面反射系数
  float shininess;         // 高光指数

  float clamp(float num) { return num < 0 ? 0 : num; }
};

#endif  // MATERIAL_H
