#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <vector>

#include "hit.hpp"
#include "material.hpp"
#include "ray.hpp"
using namespace std;

// Base class for all 3d entities.
class Object3D {
 public:
  Material *material;

  Object3D() : material(nullptr) {}

  explicit Object3D(Material *material) : material(material) {}

  virtual ~Object3D() = default;

  // Intersect Ray with this object. If hit, store information in hit
  // structure. 定义求交函数
  virtual bool intersect(const Ray &r, Hit &h, float tmin = 0) = 0;

  virtual Vector3f min() const { return Vector3f(); }

  virtual Vector3f max() const { return Vector3f(); }

  virtual Ray shine() const { return Ray(Vector3f::ZERO, Vector3f::ZERO); }
};

#endif
