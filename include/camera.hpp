#ifndef CAMERA_H
#define CAMERA_H

#include <float.h>
#include <vecmath.h>

#include <cmath>

#include "ray.hpp"
#include "utils.hpp"

class Camera {
 public:
  Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up,
         int imgW, int imgH) {
    this->center = center;
    this->direction = direction.normalized();
    this->horizontal = Vector3f::cross(this->direction, up).normalized();
    this->up = Vector3f::cross(this->horizontal, this->direction);
    this->width = imgW;
    this->height = imgH;
  }

  virtual ~Camera() = default;

  // Generate rays for each screen-space coordinate
  virtual Ray generateRay(const Vector2f &point) = 0;

  int getWidth() const { return width; }

  int getHeight() const { return height; }

 protected:
  // Extrinsic parameters
  Vector3f center;
  Vector3f direction;
  Vector3f up;
  Vector3f horizontal;
  // Intrinsic parameters
  int width;
  int height;
};

// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {
 public:
  PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
                    const Vector3f &up, int imgW, int imgH, float angle,
                    float f = 20.0, float aperture = 1.0)
      : Camera(center, direction, up, imgW, imgH),
        f(f),
        aperture(aperture),
        fx((imgW / 2.0) / tan(angle / 2)) {}

  Ray generateRay(const Vector2f &point) override {
    float dx = RND * aperture, dy = RND * aperture;
    Vector3f dr = (Vector3f(f * (point.x() - width / 2.0) / fx - dx,
                            f * (height / 2.0 - point.y()) / fx - dy, f))
                      .normalized();
    Matrix3f r = Matrix3f(horizontal, -up, direction);
    return Ray(center + horizontal * dx - up * dy, (r * dr).normalized());
  }

 protected:
  float fx;
  float f, aperture;  // 焦距和光圈
};

#endif  // CAMERA_H