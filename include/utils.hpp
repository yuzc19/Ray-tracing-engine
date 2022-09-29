#pragma once

#include <vecmath.h>

#include <cmath>
#include <cstdlib>
#include <random>

#define PI 3.14159265358979323846
// Helpers for random number generation
static std::mt19937 mersenneTwister;  // 马特赛特旋转演算法
static std::uniform_real_distribution<float> uniform;
#define RND (2.0 * uniform(mersenneTwister) - 1.0)
#define RND01 (uniform(mersenneTwister))

const float INF = 0x3f3f3f3f;
const int MAX_TRACE_DEPTH = 10;

inline Vector3f diffDir(const Vector3f &n) {
  float theta = 2 * PI * RND01, dis = RND01;
  Vector3f z = n;
  Vector3f x =
      (Vector3f::cross(
           (fabs(z.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), z))
          .normalized();
  Vector3f y = Vector3f::cross(z, x);
  return (x * cos(theta) * sqrt(dis) + y * sin(theta) * sqrt(dis) +
          z * sqrt(1 - dis))
      .normalized();
}