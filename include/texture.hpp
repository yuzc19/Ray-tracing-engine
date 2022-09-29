#pragma once

#include <vecmath.h>

class Texture {  // 纹理
 public:
  unsigned char *image = nullptr;
  int w, h, c;

  Vector3f getColor(float u, float v) {
    u -= int(u), v -= int(v);  // 水平/竖直投影
    if (u < 0) u = u + 1;
    if (v < 0) v = v + 1;
    // 注意不要颠倒
    u = w * u, v = h * v;
    float a = u - (int)u, b = v - (int)v;
    Vector3f color = Vector3f::ZERO;
    // 平滑一下
    color += (1 - a) * (1 - b) * getColor((int)u, (int)v);
    color += a * (1 - b) * getColor((int)u + 1, (int)v);
    color += (1 - a) * b * getColor((int)u, (int)v + 1);
    color += a * b * getColor((int)u + 1, (int)v + 1);
    return color;
  }

 private:
  Vector3f getColor(int u, int v) {
    u = min(w - 1, max(u, 0)), v = min(h - 1, max(v, 0));
    int index = v * w * c + u * c;
    return Vector3f(image[index], image[index + 1], image[index + 2]) / 255.0;
  }
};