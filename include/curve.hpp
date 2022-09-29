#ifndef CURVE_HPP
#define CURVE_HPP

#include <vecmath.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "object3d.hpp"

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in
// all the data.
struct CurvePoint {
  Vector3f V;  // Vertex
  Vector3f T;  // Tangent  (unit)
};

class Curve : public Object3D {
 protected:
  std::vector<Vector3f> controls;
  std::vector<float> knot, tpad;
  int n, k;

 public:
  float ymin = INF, ymax = -INF, radius = 0, low, high;

  explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {
    for (auto pt : controls) {
      ymin = std::min(pt.y(), ymin);
      ymax = std::max(pt.y(), ymax);
      radius = std::max(std::max(radius, fabs(pt.x())), fabs(pt.z()));
    }
  }

  bool intersect(const Ray &r, Hit &h, float tmin) override { return false; }

  std::vector<Vector3f> &getControls() { return controls; }

  virtual void discretize(int resolution, std::vector<CurvePoint> &data) = 0;

  CurvePoint getPoint(float mu) {
    int bpos = upper_bound(knot.begin(), knot.end(), mu) - knot.begin() - 1;
    std::vector<float> s(k + 2, 0);
    s[k] = 1;
    std::vector<float> ds(k + 1, 1);
    for (int p = 1; p <= k; p++) {
      for (int ii = k - p; ii <= k; ii++) {
        int i = ii + bpos - k;
        float w1, w2, dw1, dw2;
        if (tpad[i + p] == tpad[i])
          w1 = mu, dw1 = 1;
        else
          w1 = (mu - tpad[i]) / (tpad[i + p] - tpad[i]),
          dw1 = 1.0 / (tpad[i + p] - tpad[i]);
        if (tpad[i + p + 1] == tpad[i + 1])
          w2 = 1 - mu, dw2 = -1;
        else
          w2 = (tpad[i + p + 1] - mu) / (tpad[i + p + 1] - tpad[i + 1]),
          dw2 = -1.0 / (tpad[i + p + 1] - tpad[i + 1]);
        if (p == k) ds[ii] = (dw1 * s[ii] + dw2 * s[ii + 1]) * p;
        s[ii] = w1 * s[ii] + w2 * s[ii + 1];
      }
    }
    s.pop_back();
    int lsk = k - bpos, rsk = bpos + 1 - n;
    if (lsk > 0) {
      for (int i = lsk; i < s.size(); i++)
        s[i - lsk] = s[i], ds[i - lsk] = ds[i];
      s.resize(s.size() - lsk), ds.resize(ds.size() - lsk);
      lsk = 0;
    }
    if (rsk > 0) {
      if (rsk < s.size())
        s.resize(s.size() - rsk), ds.resize(ds.size() - rsk);
      else
        s.clear(), ds.clear();
    }
    CurvePoint now;
    now.V = Vector3f::ZERO, now.T = Vector3f::ZERO;
    for (int i = 0; i < s.size(); i++)
      now.V += controls[i - lsk] * s[i], now.T += controls[i - lsk] * ds[i];
    return now;
  }
};

class BezierCurve : public Curve {
 public:
  explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
    if (points.size() < 4 || points.size() % 3 != 1) {
      printf("Number of control points of BezierCurve must be 3n+1!\n");
      exit(0);
    }
    n = controls.size(), k = n - 1;
    knot.resize(2 * n, 0);
    for (int i = 0; i < n; i++) knot[i + n] = 1;
    for (int i = 0; i <= n + k; i++) tpad.push_back(knot[i]);
    for (int i = 0; i < k; i++) tpad.push_back(knot[n + k]);
    low = 0, high = 1;
  }

  void discretize(int resolution, std::vector<CurvePoint> &data) override {
    data.clear();
    resolution = resolution * (n / k);
    for (int index = 0; index < resolution; index++) {
      float mu = (float)index / (float)resolution;
      data.push_back(getPoint(mu));
    }
  }
};

class BsplineCurve : public Curve {
 public:
  BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
    if (points.size() < 4) {
      printf("Number of control points of BspineCurve must be more than 4!\n");
      exit(0);
    }
    n = controls.size(), k = 3;
    for (int i = 0; i <= n + k; i++) knot.push_back((float)i / (float)(n + k));
    for (int i = 0; i <= n + k; i++) tpad.push_back(knot[i]);
    for (int i = 0; i < k; i++) tpad.push_back(knot[n + k]);
    low = knot[k], high = knot[n];
  }

  void discretize(int resolution, std::vector<CurvePoint> &data) override {
    data.clear();
    resolution = resolution * (n / k);
    for (int index = 0; index < resolution; index++) {
      float mu =
          ((float)index / (float)resolution) * (knot[n] - knot[k]) + knot[k];
      data.push_back(getPoint(mu));
    }
  }
};

#endif  // CURVE_HPP
