#pragma once

#include <algorithm>
#include <map>
#include <vector>

#include "aabb.hpp"
#include "hit.hpp"
#include "object3d.hpp"

const float ALPHA = 0.7;  // 收敛速率

struct SPPMKDTreeNode {
  Hit* hit;
  Vector3f minv = Vector3f(INF, INF, INF), maxv = -Vector3f(INF, INF, INF);
  float maxrSquared = 0;
  SPPMKDTreeNode *lson = nullptr, *rson = nullptr;
};

class SPPMKDTree {
  int n;
  Hit** hits;

  SPPMKDTreeNode* build(int l, int r, int d) {
    SPPMKDTreeNode* p = new SPPMKDTreeNode;
    for (int i = l; i <= r; i++) {
      p->minv = minV(p->minv, hits[i]->p), p->maxv = maxV(p->maxv, hits[i]->p);
      p->maxrSquared = std::max(p->maxrSquared, hits[i]->rSquared);
    }
    int m = (l + r) >> 1;
    std::nth_element(hits + l, hits + m, hits + r + 1,
                     [&d](Hit* a, Hit* b) { return a->p[d] < b->p[d]; });
    p->hit = hits[m];
    if (l < m) p->lson = build(l, m - 1, (d + 1) % 3);
    if (m < r) p->rson = build(m + 1, r, (d + 1) % 3);
    return p;
  }

  void del(SPPMKDTreeNode* p) {
    if (p->lson) del(p->lson);
    if (p->rson) del(p->rson);
    delete p;
  }

 public:
  SPPMKDTreeNode* root;

  SPPMKDTree(vector<Hit*>* oriHits) {
    n = oriHits->size();
    hits = new Hit*[n];
    for (int i = 0; i < n; i++) hits[i] = (*oriHits)[i];
    root = build(0, n - 1, 0);
  }

  ~SPPMKDTree() {
    if (!root) return;
    del(root);
    delete[] hits;
  }

  float sqr(float a) { return a * a; }

  void update(SPPMKDTreeNode* p, const Vector3f& photon, const Vector3f& power,
              const Vector3f& d) {
    float minSquared = 0;
    if (photon.x() > p->maxv.x()) minSquared += sqr(photon.x() - p->maxv.x());
    if (photon.x() < p->minv.x()) minSquared += sqr(p->minv.x() - photon.x());
    if (photon.y() > p->maxv.y()) minSquared += sqr(photon.y() - p->maxv.y());
    if (photon.y() < p->minv.y()) minSquared += sqr(p->minv.y() - photon.y());
    if (photon.z() > p->maxv.z()) minSquared += sqr(photon.z() - p->maxv.z());
    if (photon.z() < p->minv.z()) minSquared += sqr(p->minv.z() - photon.z());
    if (minSquared > p->maxrSquared) return;
    if ((photon - p->hit->p).squaredLength() <= p->hit->rSquared) {
      Hit* hit = p->hit;
      float factor = (hit->n * ALPHA + ALPHA) / (hit->n * ALPHA + 1.0);
      hit->n++;
      hit->rSquared *= factor;  // 缩减比例
      hit->flux =
          (hit->flux + hit->power * power) * factor;  // hit->power已经算出
    }
    if (p->lson) update(p->lson, photon, power, d);
    if (p->rson) update(p->rson, photon, power, d);
    p->maxrSquared = p->hit->rSquared;
    if (p->lson)
      p->maxrSquared = std::max(p->maxrSquared, p->lson->hit->rSquared);
    if (p->rson)
      p->maxrSquared = std::max(p->maxrSquared, p->rson->hit->rSquared);
  }
};

struct MeshKDTreeNode {
  Vector3f minv, maxv;
  vector<Object3D*>* faces;
  MeshKDTreeNode *lson = nullptr, *rson = nullptr;

  MeshKDTreeNode(Vector3f minv, Vector3f maxv) : minv(minv), maxv(maxv) {}

  bool intersect(Object3D* face) {
    Vector3f faceMin = face->min(), faceMax = face->max();
    return (faceMin.x() < maxv.x() ||
            faceMin.x() == maxv.x() && faceMin.x() == faceMax.x()) &&
           (faceMax.x() > minv.x() ||
            faceMax.x() == minv.x() && faceMin.x() == faceMax.x()) &&
           (faceMin.y() < maxv.y() ||
            faceMin.y() == maxv.y() && faceMin.y() == faceMax.y()) &&
           (faceMax.y() > minv.y() ||
            faceMax.y() == minv.y() && faceMin.y() == faceMax.y()) &&
           (faceMin.z() < maxv.z() ||
            faceMin.z() == maxv.z() && faceMin.z() == faceMax.z()) &&
           (faceMax.z() > minv.z() ||
            faceMax.z() == minv.z() && faceMin.z() == faceMax.z());  // 注意平面
  }
};

class MeshKDTree {
 public:
  MeshKDTreeNode* root;

  MeshKDTree(vector<Object3D*>* oriFaces) {
    Vector3f minv = Vector3f(INF, INF, INF), maxv = -minv;
    // 取到每个维数最小/大的点
    for (auto face : *oriFaces) {
      minv = minV(minv, face->min());
      maxv = maxV(maxv, face->max());
    }
    root = build(0, 0, oriFaces, minv, maxv);
  }

  bool intersect(MeshKDTreeNode* p, const Ray& ray, Hit& hit,
                 float tmin) const {
    bool flag = false;
    for (auto face : (*p->faces)) {
      if (face->intersect(ray, hit, tmin)) flag = true;
    }
    float tl = INF, tr = INF;  // 估计可能性
    if (p->lson) AABB(p->lson->minv, p->lson->maxv).intersect(ray, tl);
    if (p->rson) AABB(p->rson->minv, p->rson->maxv).intersect(ray, tr);
    if (tl < tr) {
      if (hit.t <= tl) return flag;
      if (p->lson && intersect(p->lson, ray, hit, tmin)) return true;
      if (hit.t <= tr) return flag;
      if (p->rson && intersect(p->rson, ray, hit, tmin)) return true;
    } else {
      if (hit.t <= tr) return flag;
      if (p->rson && intersect(p->rson, ray, hit, tmin)) return true;
      if (hit.t <= tl) return flag;
      if (p->lson && intersect(p->lson, ray, hit, tmin)) return true;
    }
    return flag;
  }

 private:
  MeshKDTreeNode* build(int depth, int div, vector<Object3D*>* faces,
                        const Vector3f& minv, const Vector3f& maxv) {
    MeshKDTreeNode* p = new MeshKDTreeNode(minv, maxv);
    Vector3f maxl = p->maxv, minr = p->minv;
    maxl[div] = minr[div] = (p->minv[div] + p->maxv[div]) / 2;
    p->faces = new vector<Object3D*>;
    for (auto face : *faces) {
      if (p->intersect(face)) p->faces->push_back(face);
    }
    if (p->faces->size() > 128 && depth < 24) {  // 线性即可
      p->lson = build(depth + 1, (div + 1) % 3, p->faces, minv, maxl);
      p->rson = build(depth + 1, (div + 1) % 3, p->faces, minr, maxv);
      vector<Object3D*>*faceL = p->lson->faces, *faceR = p->rson->faces;
      map<Object3D*, int> cnt;
      for (auto face : *faceL) cnt[face]++;
      for (auto face : *faceR) cnt[face]++;
      p->lson->faces = new vector<Object3D*>;
      p->rson->faces = new vector<Object3D*>;
      p->faces->clear();
      for (auto face : *faceL)
        if (cnt[face] == 1)
          p->lson->faces->push_back(face);
        else
          p->faces->push_back(face);  // 两边都在
      for (auto face : *faceR)
        if (cnt[face] == 1) p->rson->faces->push_back(face);
    }
    return p;
  }
};