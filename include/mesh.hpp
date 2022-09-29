#ifndef MESH_H
#define MESH_H

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include "Vector2f.h"
#include "Vector3f.h"
#include "aabb.hpp"
#include "kdtree.hpp"
#include "object3d.hpp"
#include "ray.hpp"
#include "triangle.hpp"

class Mesh : public Object3D {
 public:
  vector<Object3D *> triangles;

  Mesh(const char *filename, Material *m) : Object3D(m) {
    std::vector<TriangleIndex> vIdx, tIdx, nIdx;
    std::vector<Vector3f> v, vn;
    std::vector<Vector2f> vt;
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
      std::cout << "Cannot open " << filename << "\n";
      return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string vnTok("vn");
    std::string texTok("vt");
    std::string bslash("/"), space(" ");
    std::string tok;
    int texID;
    while (true) {
      std::getline(f, line);
      if (f.eof()) {
        break;
      }
      if (line.size() < 3) {
        continue;
      }
      if (line.at(0) == '#') {
        continue;
      }
      std::stringstream ss(line);
      ss >> tok;
      if (tok == vTok) {
        Vector3f vec;
        ss >> vec[0] >> vec[1] >> vec[2];
        v.push_back(vec);
        aabb.update(vec);
      } else if (tok == fTok) {
        bool tFlag = 1, nFlag = 1;
        TriangleIndex vId, tId, nId;
        for (int i = 0; i < 3; i++) {
          std::string str;
          ss >> str;
          std::vector<std::string> id;
          std::string::size_type pos;
          str += bslash;
          int size = str.size();
          for (int i = 0; i < size; i++) {
            pos = str.find(bslash, i);
            if (pos < size) {
              std::string s = str.substr(i, pos - i);
              id.push_back(s);
              i = pos + bslash.size() - 1;
            }
          }
          vId[i] = atoi(id[0].c_str()) - 1;
          if (id.size() > 1) {
            tId[i] = atoi(id[1].c_str()) - 1;
          }
          if (id.size() > 2) {
            nId[i] = atoi(id[2].c_str()) - 1;
          }
        }
        vIdx.push_back(vId);
        tIdx.push_back(tId);
        nIdx.push_back(nId);
      } else if (tok == texTok) {
        Vector2f texcoord;
        ss >> texcoord[0];
        ss >> texcoord[1];
        vt.push_back(texcoord);
      } else if (tok == vnTok) {
        Vector3f vec;
        ss >> vec[0] >> vec[1] >> vec[2];
        vn.push_back(vec);
      }
    }
    f.close();
    for (int triId = 0; triId < (int)vIdx.size(); triId++) {
      TriangleIndex &vIndex = vIdx[triId];
      triangles.push_back((Object3D *)new Triangle(v[vIndex[0]], v[vIndex[1]],
                                                   v[vIndex[2]], m));
      TriangleIndex &tIndex = tIdx[triId];
      if (tIndex.valid())
        ((Triangle *)triangles.back())
            ->setVT(vt[tIndex[0]], vt[tIndex[1]], vt[tIndex[2]]);
      TriangleIndex &nIndex = nIdx[triId];
      if (nIndex.valid())
        ((Triangle *)triangles.back())
            ->setVNorm(vn[nIndex[0]], vn[nIndex[1]], vn[nIndex[2]]);
    }
    kdTree = new MeshKDTree(&triangles);
  }

  ~Mesh() {
    for (int i = 0; i < triangles.size(); i++) delete triangles[i];
    delete kdTree;
  }

  struct TriangleIndex {
    TriangleIndex() {
      x[0] = -1;
      x[1] = -1;
      x[2] = -1;
    }
    int &operator[](const int i) { return x[i]; }
    // By Computer Graphics convention, counterclockwise winding is front
    // face
    int x[3]{};
    bool valid() { return x[0] != -1 && x[1] != -1 && x[2] != -1; }
  };

  bool intersect(const Ray &r, Hit &h, float tmin = 0) override {
    float t;
    if (!aabb.intersect(r, t)) return false;
    if (t > h.getT()) return false;
    return kdTree->intersect(kdTree->root, r, h, tmin);
    // previous test
    // bool flag = false;
    // for (auto triangle : triangles) {
    //   if (triangle->intersect(r, h, tmin)) flag = true;
    // };
    // return flag;
  }

  Vector3f min() const override { return aabb.bounds[0]; }

  Vector3f max() const override { return aabb.bounds[1]; }

  Ray shine() const override {
    int trig = RND01 * triangles.size();
    return triangles[trig]->shine();
  }

 private:
  AABB aabb;
  MeshKDTree *kdTree;
};

#endif
