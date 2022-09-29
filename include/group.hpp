#ifndef GROUP_H
#define GROUP_H

#include <iostream>
#include <vector>

#include "hit.hpp"
#include "kdtree.hpp"
#include "object3d.hpp"
#include "ray.hpp"

class Group : public Object3D {
 public:
  Group() {}

  explicit Group(int num_objects) { group.resize(num_objects, nullptr); }

  Group(const vector<Object3D *> &objs)
      : group(objs), kdTree(new MeshKDTree(&group)) {}

  ~Group() override {
    for (auto i : group) delete i;
    delete kdTree;
  }

  bool intersect(const Ray &r, Hit &h, float tmin = 0) {
    return kdTree->intersect(kdTree->root, r, h, tmin);
    // previous test
    // bool flag = false;
    // for (auto i : group) {
    //   if (i->intersect(r, h, tmin)) flag = true;
    // }
    // return flag;
  }

  void addObject(int index, Object3D *obj) { group[index] = obj; }

  int getGroupSize() { return group.size(); }

  Object3D *operator[](const int &i) {
    if (i >= group.size() || i < 0) {
      std::cerr << "Index Error: i = " << i << std::endl;
      return nullptr;
    }
    return group[i];
  }

  vector<Object3D *> getIlluminant() const {
    vector<Object3D *> illuminant;
    for (int i = 0; i < group.size(); i++)
      if (group[i]->material->emission != Vector3f::ZERO)
        illuminant.push_back(group[i]);
    return illuminant;  // 自发光
  }

 private:
  std::vector<Object3D *> group;
  MeshKDTree *kdTree;
};

#endif