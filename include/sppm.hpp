#pragma once

#include <time.h>

#include "camera.hpp"
#include "group.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "kdtree.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "utils.hpp"
using namespace std;

const int CKPT_INTERVAL = 5;

class SPPM {
 public:
  const SceneParser& scene;
  int numRounds, numPhotons, ckpt;
  Camera* camera;
  Group* group;
  vector<Object3D*> illuminants;
  int w, h;
  SPPMKDTree* kdTree = nullptr;
  vector<Hit*> hitPoints;
  const char* output;

  SPPM(const SceneParser& scene, int numRounds, int numPhotons, int ckpt,
       const char* output)
      : scene(scene),
        numRounds(numRounds),
        numPhotons(numPhotons),
        ckpt(ckpt),
        output(output) {
    camera = scene.getCamera();
    group = scene.getGroup();
    illuminants = group->getIlluminant();
    w = camera->getWidth();
    h = camera->getHeight();
    for (int u = 0; u < w; u++)
      for (int v = 0; v < h; v++) hitPoints.push_back(new Hit());
    cout << "Width: " << w << " Height: " << h << endl;
  }

  ~SPPM() {
    for (int u = 0; u < w; u++)
      for (int v = 0; v < h; v++) delete hitPoints[u * w + v];
    delete kdTree;
  }

  void goHit(const Ray& r, Vector3f power, Hit* hit, int depth) {
    hit->t = INF;
    if (!group->intersect(r, *hit, 1e-3)) {
      hit->fluxLight += hit->power * scene.getBackgroundColor();
      return;
    }
    Vector3f origin = r.pointAtParameter(hit->t), n = hit->getNormal();
    bool into = Vector3f::dot(r.getDirection(), n) < 0;  // 是否射入物体
    Vector3f rn = into ? n : -n;
    if (depth > MAX_TRACE_DEPTH || maxNorm(power) < 1e-3) return;
    depth += 1;
    Material* material = hit->getMaterial();
    float type = RND01;
    if (type <= material->type.x()) {                     // 漫反射
      hit->power = power * hit->color;                    // 衰减
      hit->fluxLight += hit->power * material->emission;  // 自发光
      return;                                             // 直接返回
    } else {                                              // 高光反射
      Ray reflRay = Ray(origin, r.getDirection() -
                                    2 * Vector3f::dot(n, r.getDirection()) * n);
      if (type <= material->type.x() + material->type.y()) {
        goHit(reflRay, power * hit->color, hit, depth);
        return;
      } else {
        float refr = into ? 1 / material->refr : material->refr;  // 处理折射率
        float cos1 = Vector3f::dot(r.getDirection(), rn);
        float cos2 = 1 - refr * refr * (1 - cos1 * cos1);
        if (cos2 < 0) {
          goHit(reflRay, power * hit->color, hit, depth);
          return;
        }
        Vector3f refrDir = (r.getDirection() * refr -
                            n * ((into ? 1 : -1) * (cos1 * refr + sqrt(cos2))))
                               .normalized();
        float R0 = ((1.0 - refr) * (1.0 - refr)) /
                   ((1.0 + refr) * (1.0 + refr));  // 反射强度
        float Re =
            R0 + (1 - R0) * pow(1 - (into ? -cos1 : Vector3f::dot(refrDir, n)),
                                5.0);  // 菲涅尔算概率
        if (RND01 < Re) {
          goHit(reflRay, power * hit->color, hit, depth);
          return;
        } else {
          goHit(Ray(origin, refrDir), power * hit->color, hit, depth);
          return;
        }
      }
    }
  }

  void goPhoton(const Ray& r, Vector3f power, int depth) {
    Hit hit;
    if (!group->intersect(r, hit, 1e-3)) return;
    Vector3f origin = r.pointAtParameter(hit.t), n = hit.getNormal();
    bool into = Vector3f::dot(r.getDirection(), n) < 0;  // 是否射入物体
    Vector3f rn = into ? n : -n;
    if (depth > MAX_TRACE_DEPTH || maxNorm(power) < 1e-3) return;
    depth += 1;
    Material* material = hit.getMaterial();
    float type = RND01;
    if (type <= material->type.x()) {  // 漫反射
      kdTree->update(kdTree->root, hit.p, power, r.direction);
      goPhoton(Ray(origin, diffDir(rn)), power * hit.color, depth);
      return;
    } else {
      Ray reflRay = Ray(origin, r.getDirection() -
                                    2 * Vector3f::dot(n, r.getDirection()) * n);
      if (type <= material->type.x() + material->type.y()) {
        goPhoton(reflRay, power * hit.color, depth);
        return;
      } else {
        float refr = into ? 1 / material->refr : material->refr;  // 处理折射率
        float cos1 = Vector3f::dot(r.getDirection(), rn);
        float cos2 = 1 - refr * refr * (1 - cos1 * cos1);
        if (cos2 < 0) {
          goPhoton(reflRay, power * hit.color, depth);
          return;
        }
        Vector3f refrDir = (r.getDirection() * refr -
                            n * ((into ? 1 : -1) * (cos1 * refr + sqrt(cos2))))
                               .normalized();
        float R0 = ((1.0 - refr) * (1.0 - refr)) /
                   ((1.0 + refr) * (1.0 + refr));  // 反射强度
        float Re =
            R0 + (1 - R0) * pow(1 - (into ? -cos1 : Vector3f::dot(refrDir, n)),
                                5.0);  // 菲涅尔算概率
        if (RND01 < Re) {
          goPhoton(reflRay, power * hit.color, depth);
          return;
        } else {
          goPhoton(Ray(origin, refrDir), power * hit.color, depth);
          return;
        }
      }
    }
  }

  void save(const char* filename, int numRounds, int numPhotons) {
    Image outImg(w, h);
    for (int u = 0; u < w; u++)
      for (int v = 0; v < h; v++) {
        Hit* hit = hitPoints[u * h + v];
        outImg.SetPixel(
            u, v,
            hit->fluxLight / numRounds +
                hit->flux /
                    (PI * hit->rSquared * numPhotons *
                     numRounds));  // fluxLight是自发光，flux是所携带的吞吐量
      }
    outImg.SaveBMP(filename);
  }

  void render() {
    time_t start = time(NULL);
    Vector3f color = Vector3f::ZERO;
    printf("Threads: %d\n", omp_get_num_procs());
    for (int round = 0; round < numRounds; round++) {
      float elapsed = (time(NULL) - start),
            progress = (1.0 + round) / numRounds;
      fprintf(
          stderr, "\rRendering (%d/%d Rounds) %5.2f%% Time: %.2f/%.2f sec\n",
          round + 1, numRounds, progress * 100., elapsed, elapsed / progress);
#pragma omp parallel for schedule(dynamic, 1)
      for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
          Ray camRay = camera->generateRay(Vector2f(x + RND, y + RND));
          goHit(camRay, Vector3f(1, 1, 1), hitPoints[x * h + y], 0);
        }
      }
      if (kdTree) delete kdTree;
      kdTree = new SPPMKDTree(&hitPoints);
      // photon tracing pass
#pragma omp parallel for schedule(dynamic, 1)
      for (int i = 0; i < numPhotons / illuminants.size(); i++) {
        for (int j = 0; j < illuminants.size(); j++) {
          Ray ray = illuminants[j]->shine();
          goPhoton(ray, illuminants[j]->material->emission * 200, 0);
        }
      }
      if ((round + 1) % ckpt == 0) {
        char filename[100];
        sprintf(filename, "sppm_output/ckpt-%d.bmp", round + 1);
        save(filename, round + 1, numPhotons);
      }
    }
    save(output, numRounds, numPhotons);
  }
};