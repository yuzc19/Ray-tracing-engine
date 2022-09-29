#pragma once

#include <omp.h>
#include <time.h>

#include "camera.hpp"
#include "group.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "kdtree.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "utils.hpp"
using namespace std;

class PT {
 public:
  const SceneParser& scene;
  int samples;
  bool dispersion = false;
  const char* output;

  PT(const SceneParser& scene, int samples, const char* output,
     bool dispersion = false)
      : scene(scene),
        samples(samples),
        output(output),
        dispersion(dispersion) {}

  Vector3f pt(const Ray& r, int depth, float rrefr = 0) {
    Group* group = scene.getGroup();
    Hit hit;
    if (!group->intersect(r, hit, 1e-3)) return scene.getBackgroundColor();
    Vector3f origin = r.pointAtParameter(hit.getT()), n = hit.getNormal();
    bool into = Vector3f::dot(r.getDirection(), n) < 0;  // 是否射入物体
    Vector3f rn = into ? n : -n;
    Vector3f color = hit.color;  // 相乘
    if (depth > MAX_TRACE_DEPTH) return Vector3f(0, 0, 0);
    Material* material = hit.getMaterial();
    depth += 1;
    if (depth > 5) {
      float p = maxNorm(color);
      if (RND01 < p)
        color = color / p;
      else
        return material->emission;
    }
    float type = RND01;
    if (type <= material->type.x()) {  // 漫反射
      return material->emission +
             color * pt(Ray(origin, diffDir(rn)), depth, rrefr);
    } else {
      Ray reflRay = Ray(origin, r.getDirection() -
                                    2 * Vector3f::dot(n, r.getDirection()) * n);
      if (type <= material->type.x() + material->type.y()) {
        return material->emission + color * pt(reflRay, depth, rrefr);
      } else {
        float refr = into ? 1 / material->refr : material->refr;  // 处理折射率
        if (rrefr) refr = into ? 1 / rrefr : rrefr;  // 处理色散
        float cos1 = Vector3f::dot(r.getDirection(), rn);
        float cos2 = 1 - refr * refr * (1 - cos1 * cos1);
        if (cos2 < 0)
          return material->emission + color * pt(reflRay, depth, rrefr);
        Vector3f refrDir = (r.getDirection() * refr -
                            n * ((into ? 1 : -1) * (cos1 * refr + sqrt(cos2))))
                               .normalized();
        float R0 = ((1.0 - refr) * (1.0 - refr)) /
                   ((1.0 + refr) * (1.0 + refr));  // 反射强度
        float Re = R0 +
                   (1 - R0) *
                       pow(1 - (into ? -cos1 : Vector3f::dot(refrDir, n)), 5.0),
              Tr = 1 - Re, P = 0.25 + 0.5 * Re, RP = Re / P,
              TP = Tr / (1 - P);  // 菲涅尔算概率/光强
        if (depth > 3) {
          if (RND01 < P)
            return material->emission + color * pt(reflRay, depth, rrefr) * RP;
          else
            return material->emission +
                   color * pt(Ray(origin, refrDir), depth, rrefr) * TP;
        }
        return material->emission +
               color * (pt(reflRay, depth, rrefr) * Re +
                        pt(Ray(origin, refrDir), depth, rrefr) * Tr);
      }
    }
  }

  void render() {
    Camera* camera = scene.getCamera();
    int w = camera->getWidth(), h = camera->getHeight();
    cout << "Width: " << w << " Height: " << h << endl;
    Image outImg(w, h);
    time_t start = time(NULL);
    printf("Threads: %d\n", omp_get_num_procs());
#pragma omp parallel for schedule(dynamic, 1)  // OpenMP
    for (int y = 0; y < h; y++) {
      float elapsed = (time(NULL) - start), progress = (1.0 + y) / h;
      fprintf(stderr, "\rRendering (%d spp) %5.2f%% Time: %.2f/%.2f s", samples,
              progress * 100., elapsed, elapsed / progress);
      for (int x = 0; x < w; x++) {
        Vector3f color = Vector3f::ZERO;
        for (int s = 0; s < samples; s++) {
          Ray camRay = camera->generateRay(Vector2f(x + RND, y + RND));
          if (!dispersion) {
            color += pt(camRay, 0);
          } else {
            float refrr = 1.6473, refrg = 1.6527, refrb = 1.6726;
            color[0] += pt(camRay, 0, refrr)[0],
                color[1] += pt(camRay, 0, refrg)[1],
                color[2] += pt(camRay, 0, refrb)[2];
          }
        }
        outImg.SetPixel(x, y, color / samples);
      }
    }
    outImg.SaveBMP(output);
    printf("\nResult: %s", output);
  }
};