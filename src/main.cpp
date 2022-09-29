#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>

#include "pt.hpp"
#include "sppm.hpp"
using namespace std;

int main(int argc, char* argv[]) {
  for (int argNum = 1; argNum < argc; ++argNum) {
    std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
  }
  if (argc < 5) {
    std::cout << "Usage: ./bin/PA <input scene file> <output bmp file> "
                 "<method> [<spp>]/[<numRounds> <numPhotons> <ckpt_interval>]"
              << endl;
    return 1;
  }
  SceneParser scene(argv[1]);
  if (!strcmp(argv[3], "pt")) {
    int samps = atoi(argv[4]);
    if (argc == 5) {
      PT pt(scene, samps, argv[2]);
      pt.render();
    }
    if (argc == 6 && !strcmp(argv[5], "dispersion")) {
      PT pt(scene, samps, argv[2], true);
      pt.render();
    }
  } else if (!strcmp(argv[3], "sppm")) {
    int numRounds = atoi(argv[4]), numPhotons = atoi(argv[5]),
        ckpt = atoi(argv[6]);
    SPPM sppm(scene, numRounds, numPhotons, ckpt, argv[2]);
    sppm.render();
  } else {
    cout << "Unknown method: " << argv[4] << endl;
    return 1;
  }
  return 0;
}
