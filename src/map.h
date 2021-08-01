/**
 * map.h
 *
 * Created on: Dec 12, 2016
 * Author: mufferm
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include "helper_functions.h"

struct LandmarkObs;

class Map {
public:
    std::vector<LandmarkObs> landmark_list; // List of landmarks in the map
};

#endif  // MAP_H_
