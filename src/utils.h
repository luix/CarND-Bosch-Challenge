//===--- utils--.h ----------------------------------------------*- C++ -*-===//
//
// This source file is part of the Bosch Challenge Path Planner Project
//
// Author: Luis Vivero <luis.a.vivero@gmail.com>
//
//===----------------------------------------------------------------------===//
//  Udacity Open Source Self-Driving Car
//  Copyright (C) 2016, 2017 Udacity
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//===----------------------------------------------------------------------===//

#ifndef PATH_PLANNING_UTILS_H_H
#define PATH_PLANNING_UTILS_H_H

#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cassert>
#include <limits>
#include <algorithm>
#include <random>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;
using WayPoints = vector<double>;
using Spline = tk::spline;
using SensorData = vector<double>;

const double kInfinity = numeric_limits<double>::infinity();

struct Path {
public:
  Path (const WayPoints xs, const WayPoints ys):
      X(xs), Y(ys) {}
  Path (const Path & rhs): X(rhs.X), Y(rhs.Y) {}
  size_t size() const {
    return X.size();
  }
public:
  WayPoints X;
  WayPoints Y;
};

#endif //PATH_PLANNING_UTILS_H_H
