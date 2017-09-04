//===--- Road--.h -----------------------------------------------*- C++ -*-===//
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

#ifndef PATH_PLANNING_ROAD_H
#define PATH_PLANNING_ROAD_H

#include "Vehicle.h"


class Road {
public:
  WayPoints x;
  WayPoints y;
  WayPoints s;
  WayPoints dx;
  WayPoints dy;

  vector<Spline> lane_s2x;
  vector<Spline> lane_s2y;

  const double kLaneWidth = 4;
  const int kRightMostLane = 2;
  const double kMaxRoadLenght = 6945.554;

public:

  Road(const WayPoints & x,
       const WayPoints & y,
       const WayPoints & s,
       const WayPoints & dx,
       const WayPoints & dy):
      x(x), y(y), s(s), dx(dx), dy(dy) {

    auto n = x.size();
    assert (x.size() == n);
    assert (y.size() == n);
    assert (s.size() == n);
    assert (dx.size() == n);
    assert (dy.size() == n);

    double kHalfLane = kLaneWidth / 2;
    double lower = -0.5;
    double upper = 0.1;
    uniform_real_distribution<double> unif(lower, upper);
    default_random_engine re(random_device{}());
    for (int lane = 0; lane <= kRightMostLane; ++lane) {
      vector<double> lane_x;
      vector<double> lane_y;
      vector<double> lane_s;
      for (auto i = 0; i < n; ++i) {
        double jitter = -0.2;//unif(re);
        lane_x.push_back(x[i] + dx[i] * (kHalfLane + lane*kLaneWidth + jitter));
        lane_y.push_back(y[i] + dy[i] * (kHalfLane + lane*kLaneWidth + jitter));
        lane_s.push_back(s[i]);
      }
      for (auto i = 0; i < n; ++i) {
        double jitter = -0.2;//unif(re);
        lane_x.push_back(x[i] + dx[i] * (kHalfLane + lane*kLaneWidth + jitter));
        lane_y.push_back(y[i] + dy[i] * (kHalfLane + lane*kLaneWidth + jitter));
        lane_s.push_back(s[i] + kMaxRoadLenght);
      }
      Spline s2x; s2x.set_points(lane_s, lane_x);
      Spline s2y; s2y.set_points(lane_s, lane_y);
      lane_s2x.push_back(s2x);
      lane_s2y.push_back(s2y);
    }
  }

  virtual ~Road() {}

  int FindLane(double car_d) const;

  int FindFrontCarInLane(const Vehicle &sdc,
                         const vector<RaceCar> &peer_cars,
                         int lane) const;

  int FindRearCarInLane(const Vehicle &sdc,
                        const vector<RaceCar> &peer_cars,
                        int lane) const;
};

#endif //PATH_PLANNING_ROAD_H
