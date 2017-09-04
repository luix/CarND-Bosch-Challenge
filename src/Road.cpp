//===--- Road--.cpp ---------------------------------------------*- C++ -*-===//
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

#include "Road.h"

int Road::FindLane(double car_d) const {
  int lane = floor(car_d / kLaneWidth);
  lane = max(0, lane);
  lane = min(kRightMostLane, lane);
  return lane;
}


int Road::FindFrontCarInLane(const Vehicle &sdc,
                       const vector<RaceCar> &peer_cars,
                       int lane) const {
  int found_car = -1;
  double found_dist = kInfinity;
  for (auto i = 0; i < peer_cars.size(); ++i) {
    const RaceCar & car = peer_cars[i];
    auto car_lane = FindLane(car.d);
    if ( (car_lane == lane) && (car.s >= sdc.s) ) {
      double dist = car.s - sdc.s;
      if (dist < found_dist) {
        found_car = i;
        found_dist = dist;
      }
    }
  }
  return found_car;
}

int Road::FindRearCarInLane(const Vehicle &sdc,
                             const vector<RaceCar> &peer_cars,
                             int lane) const {
    int found_car = -1;
    double found_dist = kInfinity;
    for (auto i = 0; i < peer_cars.size(); ++i) {
      const RaceCar & car = peer_cars[i];
      auto car_lane = FindLane(car.d);
      if ( (car_lane == lane) && (car.s <= sdc.s) ) {
        double dist = sdc.s - car.s;
        if (dist < found_dist) {
          found_car = i;
          found_dist = dist;
        }
      }
    }
    return found_car;
}