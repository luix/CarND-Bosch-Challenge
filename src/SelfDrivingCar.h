//===--- SelfDrivingCar--.h -------------------------------------*- C++ -*-===//
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

#ifndef PATH_PLANNING_SELFDRIVINGCAR_H
#define PATH_PLANNING_SELFDRIVINGCAR_H

#include <vector>
#include "Road.h"


class SelfDrivingCar {
private:
  Road road;
  WayPoints previous_s_path;

  const double kTimeInterval = 0.02;
  const int kPathLength = 40;
  const double kMilesPerHourToMetersPerSecond = 0.447;
  const double kMaxSpeed = 21.75;
  const double kSafeCarDistance = 8; // 15 meters
  const size_t kMaxPreviousPathSteps = 20;
  bool in_lane_change;

public:
  SelfDrivingCar(const Road & road): road(road) {
    in_lane_change = false;
  }

  virtual ~SelfDrivingCar() {}

  Path Planner(const Path &previous_path,
               const Vehicle &ego_car,
               const vector<RaceCar> &racers);

private:
  Path FastStart(const Path &previous_path,
                 const Vehicle &ego_car,
                 const vector<RaceCar> &racers);

  Path KeepLane(const Path &previous_path,
                const Vehicle &ego_car,
                const vector<RaceCar> &racers);

  Path ChangeLane(const Path &previous_path,
                  const Vehicle &ego_car,
                  const vector<RaceCar> &racers,
                  int lane_change);

  bool SafeToChangeLane(const Vehicle &ego_car,
                        const vector<RaceCar> &racers,
                        int target_lane) const;

  double Accelerate(double current_speed, double target_speed) const;
};

#endif //PATH_PLANNING_SELFDRIVINGCAR_H
