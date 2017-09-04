//===--- Vehicle--.cpp ------------------------------------------*- C++ -*-===//
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

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


#include "utils.h"

class Vehicle {
public:
  double x, y;
  double s, d;
  double yaw;
  double speed;

  Vehicle(double x, double y,
          double s, double d,
          double yaw, double speed):
      x(x), y(y),
      s(s), d(d),
      yaw(yaw),
      speed(speed) {};
};


// Other cars on the road
class RaceCar {
public:
  int id;
  double x, y;
  double vx, vy;
  double s, d;

  RaceCar(const SensorData & sensor):
      id(sensor[0]), x(sensor[1]), y(sensor[2]),
      vx(sensor[3]), vy(sensor[4]),
      s(sensor[5]), d(sensor[6]) {}
};

#endif //PATH_PLANNING_VEHICLE_H
