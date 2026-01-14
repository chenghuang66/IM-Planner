#pragma once

#include <gflags/gflags.h>
#include <cmath>

DECLARE_double(MapResolution);
DECLARE_uint32(HeadingNumber);
DECLARE_double(HeadingResolution);
DECLARE_double(WeightCostToCome);
DECLARE_double(WeightCostToGoal);
DECLARE_double(WeightCostToObstacle);
DECLARE_double(ReversePenalty);
DECLARE_double(SteerPenalty);
DECLARE_double(KeepPenalty);
DECLARE_double(MinKeepStep);
DECLARE_double(FrontHang);
DECLARE_double(Wheelbase);
DECLARE_double(RearHang);
DECLARE_double(VehicleWidth);
DECLARE_uint32(SteerClassNumber);
DECLARE_uint32(StepClassNumber);
DECLARE_double(MinStepSize);
DECLARE_double(MinTurningRadius);
DECLARE_double(MaxSteer);

DEFINE_double(MapResolution, 0.2, "Map resolution");
DEFINE_uint32(HeadingNumber, 144, "Heading number");
DEFINE_double(HeadingResolution, 2 * M_PI / FLAGS_HeadingNumber, "Heading resolution");
DEFINE_double(WeightCostToCome, 1.0, "Weight of cost to come");
DEFINE_double(WeightCostToGoal, 1.0, "Weight of cost to goal");
DEFINE_double(WeightCostToObstacle, 1.0, "Weight of coat to obstacle");
DEFINE_double(ReversePenalty, 0.5, "Penalty of reverse driving");
DEFINE_double(SteerPenalty, 0.1, "Penalty of steering");
DEFINE_double(KeepPenalty, 0.1, "Penalty of gear change");
DEFINE_double(MinKeepStep, 20, "Minimum keep step without penalty");
DEFINE_double(FrontHang, 0.920, "Front hang length of vehicle");
DEFINE_double(Wheelbase, 2.560, "Wheelbase length of vehicle");
DEFINE_double(RearHang, 0.880, "Rear hang length of vehicle");
DEFINE_double(VehicleWidth, 1.50, "Width of vehicle");
DEFINE_uint32(SteerClassNumber, 5, "How many steer angles can take");
DEFINE_uint32(StepClassNumber, 10, "How many steps can take");
DEFINE_double(MinStepSize, sqrt(2) * FLAGS_MapResolution, "Minimun step size");
DEFINE_double(MinTurningRadius, 6.2, "Minimum turning radius");
DEFINE_double(MaxSteer, 0.75, "Maximum steer angle");