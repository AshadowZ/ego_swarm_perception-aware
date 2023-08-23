#include <ros/ros.h>
#include <iostream>
#include <traj_utils/Bspline.h>
#include <Eigen/Eigen>
#include <bspline_opt/bspline_optimizer.h>
#include <cmath>

using namespace std;
using ego_planner::UniformBspline;

void computeYaw(int myArg); // 计算yaw角的子线程
// void computeYawVel(traj_utils::Bspline& bspline); // 接收B样条轨迹，用velocity-tracking补全其中的yaw角部分
