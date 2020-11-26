/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,  PickNik Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: John Stechschulte
 * Desc: Plugin for AX=XB solver implementations in OpenCV. */

#include <moveit/handeye_calibration_solver/handeye_solver_opencv.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_solver_opencv";

void HandEyeSolverOpenCV::initialize()
{
  solver_names_ = { "Daniilidis1999", "ParkMartin", "TsaiLenz1989", "HoraudDornaika1995", "Andreff1999" };
  camera_robot_pose_ = Eigen::Isometry3d::Identity();
}

const std::vector<std::string>& HandEyeSolverOpenCV::getSolverNames() const
{
  return solver_names_;
}

const Eigen::Isometry3d& HandEyeSolverOpenCV::getCameraRobotPose() const
{
  return camera_robot_pose_;
}

bool HandEyeSolverOpenCV::solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                                const std::vector<Eigen::Isometry3d>& object_wrt_sensor, SensorMountType setup,
                                const std::string& solver_name)
{
  const size_t& number_of_poses = effector_wrt_world.size();
  // Check the size of the two sets of pose sample equal
  if (number_of_poses != object_wrt_sensor.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "The sizes of the two input pose sample vectors are not equal: "
                                    "effector_wrt_world.size() = "
                                        << effector_wrt_world.size()
                                        << " and object_wrt_sensor.size() == " << object_wrt_sensor.size());
    return false;
  }

  auto it = std::find(solver_names_.begin(), solver_names_.end(), solver_name);
  if (it == solver_names_.end())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown handeye solver name: " << solver_name);
    return false;
  }

  // if (setup == EYE_TO_HAND)
  //  python_value = PyString_FromString("Fixed");
  // else if (setup == EYE_IN_HAND)
  //  python_value = PyString_FromString("Moving");

  for (size_t i = 0; i < number_of_poses; ++i)
  {
  }

  return true;
}

}  // namespace moveit_handeye_calibration
