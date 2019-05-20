/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef LINKSTATESLISTENER_H
#define LINKSTATESLISTENER_H

#include <ros/ros.h>
#include <iostream>
#include <map>
#include <eigen3/Eigen/Geometry>
#include <gazebo_msgs/LinkStates.h>

class LinkStatesListener
{
public:
  // Constructor that receives a nodehandle and the links it want to listen
  LinkStatesListener(ros::NodeHandle &nh, const std::vector<std::string> &link_names);

  // Callback that filters the links and extracts the current position and orientation
  void Callback(const gazebo_msgs::LinkStates::ConstPtr &msg);

  // Is the subscriber already listening?
  bool isListening() const;

  // Wait until the subscriber is listening?
  void wait_until_Listener_ready() const;

  // Return the last subscribed position of the link
  Eigen::Vector3d getMapPosition(const std::string &name) const;

  // Return the last subscribed orientation of the link
  Eigen::Quaterniond getMapOrientation(const std::string &name) const;

protected:

  // Subscriber
  ros::Subscriber link_listener_;

  // Stores the current position and orientations of the desired links
  std::map<std::string, Eigen::Vector3d> map_positions_;
  std::map<std::string, Eigen::Quaterniond> map_orientations_;

  // Flag to check if the subscriber is already receiving information
  bool info_received_;
};

#endif // LINKSTATESLISTENER_H
