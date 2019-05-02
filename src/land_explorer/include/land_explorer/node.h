#ifndef LANDEXPL_NODE_H
#define LANDEXPL_NODE_H

#include <ros/node_handle.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>

class LandExplorer :public ros::NodeHandle
{
public:
  LandExplorer(void);
  int exec(void);

private:
  bool calcMovingTarget(std::pair<double, double> &pos, const std::string &frame_id);
  void moveTo(const std::pair<double, double> &pos, const std::string &frame_id);

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_;
  std::string map_topic_;
  std::string map_frame_;
  std::string base_link_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double map_free_thresh_;
  double map_occupied_thresh_;

  double laser_max_dis_;
  double robot_width_;

  friend class OccupancyGridHelper;
};

#endif
