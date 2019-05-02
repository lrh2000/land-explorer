#include <cstring>
#include <queue>
#include <ros/topic.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "land_explorer/node.h"

class OccupancyGridHelper
{
public:
  OccupancyGridHelper(const nav_msgs::OccupancyGridConstPtr &map,
                      LandExplorer *node, const std::string &frame_id);
  ~OccupancyGridHelper(void);
  void initMap(const std::pair<int, int> &origin);

  std::pair<int, int> convertToCellPos(const std::pair<double, double> &pos,
                                        const std::string &origin_frame);
  std::pair<double, double> convertFromCellPos(const std::pair<int, int> &pos,
                                               const std::string &target_frame);

  int countUnstableCells(void);
  void getUnknownsMax(int &max_val, std::pair<int, int> &max_pos);

private:
  template<typename T1, typename T2>
    T2 getValue(T1 array, int x, int y, T2 null);
  template<typename T1, typename T2>
    void setValue(T1 array, int x, int y, T2 data);

  void calcOccupied(void);
  void calcFree(const std::pair<int, int> &origin);
  void calcPassable(void);
  void calcReachable(const std::pair<int, int> &origin);
  void calcUnknowns(void);

  bool *calcVisible(bool *source,
      const std::pair<int, int> &pos, int half_width);

private:
  nav_msgs::OccupancyGridConstPtr map_;
  LandExplorer *node_;
  std::string frame_id_;
  bool *not_free_;
  bool *is_occupied_;
  bool *is_passable_;
  bool *is_reachable_;
  int *unknowns_;
  int unstable_cells_;
};

OccupancyGridHelper::OccupancyGridHelper(const nav_msgs::OccupancyGridConstPtr &map,
                                          LandExplorer *node, const std::string &frame_id)
  : map_(map), node_(node), frame_id_(frame_id),
    not_free_(new bool[map->info.width * map->info.height]),
    is_occupied_(new bool[map->info.width * map->info.height]),
    is_passable_(new bool[map->info.width * map->info.height]),
    is_reachable_(new bool[map->info.width * map->info.height]),
    unknowns_(new int[map->info.width * map->info.height])
{}

OccupancyGridHelper::~OccupancyGridHelper(void)
{
  delete[] not_free_;
  delete[] is_occupied_;
  delete[] is_passable_;
  delete[] is_reachable_;
  delete[] unknowns_;
}

template<typename T1, typename T2>
  T2 OccupancyGridHelper::getValue(T1 array, int x, int y, T2 null)
{
  if (x < 0 || x >= map_->info.width)
    return null;
  if (y < 0 || y >= map_->info.height)
    return null;
  return array[x + y * map_->info.width];
}

template<typename T1, typename T2>
  void OccupancyGridHelper::setValue(T1 array, int x, int y, T2 data)
{
  if (x < 0 || x >= map_->info.width || y < 0 || y >= map_->info.height)
    ROS_ERROR("OccupancyGridHelper::setValue is called with illegal (x, y)!!!");
  array[x + y * map_->info.width] = data;
}

inline int OccupancyGridHelper::countUnstableCells(void)
{
  return unstable_cells_;
}

void OccupancyGridHelper::getUnknownsMax(int &max_val, std::pair<int, int> &max_pos)
{
  int step = node_->robot_width_ / map_->info.resolution;

  max_val = -1;
  for (int x = 0;x < map_->info.width;x += step)
  {
    for (int y = 0;y < map_->info.height;y += step)
    {
      int val = getValue(unknowns_, x, y, -1);
      if (val <= max_val)
        continue;
      max_val = val;
      max_pos.first = x;
      max_pos.second = y;
    }
  }
}

void OccupancyGridHelper::initMap(const std::pair<int, int> &origin)
{
  calcOccupied();
  calcFree(origin);
  calcPassable();
  calcReachable(origin);
  calcUnknowns();
}

void OccupancyGridHelper::calcOccupied(void)
{
  int occupied_thresh = node_->map_occupied_thresh_ * 100;

  for (int x = 0;x < map_->info.width;++x)
  {
    for (int y = 0;y < map_->info.height;++y)
    {
      int prob = getValue(map_->data, x, y, -1);
      if (prob >= occupied_thresh)
        setValue(is_occupied_, x, y, true);
      else
        setValue(is_occupied_, x, y, false);
    }
  }
}

void OccupancyGridHelper::calcFree(const std::pair<int, int> &origin)
{
  int free_thresh = node_->map_free_thresh_ * 100;
  int occupied_thresh = node_->map_occupied_thresh_ * 100;
  int half_width = node_->laser_max_dis_ / map_->info.resolution / sqrt(2);
  int origin_x = origin.first;
  int origin_y = origin.second;

  bool *visible = calcVisible(is_occupied_, origin, half_width);

  unstable_cells_ = 0;
  for (int x = 0;x < map_->info.width;++x)
  {
    for (int y = 0;y < map_->info.height;++y)
    {
      int prob = getValue(map_->data, x, y, -1);
      if (prob > free_thresh || prob < 0)
        setValue(not_free_, x, y, true);
      else
        setValue(not_free_, x, y, false);

      if (std::abs(x - origin_x) <= half_width && std::abs(y - origin_y) <= half_width &&
          (prob < 0 || (prob > free_thresh && prob < occupied_thresh))) {
        int dx = x - origin_x + half_width;
        int dy = y - origin_y + half_width;
        if (visible[dx + dy * (half_width * 2 + 1)]) {
          setValue(not_free_, x, y, false);
          ++unstable_cells_;
        }
      }
    }
  }

  delete[] visible;
}

void OccupancyGridHelper::calcPassable(void)
{
  int half_width = node_->robot_width_ / map_->info.resolution / sqrt(2);

  for (int x = 0;x < map_->info.width;++x)
  {
    for (int y = 0;y < map_->info.height;++y)
    {
      bool ok = true;
      for (int new_x = x - half_width;new_x <= x + half_width;++new_x)
        for (int new_y = y - half_width;new_y <= y + half_width;++new_y)
          ok &= !getValue(not_free_, new_x, new_y, true);
      setValue(is_passable_, x, y, ok);
    }
  }
}

void OccupancyGridHelper::calcReachable(const std::pair<int, int> &origin)
{
  static const int dxdy[4][2] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};
  std::queue<std::pair<int, int> > queue;

  memset(is_reachable_, 0, sizeof(bool) * map_->info.height * map_->info.width);
  setValue(is_reachable_, origin.first, origin.second, true);
  queue.push(origin);
  while (!queue.empty())
  {
    int x = queue.front().first;
    int y = queue.front().second;
    queue.pop();

    for (int i = 0;i < 4;++i)
    {
      int new_x = dxdy[i][0] + x;
      int new_y = dxdy[i][1] + y;
      if (!getValue(is_passable_, new_x, new_y, false))
        continue;
      if (getValue(is_reachable_, new_x, new_y, true))
        continue;
      setValue(is_reachable_, new_x, new_y, true);
      queue.push(std::make_pair(new_x, new_y));
    }
  }
}

void OccupancyGridHelper::calcUnknowns(void)
{
  int laser_range = node_->laser_max_dis_ / map_->info.resolution;
  int half_width = laser_range / sqrt(2);
  int width = half_width * 2 + 1;
  int free_thresh = node_->map_free_thresh_ * 100;
  int occupied_thresh = node_->map_occupied_thresh_ * 100;
  int step = node_->robot_width_ / map_->info.resolution;

  for (int x = 0;x < map_->info.width;x += step)
  {
    for (int y = 0;y < map_->info.height;y += step)
    {
      int &unknown = unknowns_[x + y * map_->info.width];
      unknown = -1;
      if (!getValue(is_reachable_, x, y, false))
        continue;

      bool *visible = calcVisible(is_occupied_, std::make_pair(x, y), half_width);
      unknown = 0;
      for (int i = 0;i < width;++i)
      {
        for (int j = 0;j < width;++j)
        {
          if (!visible[i + j * width])
            continue;
          int prob = getValue(map_->data, x + (i - half_width), y + (j - half_width), -1);
          if (prob < 0 || (prob > free_thresh && prob < occupied_thresh))
            ++unknown;
        }
      }
      delete[] visible;
    }
  }
}

bool *OccupancyGridHelper::calcVisible(bool *source,
          const std::pair<int, int> &pos, int half_width)
{
  int origin_x = pos.first;
  int origin_y = pos.second;
  int width = half_width * 2 + 1;
  bool *result = new bool[width * width];

  for (int i = 0;i < width;++i)
  {
    int dx = i - half_width;
    int target_x = origin_x + dx;
    for (int j = 0;j < width;++j)
    {
      int dy = j - half_width;
      int target_y = origin_y + dy;

      bool ok = true;
      if (std::abs(dx) <= std::abs(dy) && dy) {
        int inc = dy > 0 ? 1 : -1;
        int y = origin_y - inc;
        double x = origin_x;
        double k = 1.0 * dx / dy;
        do {
          y += inc;
          if (getValue(source, (int)x, y, true)) {
            ok = false;
            break;
          }
          x += k * inc;
        } while (y != target_y);
      } else if (std::abs(dx) >= std::abs(dy) && dx) {
        int inc = dx > 0 ? 1 : -1;
        int x = origin_x - inc;
        double y = origin_y;
        double k = 1.0 * dy / dx;
        do {
          x += inc;
          if (getValue(source, x, (int)y, true)) {
            ok = false;
            break;
          }
          y += k * inc;
        } while (x != target_x);
      }
      result[i + width * j] = ok;
    }
  }

  return result;
}

std::pair<int, int> OccupancyGridHelper::convertToCellPos(
        const std::pair<double, double> &pos, const std::string &origin_frame)
{
  geometry_msgs::PointStamped point;
  point.header.stamp = ros::Time(0);
  point.header.frame_id = origin_frame;
  point.point.x = pos.first;
  point.point.y = pos.second;

  point = node_->tf_buffer_.transform(point, frame_id_);
  int x = (point.point.x - map_->info.origin.position.x) / map_->info.resolution;
  int y = (point.point.y - map_->info.origin.position.y) / map_->info.resolution;
  return std::make_pair(x, y);
}

std::pair<double, double> OccupancyGridHelper::convertFromCellPos(
        const std::pair<int, int> &pos, const std::string &target_frame)
{
  geometry_msgs::PointStamped point;
  point.header.stamp = ros::Time(0);
  point.header.frame_id = frame_id_;
  point.point.x = pos.first * map_->info.resolution + map_->info.origin.position.x;
  point.point.y = pos.second * map_->info.resolution + map_->info.origin.position.y;

  point = node_->tf_buffer_.transform(point, target_frame);
  return std::make_pair(point.point.x, point.point.y);
}

LandExplorer::LandExplorer(void)
  : move_base_("/move_base", true),
    map_topic_("/map"),
    map_frame_("map"),
    base_link_frame_("base_link"),
    tf_listener_(tf_buffer_),
    map_free_thresh_(0.49),
    map_occupied_thresh_(0.51),
    laser_max_dis_(3.5),
    robot_width_(0.3)
{
  ROS_INFO("Check for some necessary topics:");

  ROS_INFO("Waiting for map topic (%s)...", map_topic_.c_str());
  ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic_, *this);
  ROS_INFO("Done.");

  ROS_INFO("Waiting for move_base action server...");
  move_base_.waitForServer();
  ROS_INFO("Done.\n");
}

int LandExplorer::exec(void)
{
  int id = 1;

  while (ros::ok())
  {
    ROS_INFO("ID: %d", id++);

    std::pair<double, double> pos;
    if (!calcMovingTarget(pos, base_link_frame_))
      break;
    moveTo(pos, base_link_frame_);
  }

  return 0;
}

bool LandExplorer::calcMovingTarget(std::pair<double, double> &pos, const std::string &frame_id)
{
  ROS_INFO("Getting current map...");
  nav_msgs::OccupancyGridConstPtr map_ptr =
        ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic_, *this);
  ROS_INFO("Done.");

  ROS_INFO("Calculating where to move...");
  OccupancyGridHelper map(map_ptr, this, map_frame_);
  map.initMap(map.convertToCellPos(std::make_pair(0.0, 0.0), base_link_frame_));

  int gain;
  std::pair<int, int> pos1;
  map.getUnknownsMax(gain, pos1);
  if (gain < 5) {
    ROS_INFO("The current map is good enough! No need to move.\n");
    ROS_INFO("Exiting...");
    return false;
  }
  ROS_INFO("Moving to (%d, %d) in coordinate frame of map cells, "
        "which is expected to make %d cells in the map clear.", pos1.first, pos1.second, gain);

  pos = map.convertFromCellPos(pos1, frame_id);
  return true;
}

void LandExplorer::moveTo(const std::pair<double, double> &pos, const std::string &frame_id)
{
  ROS_INFO("Moving to (%lf, %lf) in %s frame...", pos.first, pos.second, frame_id.c_str());

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = pos.first;
  goal.target_pose.pose.position.y = pos.second;
  goal.target_pose.pose.orientation.w = 1;
  move_base_.sendGoal(goal);

  ROS_INFO("Waiting for move_base to complete this move (for at most 20 seconds)...");
  if (move_base_.waitForResult(ros::Duration(20.0))) {
    if (move_base_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Moved successfully.\n");
    else
      ROS_INFO("Failed to move. Ignore this and continue.\n");
  } else {
    ROS_INFO("Timeout. Cancel this move and continue.\n");
    move_base_.cancelGoal();
    move_base_.waitForResult();
  }
}
