#include <thread>
#include <mutex>
#include <unordered_map>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Open3D/Visualization/Visualizer/Visualizer.h>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/IO/ClassIO/PointCloudIO.h>

class HeightGrid
{
public:
  HeightGrid(const nav_msgs::OccupancyGrid::ConstPtr &grid,
              double now_x, double now_y, int occupied_prob);
  HeightGrid(const HeightGrid &) = delete;
  ~HeightGrid(void);
  HeightGrid &operator =(const HeightGrid &) = delete;

  inline void reportHeight(double x, double y, double z);
  HeightGrid &operator +=(const HeightGrid &other);

  void calculate(void);
  void generatePointCloud(std::vector<Eigen::Vector3d> &points);

private:
  inline std::tuple<double, double> getPosition(int x, int y) const;
  inline int getIndex(int x, int y) const;
  inline int getIndex(double x, double y) const;

private:
  int width_, height_;
  double origin_x_, origin_y_;
  double resolution_;
  double *reported_heights_;
  double *heights_;
  double current_x_, current_y_;
};

inline std::tuple<double, double> HeightGrid::getPosition(int x, int y) const
{
  return std::make_tuple(x * resolution_ + origin_x_, y * resolution_ + origin_y_);
}

inline int HeightGrid::getIndex(int x, int y) const
{
  if (x < 0 || x >= width_)
    return -1;
  if (y < 0 || y >= height_)
    return -1;
  return x + y * width_;
}

inline int HeightGrid::getIndex(double x, double y) const
{
  return getIndex(int((x - origin_x_) / resolution_), int((y - origin_y_) / resolution_));
}

inline void HeightGrid::reportHeight(double x, double y, double z)
{
  int idx = getIndex(x, y);
  if (idx < 0)
    return;
  double &h = reported_heights_[getIndex(x, y)];
  h = std::max(h, z);

  for (int dx = -1;dx <= 1;++dx)
  {
    for (int dy = -1;dy <= 1;++dy)
    {
      if (!dx && !dy)
        continue;
      int nx = x + dx;
      int ny = y + dy;
      int idx = getIndex(nx, ny);
      if (idx < 0)
        continue;

      if (heights_[idx] && reported_heights_[idx] < z)
        reported_heights_[idx] = z;
    }
  }
}

HeightGrid::HeightGrid(const nav_msgs::OccupancyGrid::ConstPtr &grid,
                            double now_x, double now_y, int occupied_prob)
  : width_(grid->info.width), height_(grid->info.height),
    origin_x_(grid->info.origin.position.x),
    origin_y_(grid->info.origin.position.y),
    resolution_(grid->info.resolution),
    reported_heights_(new double[width_ * height_]),
    heights_(new double[width_ * height_]),
    current_x_(now_x), current_y_(now_y)
{
  bool *visited = new bool[width_ * height_];
  memset(visited, 0, sizeof(visited[0]) * width_ * height_);
  std::function<void(int, int)> func = [&] (int x, int y) {
      int idx = getIndex(x, y);
      if (idx < 0 || visited[idx])
        return;
      int prob = grid->data[idx];
      if (prob >= occupied_prob)
        return;
      visited[idx] = true;
      for (int dx = -1;dx <= 1;dx += 1)
      {
        for (int dy = -1;dy <= 1;dy += 1)
        {
          if (!dx && !dy)
            continue;
          if (dx && dy)
            continue;
          func(x + dx, y + dy);
        }
      }
    };

  int idx = getIndex(now_x, now_y);
  func(idx % width_, idx / width_);
  func(0, 0);

  for (int x = 0;x < width_;++x)
  {
    for (int y = 0;y < height_;++y)
    {
      int idx = getIndex(x, y);
      reported_heights_[idx] = heights_[idx] = (visited[idx] ? 0 : 0.1);
    }
  }
  delete[] visited;
}

HeightGrid::~HeightGrid(void)
{
  delete[] reported_heights_;
  delete[] heights_;
}

HeightGrid &HeightGrid::operator +=(const HeightGrid &other)
{
  for (int x = 0;x < other.width_;++x)
  {
    for (int y = 0;y < other.height_;++y)
    {
      std::tuple<double, double> pos = other.getPosition(x, y);
      reportHeight(std::get<0>(pos), std::get<1>(pos), other.heights_[other.getIndex(x, y)]);
    }
  }
  return *this;
}

void HeightGrid::calculate(void)
{
  int *tags = new int[width_ * height_];
  memset(tags, 0, sizeof(tags[0]) * width_ * height_);

  std::function<void(int, int)> func = [&] (int x, int y) {
      int tag = tags[getIndex(x, y)];
      for (int dx = -1;dx <= 1;++dx)
      {
        for (int dy = -1;dy <= 1;++dy)
        {
          if (!dx && !dy)
            continue;
          int nx = x + dx;
          int ny = y + dy;
          int idx = getIndex(nx, ny);
          if (idx < 0 || tags[idx] || !heights_[idx])
            continue;
          tags[idx] = tag;
          func(nx, ny);
        }
      }
    };

  int tag = 1;
  for (int x = 0;x < width_;++x)
  {
    for (int y = 0;y < height_;++y)
    {
      int idx = getIndex(x, y);
      if (tags[idx])
        continue;
      tags[idx] = tag++;
      if (!heights_[idx])
        continue;
      func(x, y);
    }
  }

  double *max_h = new double[tag];
  memset(max_h, 0, sizeof(max_h[0]) * tag);
  for (int z = 0;z < height_ * width_;++z)
    max_h[tags[z]] = std::max(max_h[tags[z]], reported_heights_[z]);
  for (int z = 0;z < height_ * width_;++z)
    heights_[z] = heights_[z] ? max_h[tags[z]] : 0.0;

  delete[] max_h;
  delete[] tags;
}

void HeightGrid::generatePointCloud(std::vector<Eigen::Vector3d> &points)
{
  for (int x = 0;x < width_;++x)
  {
    for (int y = 0;y < height_;++y)
    {
      double pos_x, pos_y;
      std::tie(pos_x, pos_y) = getPosition(x, y);
      int idx = getIndex(x, y);
      double height = heights_[idx];
      for (double pos_z = 0.0;pos_z <= height;pos_z += resolution_)
        points.push_back(Eigen::Vector3d(pos_x, pos_y, pos_z));
    }
  }

  for (double z = 0.2;z <= 0.6;z += resolution_ / 5)
  {
    double r = z <= 0.35 ? 0.8 * (z - 0.2) : 0.05;
    for (double x = 0.0;x <= r;x += resolution_ / 5)
    {
      for (double y = 0.0;y * y + x * x <= r * r;y += resolution_ / 5)
      {
        points.push_back(Eigen::Vector3d(current_x_ + x, current_y_ + y, z));
        if (y)
          points.push_back(Eigen::Vector3d(current_x_ + x, current_y_ + -y, z));
        if (x)
          points.push_back(Eigen::Vector3d(current_x_ + -x, current_y_ + y, z));
        if (x && y)
          points.push_back(Eigen::Vector3d(current_x_ + -x, current_y_ + -y, z));
      }
    }
  }
}

class PointCloudViewer :public open3d::visualization::Visualizer
{
public:
  PointCloudViewer(const std::string &base_link_frame,
      const std::string &map_topic, const std::vector<std::string> &rgb_img_topic,
      const std::vector<std::string> &depth_img_topic, const std::string &pc_output_topic,
      const std::vector<std::string> &camera_info_topic, ros::NodeHandle &node, int occupied_prob);

  int exec(void);

protected:
  void KeyPressCallback(GLFWwindow *window, int key, int scancode, int action, int mods) override;

private:
  void calcPointCloud(HeightGrid &grid, const std::string &map_frame, double resolution,
                      const sensor_msgs::Image::ConstPtr &depth_img, const sensor_msgs::Image::ConstPtr &rgb_img,
                      const sensor_msgs::CameraInfo::ConstPtr &camera_info);
  void spinThreadMain(void);
  void rgbImageCallback(const sensor_msgs::Image::ConstPtr &image);
  void depthImageCallback(const sensor_msgs::Image::ConstPtr &image);

private:
  std::string base_link_frame_;
  std::string map_topic_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<open3d::geometry::PointCloud> point_cloud_;
  std::vector<Eigen::Vector3d> new_points_;
  std::mutex lock_;

  ros::Publisher pc_pub_;
  std::vector<ros::Subscriber> rgb_img_sub_;
  std::vector<ros::Subscriber> depth_img_sub_;

  std::vector<sensor_msgs::Image::ConstPtr> rgb_img_;
  std::vector<sensor_msgs::Image::ConstPtr> depth_img_;
  std::vector<sensor_msgs::CameraInfo::ConstPtr> camera_info_;

  int occupied_prob_;
};

PointCloudViewer::PointCloudViewer(const std::string &base_link_frame,
      const std::string &map_topic, const std::vector<std::string> &rgb_img_topic,
      const std::vector<std::string> &depth_img_topic, const std::string &pc_output_topic,
      const std::vector<std::string> &camera_info_topic, ros::NodeHandle &node, int occupied_prob)
  : base_link_frame_(base_link_frame),
    map_topic_(map_topic),
    tf_listener_(tf_buffer_),
    point_cloud_(std::make_shared<open3d::geometry::PointCloud>()),
    pc_pub_(node.advertise<sensor_msgs::PointCloud2>(pc_output_topic, 3)),
    occupied_prob_(occupied_prob)
{
  for (int i = 0;i < camera_info_topic.size();++i)
  {
    camera_info_.push_back(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic[i]));
    rgb_img_sub_.push_back(node.subscribe<sensor_msgs::Image>(
                  rgb_img_topic[i], 1, &PointCloudViewer::rgbImageCallback, this));
    depth_img_sub_.push_back(node.subscribe<sensor_msgs::Image>(
                  depth_img_topic[i], 1, &PointCloudViewer::depthImageCallback, this));
  }
  rgb_img_.resize(camera_info_topic.size());
  depth_img_.resize(camera_info_topic.size());
}

int PointCloudViewer::exec(void)
{
  if (!CreateVisualizerWindow("Land Explorer 3D Mapping"))
    return -1;

  AddGeometry(point_cloud_);

  boost::thread::attributes attrs;
  attrs.set_stack_size(1024 * 1024 * 50);
  boost::thread thread(attrs, boost::bind(&PointCloudViewer::spinThreadMain, this));

  while (WaitEvents())
  {
    bool updated = false;
    if (lock_.try_lock()) {
      if (new_points_.size()) {
        point_cloud_->points_.clear();
        for (const Eigen::Vector3d &vec : new_points_)
          point_cloud_->points_.push_back(vec);
        new_points_.clear();
        updated = true;
      }
      lock_.unlock();
    }
    if (updated) {
      UpdateGeometry();
      view_control_ptr_->ResetBoundingBox();
      view_control_ptr_->FitInGeometry(*point_cloud_);
    }
  }

  DestroyVisualizerWindow();
  thread.join();

  return 0;
}

void PointCloudViewer::rgbImageCallback(const sensor_msgs::Image::ConstPtr &image)
{
  for (int i = 0;i < camera_info_.size();++i)
    if (camera_info_[i]->header.frame_id == image->header.frame_id)
      rgb_img_[i] = image;
}

void PointCloudViewer::depthImageCallback(const sensor_msgs::Image::ConstPtr &image)
{
  for (int i = 0;i < camera_info_.size();++i)
    if (camera_info_[i]->header.frame_id == image->header.frame_id)
      depth_img_[i] = image;
}

void PointCloudViewer::calcPointCloud(HeightGrid &grid, const std::string &map_frame, double resolution,
                      const sensor_msgs::Image::ConstPtr &depth_img, const sensor_msgs::Image::ConstPtr &rgb_img,
                      const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(map_frame, depth_img->header.frame_id, depth_img->header.stamp, ros::Duration(0.0));
  } catch (const tf2::ExtrapolationException &) {
    ROS_WARN("Failed to transform from %s frame to %s frame, so give up this update.\n",
                      depth_img->header.frame_id.c_str(), map_frame.c_str());
    return;
  }

  double cx = camera_info->K[2];
  double cy = camera_info->K[5];
  double fx = camera_info->K[0];
  double fy = camera_info->K[4];

  std::unordered_map<std::tuple<int, int, int>, uint32_t, boost::hash<std::tuple<int, int, int>>> map;
  for (int x = 0;x < depth_img->width;++x)
  {
    for (int y = 0;y < depth_img->height;++y)
    {
      float d = *(float *)(&depth_img->data[depth_img->step * y + x * 4]);
      if (std::isnan(d))
        continue;

      geometry_msgs::Point pt, map_pt;
      pt.x = d * ((x - cx) / fx);
      pt.y = d * ((y - cy) / fy);
      pt.z = d;
      tf2::doTransform(pt, map_pt, tf);

      grid.reportHeight(map_pt.x, map_pt.y, map_pt.z);
      uint32_t &rgb = map[std::make_tuple(int(pt.x / resolution), int(pt.y / resolution), int(pt.z / resolution))];
      rgb = *(uint32_t *)(&rgb_img->data[rgb_img->step * y + x * 3]);
    }
  }

  sensor_msgs::PointCloud2 pc;
  pc.header = depth_img->header;
  pc.height = 1;
  pc.width = map.size();

  sensor_msgs::PointCloud2Modifier pc_modifier(pc);
  pc_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pc_modifier.resize(map.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(pc, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(pc, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(pc, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> it_r(pc, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> it_g(pc, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> it_b(pc, "b");
  for (const std::pair<std::tuple<int, int, int>, uint32_t> &pt : map)
  {
    *it_x = std::get<0>(pt.first) * resolution;
    *it_y = std::get<1>(pt.first) * resolution;
    *it_z = std::get<2>(pt.first) * resolution;
    ++it_x;
    ++it_y;
    ++it_z;

    *it_r = (pt.second >>  0) & 0xff;
    *it_g = (pt.second >>  8) & 0xff;
    *it_b = (pt.second >> 16) & 0xff;
    ++it_r;
    ++it_g;
    ++it_b;
  }

  pc_pub_.publish(pc);
  ROS_INFO("Published point cloud which contains %d points to topic %s.", map.size(), pc_pub_.getTopic().c_str());
}

void PointCloudViewer::spinThreadMain(void)
{
  HeightGrid *last, *now;
  last = now = nullptr;

  int id = 0;
  while (!glfwWindowShouldClose(window_))
  {
    ros::spinOnce();
    if (glfwWindowShouldClose(window_))
      break;

    ROS_INFO("Received synchronized RGB image and depth image.");
    ROS_INFO("Start updating... ID: %d", ++id);
    nav_msgs::OccupancyGrid::ConstPtr map =
            ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic_);
    double now_x, now_y;

    try {
      geometry_msgs::TransformStamped tf;
      tf = tf_buffer_.lookupTransform(map->header.frame_id, base_link_frame_, map->header.stamp, ros::Duration(0.0));

      geometry_msgs::Point pt, tmp;
      pt.x = pt.y = pt.z = 0.0;
      tf2::doTransform(pt, tmp, tf);
      now_x = tmp.x;
      now_y = tmp.y;
    } catch (const tf2::ExtrapolationException &) {
      ROS_WARN("Failed to transform from %s frame to %s frame, so give up this update.",
                              base_link_frame_.c_str(), map->header.frame_id.c_str());
      goto next;
    }

    now = new HeightGrid(map, now_x, now_y, occupied_prob_);
    if (last) {
      *now += *last;
      delete last;
      last = nullptr;
    }

    for (int i = 0;i < camera_info_.size();++i)
    {
      if (!depth_img_[i] || !rgb_img_[i])
        continue;
      if (depth_img_[i]->header.stamp != rgb_img_[i]->header.stamp)
        continue;
      calcPointCloud(*now, map->header.frame_id, map->info.resolution, depth_img_[i], rgb_img_[i], camera_info_[i]);
    }
    now->calculate();

    lock_.lock();
    now->generatePointCloud(new_points_);
    lock_.unlock();
    glfwPostEmptyEvent();

next:
    ROS_INFO("No.%d update is ended.\n", id);
    if (glfwWindowShouldClose(window_))
      break;
    last = now;
    now = nullptr;
  }

  if (last)
    delete last;
  if (now)
    delete now;
}

void PointCloudViewer::KeyPressCallback(GLFWwindow *window,
                  int key, int scancode, int action, int mods)
{
  if (window == window_ && key == GLFW_KEY_EQUAL && (mods & GLFW_MOD_ALT) && action == GLFW_PRESS) {
    ++occupied_prob_;
    ROS_INFO("Occupied threshold is set to %d%%.\n", occupied_prob_);
    return;
  }
  if (window == window_ && key == GLFW_KEY_MINUS && (mods & GLFW_MOD_ALT) && action == GLFW_PRESS) {
    --occupied_prob_;
    ROS_INFO("Occupied threshold is set to %d%%.\n", occupied_prob_);
    return;
  }
  Visualizer::KeyPressCallback(window, key, scancode, action, mods);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping_3d");
  ros::NodeHandle n;

  PointCloudViewer viewer("base_link", "/map", {"/camera1/rgb/image_raw", "/camera2/rgb/image_raw"},
                          {"/camera1/depth/image_raw", "/camera2/depth/image_raw"}, "/points2",
                          {"/camera1/rgb/camera_info", "/camera2/rgb/camera_info"}, n, 50);
  return viewer.exec();
}
