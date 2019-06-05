#include <cstdio>
#include <Open3D/IO/ClassIO/PointCloudIO.h>
#include <Open3D/Visualization/Utility/DrawGeometry.h>

int main(int argc, char **argv)
{
  const char *input;

  if (argc > 2) {
    printf("ERROR:  Unknow command parameters.\n");
    printf("Usage: %s [input_filename]\n", argv[0]);
    return 1;
  }
  if (argc >= 2)
    printf("INFO: Change input filename to '%s'.\n", input = argv[1]);
  else
    printf("INFO: Default input filename is '%s'.\n", input = "result.ply");
  puts("");

  std::shared_ptr<open3d::geometry::PointCloud> pc = open3d::io::CreatePointCloudFromFile(input);
  if (!pc || !pc->points_.size()) {
    printf("ERROR: Failed to read input file as point cloud file.\n");
    return 1;
  }

  open3d::visualization::DrawGeometries({pc});
  return 0;
}
