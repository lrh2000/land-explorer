#! /bin/sh

urdf_filename=`mktemp`
bag_filenames=subset.bag
pose_graph_filename=land_explorer.pbstream

rosrun xacro xacro `rospack find turtlebot3_description`/urdf/turtlebot3_waffle.urdf.xacro > $urdf_filename
roslaunch cartographer_ros assets_writer_backpack_2d.launch urdf_filename:=$urdf_filename bag_filenames:=$bag_filenames pose_graph_filename:=$pose_graph_filename

cd ~/.ros
rosrun land_explorer process_ply
rosrun land_explorer draw_ply
