#include <locomotion_viewer/ThreeDim.h>

namespace locomotion_viewer{

ThreeDim::ThreeDim(std::string base_frame, 
                  std::string marker_topic, 
                  ros::NodeHandle nh):TwoDim(base_frame, marker_topic, nh){

}

ThreeDim::~ThreeDim(){


}

bool ThreeDim::publishHexahedron(const Eigen::Isometry3d& pose,
                                          Eigen::Matrix<double, 3, 8> force_polygon,
                                          bool frame_flag,
                                          rviz_visual_tools::colors color,
                                          double scale)
{
    return publishHexahedron(pose,
                             force_polygon.block(0,0,3,1),
                             force_polygon.block(0,1,3,1),
                             force_polygon.block(0,2,3,1),
                             force_polygon.block(0,3,3,1),
                             force_polygon.block(0,4,3,1),
                             force_polygon.block(0,5,3,1),
                             force_polygon.block(0,6,3,1),
                             force_polygon.block(0,7,3,1),
                             frame_flag,
                             color,
                             scale);
}

bool ThreeDim::publishHexahedron(const Eigen::Isometry3d& pose,
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        Eigen::Vector3d v5,
                                        Eigen::Vector3d v6,
                                        Eigen::Vector3d v7,
                                        Eigen::Vector3d v8,
                                        bool frame_flag,
                                        rviz_visual_tools::colors color,  
                                        double scale)
{
   return publishHexahedron(convertPose(pose), v1, v2, v3, v4, v5, v6, v7, v8, frame_flag, color, scale);
}

bool ThreeDim::publishHexahedron(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        Eigen::Vector3d v5,
                                        Eigen::Vector3d v6,
                                        Eigen::Vector3d v7,
                                        Eigen::Vector3d v8,
                                        bool frame_flag,
                                        rviz_visual_tools::colors color,  
                                        double scale){

  //    // plane 1
  publishQuadrilateral(pose, v1, v2, v3, v4, frame_flag, color, scale);

  //    // plane 2
  publishQuadrilateral(pose, v5, v6, v7, v8, frame_flag, color, scale);

  //  // plane 3
  publishQuadrilateral(pose, v1, v2, v6, v5, frame_flag, color, scale);

  //    // plane 4
  publishQuadrilateral(pose, v2, v3, v7, v6, frame_flag, color, scale);

  //    // plane 5
  publishQuadrilateral(pose, v3, v4, v8, v7, frame_flag, color, scale);

  //    // plane 6
  publishQuadrilateral(pose, v1, v4, v8, v5, frame_flag, color, scale);

  return publishMarker(triangle_marker_);
}

bool ThreeDim::publishCube(const geometry_msgs::Point& center, const double& side_size,
                                    rviz_visual_tools::colors color, const std::string& ns, std::size_t id)
{
  // Set the timestamp
  cuboid_marker_.header.stamp = ros::Time::now();
  cuboid_marker_.ns = ns;

  if (id == 0)
  {  // Provide a new id every call to this function
    cuboid_marker_.id++;
  }
  else
  {  // allow marker to be overwritten
    cuboid_marker_.id = id;
  }

  cuboid_marker_.color = getColor(color);

  // Calculate center pose
  geometry_msgs::Pose pose;
  pose.position.y = center.y;
  pose.position.x = center.x;
  pose.position.z = center.z;
  cuboid_marker_.pose = pose;

  // Calculate scale
  cuboid_marker_.scale.x = side_size;
  cuboid_marker_.scale.y = side_size;
  cuboid_marker_.scale.z = side_size;

  // Prevent scale from being zero
  if (cuboid_marker_.scale.x == 0.0)
  {
    cuboid_marker_.scale.x = rviz_visual_tools::SMALL_SCALE;
  }
  if (cuboid_marker_.scale.y == 0.0)
  {
    cuboid_marker_.scale.y = rviz_visual_tools::SMALL_SCALE;
  }
  if (cuboid_marker_.scale.z == 0.0)
  {
    cuboid_marker_.scale.z = rviz_visual_tools::SMALL_SCALE;
  }

  // Helper for publishing rviz markers
  return publishMarker(cuboid_marker_);
}

bool ThreeDim::publishEigenCube(const Eigen::Vector3d& center,
                                         rviz_visual_tools::colors color,
                                         const double side_size,
                                         const std::string & ns)
{
  return publishCube(convertPoint(center), side_size, color, ns);
}

}  // namespace locomotion_viewer
