#include <locomotion_viewer/RvizPolygonsTools.hpp>

//namespace rviz_visual_tools{

RvizPolygonsTools::RvizPolygonsTools(std::string base_frame, 
                  std::string marker_topic, 
                  ros::NodeHandle nh):RvizVisualTools(base_frame, marker_topic, nh){

}

RvizPolygonsTools::~RvizPolygonsTools(){


}

bool RvizPolygonsTools::publishEigenPath(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors color,
                    rviz_visual_tools::scales scale,
                    const std::string & ns){

  geometry_msgs::Point temp;
  std::vector<geometry_msgs::Point> trajectory;
  int points_num = eigen_path_x.rows();

  for (std::size_t i = 0; i < points_num; ++i)
    {
      temp.x = eigen_path_x(i);
      temp.y = eigen_path_y(i);
      temp.z = eigen_path_z(i);

      trajectory.push_back(temp);
    }

  //publishSphere(first, rviz_visual_tools::RED, rviz_visual_tools::XXXXLARGE, "initial_point");
  //publishSpheres(trajectory, color, rviz_visual_tools::XXXLARGE, "intermediate_points");
  publishPath(trajectory, color, scale, ns);
}

bool RvizPolygonsTools::publishEigenPathWithWayPoints(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors color,
                    rviz_visual_tools::scales scale,
                    const std::string & ns){

  geometry_msgs::Point temp;
  geometry_msgs::Point first;
  std::vector<geometry_msgs::Point> trajectory;
  int points_num = eigen_path_x.rows();

  for (std::size_t i = 0; i < points_num; ++i)
    {
      temp.x = eigen_path_x(i);
      temp.y = eigen_path_y(i);
      temp.z = eigen_path_z(i);

    if (i == 0)
      {
        first = temp;
      }

      trajectory.push_back(temp);
    }

  rviz_visual_tools::scales points_size;
  if (static_cast<rviz_visual_tools::scales>(scale) < 11){
      points_size = static_cast<rviz_visual_tools::scales>(scale + 1);
  }else{
      points_size = scale;
  }

  publishSphere(first, color, points_size, "initial_point");
  publishSpheres(trajectory, color, points_size, "intermediate_points");
  publishPath(trajectory, color, scale, ns);
}

bool RvizPolygonsTools::publishPolygonPerimeter(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors color,
                    rviz_visual_tools::scales scale,
                    const std::string & ns){

  geometry_msgs::Point temp;
  geometry_msgs::Point first;
  std::vector<geometry_msgs::Point> trajectory;
  int points_num = eigen_path_x.rows();

  for (std::size_t i = 0; i < points_num; ++i)
    {
      temp.x = eigen_path_x(i);
      temp.y = eigen_path_y(i);
      temp.z = eigen_path_z(i);

      if (i == 0)
      {
        first = temp;
      }

      trajectory.push_back(temp);
    }

  trajectory.push_back(first);

  //publishSphere(first, rviz_visual_tools::RED, rviz_visual_tools::XXXXLARGE, "initial_point");
  //publishSpheres(trajectory, color, rviz_visual_tools::XXXLARGE, "intermediate_points");
  publishPath(trajectory, color, scale, ns);
}

bool RvizPolygonsTools::publishPolygonWithSurface(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors surface_color,
                    rviz_visual_tools::scales scale,
                    const std::string & ns){

  rviz_visual_tools::colors edge_color = rviz_visual_tools::colors::BLACK;
  geometry_msgs::Point current;
  geometry_msgs::Point first;
  //Eigen::Vector3d next;
  Eigen::Vector3d average;
  std::vector<geometry_msgs::Point> trajectory;
  int points_num = eigen_path_x.rows();
  average(0) = 0.0;
  average(1) = 0.0;
  average(2) = 0.0;

  for (std::size_t i = 0; i < points_num; ++i)
    {
      current.x = eigen_path_x(i);
      current.y = eigen_path_y(i);
      current.z = eigen_path_z(i);

      average(0) += eigen_path_x(i);
      average(1) += eigen_path_y(i);
      average(2) += eigen_path_z(i);

      if (i == 0)
      {
        first = current;
      }

      trajectory.push_back(current);
    }

  average(0) /= (double)points_num;
  average(1) /= (double)points_num;
  average(2) /= (double)points_num;
  //std::cout<<"average "<<average.transpose()<<std::endl;
  //std::cout<<"points num "<<points_num<<std::endl;
  //std::cout<<"path x "<<eigen_path_x.transpose()<<std::endl;
  //std::cout<<"path y "<<eigen_path_y.transpose()<<std::endl;
  //std::cout<<"path z "<<eigen_path_z.transpose()<<std::endl;
  trajectory.push_back(first);

  for (int i = 0; i < points_num-1; ++i)
  {
      Eigen::Vector3d temp;
      Eigen::Vector3d next;
      temp(0) = eigen_path_x(i);
      temp(1) = eigen_path_y(i);
      temp(2) = eigen_path_z(i);
      next(0) = eigen_path_x(i+1);
      next(1) = eigen_path_y(i+1);
      next(2) = eigen_path_z(i+1);
      //std::cout<<"tmp "<<temp.transpose()<<std::endl;
      //std::cout<<next.transpose()<<std::endl;
      publishTriangle(average, temp, next, false, surface_color);
  }

  publishPath(trajectory, edge_color, scale, ns);
}

bool RvizPolygonsTools::publishEigenSphere(Eigen::Vector3d & point,
                                            rviz_visual_tools::colors color,
                                            rviz_visual_tools::scales scale,
                                            const std::string & ns)
{
    geometry_msgs::Point temp;
    temp.x = point(0);
    temp.y = point(1);
    temp.z = point(2);

    publishSphere(temp, color, scale, "com");
}

bool RvizPolygonsTools::publishEigenSpheres(Eigen::VectorXd & eigen_path_x,
                                            Eigen::VectorXd & eigen_path_y,
                                            Eigen::VectorXd & eigen_path_z,
                                            rviz_visual_tools::colors color,
                                            rviz_visual_tools::scales scale,
                                            const std::string & ns)
{

  geometry_msgs::Point temp;
  geometry_msgs::Point first;
  std::vector<geometry_msgs::Point> trajectory;
  int points_num = eigen_path_x.rows();

  for (std::size_t i = 0; i < points_num; ++i)
    {
      temp.x = eigen_path_x(i);
      temp.y = eigen_path_y(i);
      temp.z = eigen_path_z(i);

    if (i == 0)
      {
        first = temp;
      }

      trajectory.push_back(temp);
    }
  publishSpheres(trajectory, color, scale, "intermediate_points");
}

bool RvizPolygonsTools::publishTriangle(const Eigen::Isometry3d& pose, rviz_visual_tools::colors color, double scale)
{
    geometry_msgs::Pose geom_pose;
    geom_pose = convertPose(pose);
   return publishTriangle(geom_pose, color, scale);
}

bool RvizPolygonsTools::publishTriangle(const geometry_msgs::Pose& pose, rviz_visual_tools::colors color, double scale){
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[3];
  p[0].x = 1.0 * scale;
  p[0].y = 0.0 * scale;
  p[0].z = 0.0;

  p[1].x = 0.0 * scale;
  p[1].y = -1.0 * scale;
  p[1].z = 0.0;

  p[2].x = 0.0 * scale;
  p[2].y = 1.0 * scale;
  p[2].z = 0.0;

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.push_back(p[0]);
  triangle_marker_.points.push_back(p[1]);
  triangle_marker_.points.push_back(p[2]);


  return publishMarker(triangle_marker_);
}

bool RvizPolygonsTools::publishTriangle(Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        bool frame_flag,
                                        rviz_visual_tools::colors color,
                                        double scale)
{
    Eigen::Isometry3d pose;
    pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY());
    pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0); // translate x,y,z
    return publishTriangle(convertPose(pose), v1, v2, v3, frame_flag, color, scale);
}

bool RvizPolygonsTools::publishTriangle(const Eigen::Isometry3d& pose,
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        bool frame_flag,
                                        rviz_visual_tools::colors color,  
                                        double scale)
{
   return publishTriangle(convertPose(pose), v1, v2, v3, frame_flag, color, scale);
}

bool RvizPolygonsTools::publishTriangle(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        bool frame_flag,
                                        rviz_visual_tools::colors color,  
                                        double scale){
  if(frame_flag){
    publishTriangleFrame(pose, v1, v2, v3);
  }

  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[3];
  p[0].x = v1(0) * scale;
  p[0].y = v1(1) * scale;
  p[0].z = v1(2);

  p[1].x = v2(0) * scale;
  p[1].y = v2(1) * scale;
  p[1].z = v2(2);

  p[2].x = v3(0) * scale;
  p[2].y = v3(1) * scale;
  p[2].z = v3(2);

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.push_back(p[0]);
  triangle_marker_.points.push_back(p[1]);
  triangle_marker_.points.push_back(p[2]);


  return publishMarker(triangle_marker_);
}

bool RvizPolygonsTools::publishTriangleFrame(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        rviz_visual_tools::colors color,  
                                        double scale,
                                        const std::string & ns){
  std::vector<geometry_msgs::Point> points;

  geometry_msgs::Point p[4];
  p[0].x = v1(0) + pose.position.x;
  p[0].y = v1(1) + pose.position.y;
  p[0].z = v1(2) + pose.position.z;
  points.push_back(p[0]);

  p[1].x = v2(0) + pose.position.x;
  p[1].y = v2(1) + pose.position.y;
  p[1].z = v2(2) + pose.position.z;
  points.push_back(p[1]);

  p[2].x = v3(0) + pose.position.x;
  p[2].y = v3(1) + pose.position.y;
  p[2].z = v3(2) + pose.position.z;
  points.push_back(p[2]);

  p[3].x = v1(0) + pose.position.x;
  p[3].y = v1(1) + pose.position.y;
  p[3].z = v1(2) + pose.position.z;
  points.push_back(p[3]);

  return publishPath(points, color, scale, ns);
}

bool RvizPolygonsTools::publishQuadrilateral(const Eigen::Isometry3d& pose,
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        bool frame_flag,
                                        rviz_visual_tools::colors color,  
                                        double scale)
{
   return publishQuadrilateral(convertPose(pose), v1, v2, v3, v4, frame_flag, color, scale);
}

bool RvizPolygonsTools::publishQuadrilateral(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        bool frame_flag,
                                        rviz_visual_tools::colors color,  
                                        double scale){

  if(frame_flag){
    publishQuadrilateralFrame(pose, v1, v2, v3, v4);
  }

  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[4];

  p[0].x = v1(0) * scale;
  p[0].y = v1(1) * scale;
  p[0].z = v1(2);

  p[1].x = v2(0) * scale;
  p[1].y = v2(1) * scale;
  p[1].z = v2(2);

  p[2].x = v3(0) * scale;
  p[2].y = v3(1) * scale;
  p[2].z = v3(2);

  p[3].x = v4(0) * scale;
  p[3].y = v4(1) * scale;
  p[3].z = v4(2);

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.push_back(p[0]);
  triangle_marker_.points.push_back(p[1]);
  triangle_marker_.points.push_back(p[2]);

  triangle_marker_.points.push_back(p[2]);
  triangle_marker_.points.push_back(p[3]);
  triangle_marker_.points.push_back(p[0]);

  return publishMarker(triangle_marker_);
}

bool RvizPolygonsTools::publishQuadrilateralFrame(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        rviz_visual_tools::colors color,  
                                        double scale,
                                        const std::string & ns){
  std::vector<geometry_msgs::Point> points;

  geometry_msgs::Point p[5];
  p[0].x = v1(0) + pose.position.x;
  p[0].y = v1(1) + pose.position.y;
  p[0].z = v1(2) + pose.position.z;
  points.push_back(p[0]);

  p[1].x = v2(0) + pose.position.x;
  p[1].y = v2(1) + pose.position.y;
  p[1].z = v2(2) + pose.position.z;
  points.push_back(p[1]);

  p[2].x = v3(0) + pose.position.x;
  p[2].y = v3(1) + pose.position.y;
  p[2].z = v3(2) + pose.position.z;
  points.push_back(p[2]);

  p[3].x = v4(0) + pose.position.x;
  p[3].y = v4(1) + pose.position.y;
  p[3].z = v4(2) + pose.position.z;
  points.push_back(p[3]);

  p[4].x = v1(0) + pose.position.x;
  p[4].y = v1(1) + pose.position.y;
  p[4].z = v1(2) + pose.position.z;
  points.push_back(p[4]);

  return publishPath(points, color, scale, ns);
}

bool RvizPolygonsTools::publishHexahedron(const Eigen::Isometry3d& pose,
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

bool RvizPolygonsTools::publishHexahedron(const Eigen::Isometry3d& pose,
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

bool RvizPolygonsTools::publishHexahedron(const geometry_msgs::Pose& pose, 
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

bool RvizPolygonsTools::publishDashedLine(Eigen::Vector3d& startingPoint,
                                          Eigen::Vector3d& endPoint,
                                          double segmentsLenght,
                                          rviz_visual_tools::colors color,
                                          rviz_visual_tools::scales scale){
    //std::cout<<"start building dashed line"<<std::endl;
    double distance;
    distance = sqrt((startingPoint(0) - endPoint(0))*(startingPoint(0) - endPoint(0)) +
                    (startingPoint(1) - endPoint(1))*(startingPoint(1) - endPoint(1)) +
                    (startingPoint(2) - endPoint(2))*(startingPoint(2) - endPoint(2)));
    Eigen::Vector3d direction = endPoint - startingPoint;
    direction.normalize();

    //std::cout<<"distance = "<<distance<<std::endl;
    Eigen::Vector3d start, end;
    start = startingPoint;
    end = startingPoint;
    end += direction*segmentsLenght;
    int numberOfSegments = ceil(distance/(2.0*segmentsLenght));

    for (int i = 0; i<numberOfSegments; i++){
        start = startingPoint;
        start += 2.0*double(i)*direction*segmentsLenght;

        end = start;
        end += direction*segmentsLenght;
        double tmpDistance = sqrt((startingPoint(0) - end(0))*(startingPoint(0) - end(0)) +
                                  (startingPoint(1) - end(1))*(startingPoint(1) - end(1)) +
                                  (startingPoint(2) - end(2))*(startingPoint(2) - end(2)));
        if (tmpDistance>distance){
            end = endPoint;
        }
        const Eigen::Vector3d p1 = start;
        const Eigen::Vector3d p2 = end;
        publishCylinder(p1, p2, color, scale);
    }
    //std::cout<<"finished building dashed line"<<std::endl;
    return true;
}

bool RvizPolygonsTools::publishDashedPolygonPerimeter(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    double segmentsLenght,
                    rviz_visual_tools::colors lineColor,
                    rviz_visual_tools::scales lineScale,
                    const std::string & ns){

  Eigen::Vector3d current, next;
  int points_num = eigen_path_x.rows();

  for (std::size_t i = 0; i < points_num; i++)
    {
      current.setZero(); next.setZero();
      current(0) = eigen_path_x(i);
      current(1) = eigen_path_y(i);
      current(2) = eigen_path_z(i);

      if (i == points_num-1)
      {
          next(0) = eigen_path_x(0);
          next(1) = eigen_path_y(0);
          next(2) = eigen_path_z(0);
      }else{
          next(0) = eigen_path_x(i+1);
          next(1) = eigen_path_y(i+1);
          next(2) = eigen_path_z(i+1);
      }

    publishDashedLine(current, next, segmentsLenght, lineColor, lineScale);

  }

}

bool RvizPolygonsTools::publishCube(const geometry_msgs::Point& center, const double& side_size,
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

bool RvizPolygonsTools::publishEigenCube(const Eigen::Vector3d& center,
                                         rviz_visual_tools::colors color,
                                         const double side_size,
                                         const std::string & ns)
{
  return publishCube(convertPoint(center), side_size, color, ns);
}

//}  // namespace rviz_visual_tools
