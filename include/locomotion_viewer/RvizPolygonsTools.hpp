/*
 * OnlineID.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#ifndef RVIZPOLYGONSTOOLS_H_
#define RVIZPOLYGONSTOOLS_H_

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <chrono>

//namespace rviz_visual_tools{
class RvizPolygonsTools: public rviz_visual_tools::RvizVisualTools {

public:

  RvizPolygonsTools(std::string base_frame, 
                  std::string marker_topic, 
                  ros::NodeHandle nh);

  ~RvizPolygonsTools();

  bool publishEigenPath(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors color = rviz_visual_tools::RED,
                    rviz_visual_tools::scales scale = rviz_visual_tools::MEDIUM,
                    const std::string & ns = "Path");

  bool publishEigenPathWithWayPoints(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors color = rviz_visual_tools::RED,
                    rviz_visual_tools::scales scale = rviz_visual_tools::MEDIUM,
                    const std::string & ns = "Path");   

  bool publishDashedPolygonPerimeter(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    double segmentsLenght = 0.05,
                    rviz_visual_tools::colors lineColor = rviz_visual_tools::BLACK,
                    rviz_visual_tools::scales lineScale = rviz_visual_tools::MEDIUM,
                    const std::string & ns = "Path");

  bool publishPolygonPerimeter(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors color = rviz_visual_tools::RED,
                    rviz_visual_tools::scales scale = rviz_visual_tools::MEDIUM,
                    const std::string & ns = "Path");

  bool publishPolygonWithSurface(Eigen::VectorXd & eigen_path_x,
                    Eigen::VectorXd & eigen_path_y,
                    Eigen::VectorXd & eigen_path_z,
                    rviz_visual_tools::colors color = rviz_visual_tools::RED,
                    rviz_visual_tools::scales scale = rviz_visual_tools::MEDIUM,
                    const std::string & ns = "Path");

  bool publishEigenCube(const Eigen::Vector3d& center, rviz_visual_tools::colors color, const double side_size = 0.05, const std::string & ns = "Cube");

  bool publishCube(const geometry_msgs::Point& center, const double& side_size,
                                      rviz_visual_tools::colors color, const std::string& ns, std::size_t id = 0);

  bool publishEigenSpheres(Eigen::VectorXd & eigen_path_x,
                                              Eigen::VectorXd & eigen_path_y,
                                              Eigen::VectorXd & eigen_path_z,
                                              rviz_visual_tools::colors color,
                                              rviz_visual_tools::scales scale,
                                              const std::string & ns = "Spheres");

  bool publishEigenSphere(Eigen::Vector3d & point,
                          rviz_visual_tools::colors color = rviz_visual_tools::GREEN,
                          rviz_visual_tools::scales scale = rviz_visual_tools::XLARGE,
                          const std::string & ns = "Sphere");

  bool publishTriangle(Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        bool frame_flag = true,
                                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                                        double scale = 1.0);

  bool publishTriangle(const Eigen::Affine3d& pose,
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        bool frame_flag = true,
                                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,  
                                        double scale = 1.0);

  bool publishTriangle(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        bool frame_flag = true,
                                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,  
                                        double scale = 1.0);

  bool publishTriangleFrame(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        rviz_visual_tools::colors color = rviz_visual_tools::BLACK,  
                                        double scale = 0.02,
                                        const std::string & ns = "triangle_frame");
  
  bool publishTriangle(const Eigen::Affine3d& pose, rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT, double scale = 1.0);
  
  bool publishTriangle(const geometry_msgs::Pose& pose, rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT, double scale = 1.0);

  bool publishQuadrilateralFrame(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        rviz_visual_tools::colors color = rviz_visual_tools::BLACK,  
                                        double scale = 0.02,
                                        const std::string & ns = "triangle_frame");

  bool publishQuadrilateral(const Eigen::Affine3d& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        bool frame_flag = true,
                                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,  
                                        double scale = 1.0);

  bool publishQuadrilateral(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        bool frame_flag = true,
                                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,  
                                        double scale = 1.0);

  bool publishHexahedron(const Eigen::Affine3d& pose,
                                            Eigen::Matrix<double, 3, 8> force_polygon,
                                            bool frame_flag = true,
                                            rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                                            double scale = 1.0);

  bool publishHexahedron(const Eigen::Affine3d& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        Eigen::Vector3d v5,
                                        Eigen::Vector3d v6,
                                        Eigen::Vector3d v7,
                                        Eigen::Vector3d v8,
                                        bool frame_flag = true,
                                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,  
                                        double scale = 1.0);

  bool publishHexahedron(const geometry_msgs::Pose& pose, 
                                        Eigen::Vector3d v1,
                                        Eigen::Vector3d v2,
                                        Eigen::Vector3d v3,
                                        Eigen::Vector3d v4,
                                        Eigen::Vector3d v5,
                                        Eigen::Vector3d v6,
                                        Eigen::Vector3d v7,
                                        Eigen::Vector3d v8,
                                        bool frame_flag = true,
                                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,  
                                        double scale = 1.0);

  bool publishDashedLine(Eigen::Vector3d& startingPoint,
                         Eigen::Vector3d& endPoint,
                         double segmentsLenght = 0.1,
                         rviz_visual_tools::colors color = rviz_visual_tools::BLACK,
                         rviz_visual_tools::scales scale = rviz_visual_tools::XLARGE);

  typedef RvizVisualTools RvizVisual;

private:


};

  typedef std::shared_ptr<RvizPolygonsTools> RvizPolygonsToolsPtr;

//} // namespace rviz_visual_tools

#endif /* DLS_ONLINEID_H_ */
