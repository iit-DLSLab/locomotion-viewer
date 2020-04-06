//
// Created by Romeo Orsolino on 07/04/2020.
//

#ifndef LOCOMOTION_VIEWER_TWODIM_H
#define LOCOMOTION_VIEWER_TWODIM_H

#include <locomotion_viewer/OneDim.h>
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <chrono>

namespace locomotion_viewer {

    class TwoDim : public OneDim {

    public:

        TwoDim(std::string base_frame,
                std::string marker_topic,
                ros::NodeHandle nh);

        ~TwoDim();


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

        bool publishTriangle(Eigen::Vector3d v1,
                             Eigen::Vector3d v2,
                             Eigen::Vector3d v3,
                             bool frame_flag = true,
                             rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                             double scale = 1.0);

        bool publishTriangle(const Eigen::Isometry3d& pose,
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

        bool publishTriangle(const Eigen::Isometry3d& pose, rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT, double scale = 1.0);

        bool publishTriangle(const geometry_msgs::Pose& pose, rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT, double scale = 1.0);

        bool publishQuadrilateralFrame(const geometry_msgs::Pose& pose,
                                       Eigen::Vector3d v1,
                                       Eigen::Vector3d v2,
                                       Eigen::Vector3d v3,
                                       Eigen::Vector3d v4,
                                       rviz_visual_tools::colors color = rviz_visual_tools::BLACK,
                                       double scale = 0.02,
                                       const std::string & ns = "triangle_frame");

        bool publishQuadrilateral(const Eigen::Isometry3d& pose,
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

    private:

    };

    typedef std::shared_ptr<TwoDim> TwoDimPtr;

} // locomotion_viewer

#endif //LOCOMOTION_VIEWER_TWODIM_H
