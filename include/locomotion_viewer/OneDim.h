//
// Created by Romeo Orsolino on 06/04/2020.
//

#ifndef LOCOMOTION_VIEWER_ONEDIM_H
#define LOCOMOTION_VIEWER_ONEDIM_H

#include <locomotion_viewer/ZeroDim.h>
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <chrono>

namespace locomotion_viewer {

    class OneDim : public ZeroDim {

    public:

        OneDim(std::string base_frame,
               std::string marker_topic,
               ros::NodeHandle nh);

        ~OneDim();

        bool publishEigenPath(Eigen::VectorXd &eigen_path_x,
                              Eigen::VectorXd &eigen_path_y,
                              Eigen::VectorXd &eigen_path_z,
                              rviz_visual_tools::colors color = rviz_visual_tools::RED,
                              rviz_visual_tools::scales scale = rviz_visual_tools::MEDIUM,
                              const double & dashed_line = false,
                              const double & decimation_factor = 1.0,
                              const std::string &ns = "Path");

        bool publishEigenPathWithWayPoints(Eigen::VectorXd &eigen_path_x,
                                           Eigen::VectorXd &eigen_path_y,
                                           Eigen::VectorXd &eigen_path_z,
                                           rviz_visual_tools::colors color = rviz_visual_tools::RED,
                                           rviz_visual_tools::scales scale = rviz_visual_tools::MEDIUM,
                                           const std::string &ns = "Path");

        bool publishDashedEigenPath(Eigen::VectorXd &eigen_path_x,
                                    Eigen::VectorXd &eigen_path_y,
                                    Eigen::VectorXd &eigen_path_z,
                                    rviz_visual_tools::colors lineColor = rviz_visual_tools::BLACK,
                                    rviz_visual_tools::scales lineScale = rviz_visual_tools::MEDIUM,
                                    const double & decimation_factor = 1.0,
                                    const std::string &ns = "Path");

        bool publishDashedEigenPath(Eigen::VectorXd &eigen_path_x,
                                    Eigen::VectorXd &eigen_path_y,
                                    Eigen::VectorXd &eigen_path_z,
                                    double segmentsLenght = 0.05,
                                    rviz_visual_tools::colors lineColor = rviz_visual_tools::BLACK,
                                    rviz_visual_tools::scales lineScale = rviz_visual_tools::MEDIUM,
                                    const std::string &ns = "Path");

        bool publishDashedLine(Eigen::Vector3d &startingPoint,
                               Eigen::Vector3d &endPoint,
                               rviz_visual_tools::colors color = rviz_visual_tools::BLACK,
                               rviz_visual_tools::scales scale = rviz_visual_tools::XLARGE);

        bool publishDashedLine(Eigen::Vector3d &startingPoint,
                               Eigen::Vector3d &endPoint,
                               double segmentsLenght,
                               rviz_visual_tools::colors color = rviz_visual_tools::BLACK,
                               rviz_visual_tools::scales scale = rviz_visual_tools::XLARGE);

        typedef RvizVisualTools RvizVisual;

    private:


    };

    typedef std::shared_ptr<OneDim> OneDimPtr;

} // locomotion_viewer

#endif //LOCOMOTION_VIEWER_ONEDIM_H
