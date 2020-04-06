/*
 * OnlineID.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#ifndef LOCOMOTION_VIEWER_THREEDIM_H
#define LOCOMOTION_VIEWER_THREEDIM_H

#include <locomotion_viewer/TwoDim.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <chrono>

namespace locomotion_viewer {

    class ThreeDim : public locomotion_viewer::TwoDim {

    public:

        ThreeDim(std::string base_frame,
                 std::string marker_topic,
                 ros::NodeHandle nh);

        ~ThreeDim();

        bool
        publishEigenCube(const Eigen::Vector3d &center, rviz_visual_tools::colors color, const double side_size = 0.05,
                         const std::string &ns = "Cube");

        bool publishCube(const geometry_msgs::Point &center, const double &side_size,
                         rviz_visual_tools::colors color, const std::string &ns, std::size_t id = 0);

        bool publishHexahedron(const Eigen::Isometry3d &pose,
                               Eigen::Matrix<double, 3, 8> force_polygon,
                               bool frame_flag = true,
                               rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                               double scale = 1.0);


        bool publishHexahedron(const Eigen::Isometry3d &pose,
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

        bool publishHexahedron(const geometry_msgs::Pose &pose,
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

        typedef std::shared_ptr<ThreeDim> ThreeDimPtr;

    private:

    };

} // locomotion_viewer

#endif /* LOCOMOTION_VIEWER_THREEDIM_H */
