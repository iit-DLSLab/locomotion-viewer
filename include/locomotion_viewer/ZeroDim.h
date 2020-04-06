//
// Created by Romeo Orsolino on 06/04/2020.
//

#ifndef LOCOMOTION_VIEWER_ZERODIM_H
#define LOCOMOTION_VIEWER_ZERODIM_H

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <chrono>

namespace locomotion_viewer {

    class ZeroDim : public rviz_visual_tools::RvizVisualTools {

    public:

        ZeroDim(std::string base_frame,
               std::string marker_topic,
               ros::NodeHandle nh);

        ~ZeroDim();


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

    private:

    };

    typedef std::shared_ptr<ZeroDim> ZeroDimPtr;

} // locomotion_viewer

#endif //LOCOMOTION_VIEWER_ZERODIM_H
