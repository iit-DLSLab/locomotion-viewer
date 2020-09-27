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

        double Point2isRightOfLine(const Eigen::Vector3d p0,
                                          const Eigen::Vector3d p1,
                                          const Eigen::Vector3d p2)
        {
          return (p2(1) - p0(1)) * (p1(2) - p0(2))
                 - (p1(1) - p0(1)) * (p2(2) - p0(2));
        }

        void ClockwiseSort(std::vector<Eigen::Vector3d>& p)
        {

          // sort clockwise
          for (int i = 1; i < p.size() - 1; i++) {
            for (int j = i + 1; j < p.size(); j++) {
              //the point p2 should always be on the right to be cwise thus if it
              //is on the left <0 i swap
              if (Point2isRightOfLine(p[0], p[i], p[j])  < 0.0) {
                Eigen::Vector3d tmp = p[i];
                p[i] = p[j];
                p[j] = tmp;
              }
            }
          }
        }

    private:

    };

    typedef std::shared_ptr<ZeroDim> ZeroDimPtr;

} // locomotion_viewer

#endif //LOCOMOTION_VIEWER_ZERODIM_H
