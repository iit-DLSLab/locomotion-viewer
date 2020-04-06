//
// Created by Romeo Orsolino on 06/04/2020.
//

#include <locomotion_viewer/ZeroDim.h>

namespace locomotion_viewer {

    ZeroDim::ZeroDim(std::string base_frame,
                   std::string marker_topic,
                   ros::NodeHandle nh) : RvizVisualTools(base_frame, marker_topic, nh) {}

    ZeroDim::~ZeroDim() {}

    bool ZeroDim::publishEigenSphere(Eigen::Vector3d & point,
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

    bool ZeroDim::publishEigenSpheres(Eigen::VectorXd & eigen_path_x,
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


}