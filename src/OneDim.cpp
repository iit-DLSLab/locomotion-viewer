//
// Created by Romeo Orsolino on 06/04/2020.
//

#include <locomotion_viewer/OneDim.h>

namespace locomotion_viewer {

    OneDim::OneDim(std::string base_frame,
                   std::string marker_topic,
                   ros::NodeHandle nh) : ZeroDim(base_frame, marker_topic, nh) {}

    OneDim::~OneDim() {}

    bool OneDim::publishEigenPath(Eigen::VectorXd &eigen_path_x,
                                  Eigen::VectorXd &eigen_path_y,
                                  Eigen::VectorXd &eigen_path_z,
                                  rviz_visual_tools::colors color,
                                  rviz_visual_tools::scales scale,
                                  const double & dashed_line,
                                  const std::string &ns) {

      if(dashed_line){
        publishDashedEigenPath(eigen_path_x, eigen_path_y, eigen_path_z,  color, scale);
      }else{
        geometry_msgs::Point temp;
        std::vector<geometry_msgs::Point> trajectory;
        int points_num = eigen_path_x.rows();

        for (std::size_t i = 0; i < points_num; ++i) {
          temp.x = eigen_path_x(i);
          temp.y = eigen_path_y(i);
          temp.z = eigen_path_z(i);

          trajectory.push_back(temp);
        }
        //publishSphere(first, rviz_visual_tools::RED, rviz_visual_tools::XXXXLARGE, "initial_point");
        //publishSpheres(trajectory, color, rviz_visual_tools::XXXLARGE, "intermediate_points");
        publishPath(trajectory, color, scale, ns);
      }
    }

    bool OneDim::publishEigenPathWithWayPoints(Eigen::VectorXd &eigen_path_x,
                                               Eigen::VectorXd &eigen_path_y,
                                               Eigen::VectorXd &eigen_path_z,
                                               rviz_visual_tools::colors color,
                                               rviz_visual_tools::scales scale,
                                               const std::string &ns) {

        geometry_msgs::Point temp;
        geometry_msgs::Point first;
        std::vector<geometry_msgs::Point> trajectory;
        int points_num = eigen_path_x.rows();

        for (std::size_t i = 0; i < points_num; ++i) {
            temp.x = eigen_path_x(i);
            temp.y = eigen_path_y(i);
            temp.z = eigen_path_z(i);

            if (i == 0) {
                first = temp;
            }

            trajectory.push_back(temp);
        }

        rviz_visual_tools::scales points_size;
        if (static_cast<rviz_visual_tools::scales>(scale) < 11) {
            points_size = static_cast<rviz_visual_tools::scales>(scale + 1);
        } else {
            points_size = scale;
        }

        publishSphere(first, color, points_size, "initial_point");
        publishSpheres(trajectory, color, points_size, "intermediate_points");
        publishPath(trajectory, color, scale, ns);
    }

    bool OneDim::publishDashedLine(Eigen::Vector3d &startingPoint,
                                   Eigen::Vector3d &endPoint,
                                   rviz_visual_tools::colors color,
                                   rviz_visual_tools::scales scale) {
        //std::cout<<"start building dashed line"<<std::endl;
        double distance = sqrt((startingPoint(0) - endPoint(0)) * (startingPoint(0) - endPoint(0)) +
                               (startingPoint(1) - endPoint(1)) * (startingPoint(1) - endPoint(1)) +
                               (startingPoint(2) - endPoint(2)) * (startingPoint(2) - endPoint(2)));

        double segmentsLenght = distance / 2.0;
        Eigen::Vector3d direction = endPoint - startingPoint;
        direction.normalize();

        //std::cout<<"distance = "<<distance<<std::endl;
        Eigen::Vector3d start, end;
        start = startingPoint;
        end = startingPoint;
        end += direction * segmentsLenght;
        int numberOfSegments = ceil(distance / (2.0 * segmentsLenght));

        for (int i = 0; i < numberOfSegments; i++) {
            start = startingPoint;
            start += 2.0 * double(i) * direction * segmentsLenght;

            end = start;
            end += direction * segmentsLenght;
            double tmpDistance = sqrt((startingPoint(0) - end(0)) * (startingPoint(0) - end(0)) +
                                      (startingPoint(1) - end(1)) * (startingPoint(1) - end(1)) +
                                      (startingPoint(2) - end(2)) * (startingPoint(2) - end(2)));
            if (tmpDistance > distance) {
                end = endPoint;
            }
            const Eigen::Vector3d p1 = start;
            const Eigen::Vector3d p2 = end;
            publishCylinder(p1, p2, color, scale);
        }
        //std::cout<<"finished building dashed line"<<std::endl;
        return true;
    }

    bool OneDim::publishDashedLine(Eigen::Vector3d &startingPoint,
                                   Eigen::Vector3d &endPoint,
                                   double segmentsLenght,
                                   rviz_visual_tools::colors color,
                                   rviz_visual_tools::scales scale) {
        //std::cout<<"start building dashed line"<<std::endl;
        double distance;
        distance = sqrt((startingPoint(0) - endPoint(0)) * (startingPoint(0) - endPoint(0)) +
                        (startingPoint(1) - endPoint(1)) * (startingPoint(1) - endPoint(1)) +
                        (startingPoint(2) - endPoint(2)) * (startingPoint(2) - endPoint(2)));
        Eigen::Vector3d direction = endPoint - startingPoint;
        direction.normalize();

        //std::cout<<"distance = "<<distance<<std::endl;
        Eigen::Vector3d start, end;
        start = startingPoint;
        end = startingPoint;
        end += direction * segmentsLenght;
        int numberOfSegments = ceil(distance / (2.0 * segmentsLenght));

        for (int i = 0; i < numberOfSegments; i++) {
            start = startingPoint;
            start += 2.0 * double(i) * direction * segmentsLenght;

            end = start;
            end += direction * segmentsLenght;
            double tmpDistance = sqrt((startingPoint(0) - end(0)) * (startingPoint(0) - end(0)) +
                                      (startingPoint(1) - end(1)) * (startingPoint(1) - end(1)) +
                                      (startingPoint(2) - end(2)) * (startingPoint(2) - end(2)));
            if (tmpDistance > distance) {
                end = endPoint;
            }
            const Eigen::Vector3d p1 = start;
            const Eigen::Vector3d p2 = end;
            publishCylinder(p1, p2, color, scale);
        }
        //std::cout<<"finished building dashed line"<<std::endl;
        return true;
    }

    bool OneDim::publishDashedEigenPath(Eigen::VectorXd &eigen_path_x,
                                        Eigen::VectorXd &eigen_path_y,
                                        Eigen::VectorXd &eigen_path_z,
                                        rviz_visual_tools::colors lineColor,
                                        rviz_visual_tools::scales lineScale,
                                        const std::string &ns) {

        Eigen::Vector3d current, next;
        int points_num = eigen_path_x.rows();

        for (std::size_t i = 0; i < points_num - 1; i++) {
            current.setZero();
            next.setZero();
            current(0) = eigen_path_x(i);
            current(1) = eigen_path_y(i);
            current(2) = eigen_path_z(i);

            next(0) = eigen_path_x(i + 1);
            next(1) = eigen_path_y(i + 1);
            next(2) = eigen_path_z(i + 1);

            publishDashedLine(current, next, lineColor, lineScale);

        }

    }

    bool OneDim::publishDashedEigenPath(Eigen::VectorXd &eigen_path_x,
                                        Eigen::VectorXd &eigen_path_y,
                                        Eigen::VectorXd &eigen_path_z,
                                        double segmentsLength,
                                        rviz_visual_tools::colors lineColor,
                                        rviz_visual_tools::scales lineScale,
                                        const std::string &ns) {

        Eigen::Vector3d current, next;
        int points_num = eigen_path_x.rows();

        for (std::size_t i = 0; i < points_num - 1; i++) {
            current.setZero();
            next.setZero();
            current(0) = eigen_path_x(i);
            current(1) = eigen_path_y(i);
            current(2) = eigen_path_z(i);

            next(0) = eigen_path_x(i + 1);
            next(1) = eigen_path_y(i + 1);
            next(2) = eigen_path_z(i + 1);

            publishDashedLine(current, next, segmentsLength, lineColor, lineScale);

        }

    }

}