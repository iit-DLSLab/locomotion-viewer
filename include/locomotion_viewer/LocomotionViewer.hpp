/*
 * OnlineID.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#ifndef RVIZPOLYGONSTOOLS_H_
#define RVIZPOLYGONSTOOLS_H_

#include <locomotion_viewer/ThreeDim.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <chrono>

namespace locomotion_viewer{

    using legs_colors      = std::vector<rviz_visual_tools::colors>;

    class LocomotionViewer: public locomotion_viewer::ThreeDim {

public:

  LocomotionViewer(std::string base_frame,
                   std::string marker_topic,
                   ros::NodeHandle nh);

  ~LocomotionViewer();

  legs_colors default_legs_colors_ = {rviz_visual_tools::RED,rviz_visual_tools::GREEN,rviz_visual_tools::BLUE,rviz_visual_tools::YELLOW};
  rviz_visual_tools::colors base_color_;

private:

};

    typedef std::shared_ptr<LocomotionViewer> LocomotionViewerPtr;

} // namespace locomotion_viewer

#endif /* DLS_ONLINEID_H_ */
