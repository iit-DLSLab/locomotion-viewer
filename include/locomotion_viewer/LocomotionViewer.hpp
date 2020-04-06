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

    class LocomotionViewer: public locomotion_viewer::ThreeDim {

public:

  LocomotionViewer(std::string base_frame,
                   std::string marker_topic,
                   ros::NodeHandle nh);

  ~LocomotionViewer();

private:

};

    typedef std::shared_ptr<LocomotionViewer> LocomotionViewerPtr;

} // namespace locomotion_viewer

#endif /* DLS_ONLINEID_H_ */
