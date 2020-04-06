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

    class RvizPolygonsTools: public locomotion_viewer::ThreeDim {

public:

  RvizPolygonsTools(std::string base_frame, 
                  std::string marker_topic, 
                  ros::NodeHandle nh);

  ~RvizPolygonsTools();

  typedef RvizVisualTools RvizVisual;

private:


};

  typedef std::shared_ptr<RvizPolygonsTools> RvizPolygonsToolsPtr;

} // namespace locomotion_viewer

#endif /* DLS_ONLINEID_H_ */
