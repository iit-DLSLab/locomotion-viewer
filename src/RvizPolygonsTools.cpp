#include <locomotion_viewer/RvizPolygonsTools.hpp>

namespace locomotion_viewer{

RvizPolygonsTools::RvizPolygonsTools(std::string base_frame, 
                  std::string marker_topic, 
                  ros::NodeHandle nh):ThreeDim(base_frame, marker_topic, nh){

}

RvizPolygonsTools::~RvizPolygonsTools(){


}


}  // namespace locomotion_viewer
