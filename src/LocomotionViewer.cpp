#include <locomotion_viewer/LocomotionViewer.hpp>

namespace locomotion_viewer{

LocomotionViewer::LocomotionViewer(std::string base_frame,
                                   std::string marker_topic,
                                   ros::NodeHandle nh):ThreeDim(base_frame, marker_topic, nh){
}

LocomotionViewer::~LocomotionViewer(){
}


}  // namespace locomotion_viewer
