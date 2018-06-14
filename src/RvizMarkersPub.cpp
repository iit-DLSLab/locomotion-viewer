/*
 * RvizMarkersPub.cpp
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#include "../include/motion_viewer/RvizMarkersPub.h"

RvizMarkersPub::RvizMarkersPub(const unsigned int cycles)
{
  cycles_n_ = cycles;
}


RvizMarkersPub::~RvizMarkersPub(){

}

void RvizMarkersPub::set_markers(const unsigned int marker_id, 
                    const char* marker_namespace,
                    const Eigen::Vector3d & values,
                    double weight,
                    visualization_msgs::Marker & marker){

  set_markers(marker_id, marker_namespace, values, marker);

    // Set the scale of the com -- 1x1x1 here means 1m on a side
  if(weight<0.01) weight = 0.01;
  marker.scale.x = weight;
  marker.scale.y = weight;
  marker.scale.z = weight;

}

void RvizMarkersPub::set_markers(const unsigned int marker_id, 
                                 const char* marker_namespace,
                                 const Eigen::Vector3d & values,
                                 visualization_msgs::Marker & marker){ 
  // Publish the value of the measured CoM position
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any com sent with the same namespace and id will overwrite the old one
    //std::cout<<"name space: "<<marker_namespace<<std::endl;
    marker.ns = marker_namespace;
    marker.id = marker_id;

    // Set the com type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the com action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD; 

    // Set the pose of the com.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = values(0);
    marker.pose.position.y = values(1);
    marker.pose.position.z = values(2);
    //std::cout<<predicted_particles_(j,0)<<" "<<predicted_particles_(j,0)<<std::endl;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the com -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.lifetime = ros::Duration();
  //
}

//void RvizMarkersPub::set_triangle_markers(const unsigned int marker_id, 
//                                    const char* marker_nm_triangles,
//                                    const Geometry::Triangle3d triangle,
//                                    visualization_msgs::Marker & triangle_marker){
//
//      set_triangle_markers(marker_id, marker_nm_triangles, triangle.v1, triangle_marker);
//      set_triangle_markers(marker_id, marker_nm_triangles, triangle.v2, triangle_marker);
//      set_triangle_markers(marker_id, marker_nm_triangles, triangle.v3, triangle_marker);
//}

void RvizMarkersPub::set_triangle_markers(const unsigned int marker_id, 
                                    const char* marker_namespace,
                                    const Geometry::Vertex3d vertex_vals,
                                    visualization_msgs::Marker & marker){ 
  // Publish the value of the measured CoM position
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any com sent with the same namespace and id will overwrite the old one
    //std::cout<<"name space: "<<marker_namespace<<std::endl;
    
    marker.ns = marker_namespace;
    marker.id = marker_id;

    // Set the com type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    // Set the com action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD; 

    // Set the pose of the com.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = vertex_vals(0);
    marker.pose.position.y = vertex_vals(1);
    marker.pose.position.z = vertex_vals(2);
    //std::cout<<predicted_particles_(j,0)<<" "<<predicted_particles_(j,0)<<std::endl;
    //marker.pose.orientation.x = 0.0;
    //marker.pose.orientation.y = 0.0;
    //marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the com -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.lifetime = ros::Duration();
  //
}

int RvizMarkersPub::publish_markers(int argc, char** argv){
    for(int t = 0; t < cycles_n_; t++){
      std::cout<<avg_particle_[t].transpose()<<std::endl;
    }
  // set the name of the ROS node
  ros::init(argc, argv, "motion_viewer");
  ros::NodeHandle n;
  ros::Rate r(1);
  // set the name of the marker
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("online_id_marker", 500);
  ros::Publisher array_pub = n.advertise<visualization_msgs::MarkerArray>("online_id_marker_array", 500);
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  char const * marker_nm_com = "CoM";
  char const * marker_nm_gt = "ground_truth";
  char const * marker_nm_p = "particles";
  std::cout<<"Start ROS"<<std::endl;
  while (ros::ok())
  {
    for(int t = 0; t < cycles_n_; t++){
      visualization_msgs::Marker com_marker;
      visualization_msgs::MarkerArray com_marker_array, gt_marker_array;
  
          // Set the color -- be sure to set alpha to something non-zero!
      com_marker.color.r = 1.0f;
      com_marker.color.g = 0.0f;
      com_marker.color.b = 0.0f;
      com_marker.color.a = 0.25;
      set_markers(t, marker_nm_com, com_measurements_[t], com_marker);
  
      // Publish the marker
      while (array_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(0.01);
      }

      com_marker_array.markers.push_back(com_marker);
      array_pub.publish(com_marker_array);
//  
//    Publish the avarage of the particle set
      // Set the color -- be sure to set alpha to something non-zero!
      avg_particle_marker.color.r = 0.0f;
      avg_particle_marker.color.g = 0.0f;
      avg_particle_marker.color.b = 1.0f;
      avg_particle_marker.color.a = 0.25;
      set_markers(1000+t, marker_nm_gt, avg_particle_[t], avg_particle_marker);
      gt_marker_array.markers.push_back(avg_particle_marker);
      array_pub.publish(gt_marker_array);
    
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        for(int j = 0; j < particles_n_; j++){
          visualization_msgs::Marker particles_marker;
          visualization_msgs::MarkerArray particles_marker_array;
          // Set the color -- be sure to set alpha to something non-zero!
          particles_marker.color.r = 0.0f;
          particles_marker.color.g = 1.0f;
          particles_marker.color.b = 0.0f;
          particles_marker.color.a = 0.25;
          Eigen::Vector3d current_particle;
          current_particle(0) = particles_array[t](j,0);
          current_particle(1) = particles_array[t](j,1);
          current_particle(2) = particles_array[t](j,2);
    
          //std::cout<<103 + j<<std::endl;
          set_markers(103 + j + t*cycles_n_, marker_nm_p, current_particle, weights_[t](j), particles_marker);
          particles_marker_array.markers.push_back(particles_marker);
          array_pub.publish(particles_marker_array);
        }
      }
      r.sleep();
  }
}

int RvizMarkersPub::publish_triangles(int argc, char** argv){

  // set the name of the ROS node
  ros::init(argc, argv, "triangles");
  ros::NodeHandle nh_triangles;
  ros::Rate r(1);
  // set the name of the marker
  ros::Publisher triangles_pub = nh_triangles.advertise<visualization_msgs::MarkerArray>("triangles_array", 500);
  // Set our initial shape type to be a cube
  //uint32_t shape = visualization_msgs::MarkerArray::TRIANGLE_LIST;
  std::cout<<"Start ROS"<<std::endl;
  while (ros::ok())
  {
    for(int t = 0; t < cycles_n_; t++){  
      // Publish the marker
      while (triangles_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(0.01);
      }

//  
//    Publish the avarage of the particle set
      char const * marker_nm_triangles = "triangles";
      visualization_msgs::Marker vertex_marker1, vertex_marker2, vertex_marker3;
      visualization_msgs::MarkerArray triangle_marker_array;
      // Set the color -- be sure to set alpha to something non-zero!
      vertex_marker1.color.r = 0.0f;
      vertex_marker1.color.g = 0.0f;
      vertex_marker1.color.b = 1.0f;
      vertex_marker1.color.a = 0.25;
      set_triangle_markers(2000+t*3, marker_nm_triangles, triangles_array[t].v1, vertex_marker1);
      triangle_marker_array.markers.push_back(vertex_marker1);
      vertex_marker2.color.r = 0.0f;
      vertex_marker2.color.g = 0.0f;
      vertex_marker2.color.b = 1.0f;
      vertex_marker2.color.a = 0.25;
      set_triangle_markers(2000+t*3+1, marker_nm_triangles, triangles_array[t].v2, vertex_marker2);
      triangle_marker_array.markers.push_back(vertex_marker2);
      vertex_marker3.color.r = 0.0f;
      vertex_marker3.color.g = 0.0f;
      vertex_marker3.color.b = 1.0f;
      vertex_marker3.color.a = 0.25;
      set_triangle_markers(2000+t*3+2, marker_nm_triangles, triangles_array[t].v3, vertex_marker3);
      triangle_marker_array.markers.push_back(vertex_marker3);
      triangles_pub.publish(triangle_marker_array);
      }
      r.sleep();
  }
}


void RvizMarkersPub::set_particles(const Eigen::Vector3d & avg_particle,
                                  const Eigen::Vector3d & com_measurements,
                                  const Eigen::MatrixXd & particles,
                                  const Eigen::VectorXd & weights,
                                  const unsigned int index){
  std::cout<<"Set particles"<<std::endl;
  particles_n_ = particles.rows();
  particles_array[index].resize(particles_n_, particles.cols());
  particles_array[index].setZero();
  particles_array[index] = particles;
  com_measurements_[index] = com_measurements;
  avg_particle_[index] = avg_particle;
  weights_.resize(particles_n_);
  weights_[index] = weights;
}

void RvizMarkersPub::set_triangles(const Geometry::Vertex3d vertex1,
                                   const Geometry::Vertex3d vertex2,
                                   const Geometry::Vertex3d vertex3,
                                   const unsigned int index){

  std::cout<<"Set triangles"<<std::endl;
  triangles_array[index].v1 = vertex1;
  triangles_array[index].v2 = vertex2;
  triangles_array[index].v3 = vertex3;
  std::cout<<triangles_array[index].v1.transpose()<<std::endl;
  std::cout<<triangles_array[index].v2.transpose()<<std::endl;
  std::cout<<triangles_array[index].v3.transpose()<<std::endl;

}






