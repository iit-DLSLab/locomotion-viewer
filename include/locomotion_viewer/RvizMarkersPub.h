/*
 * OnlineID.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#ifndef DLS_RVIZMARKERSPUB_H_
#define DLS_RVIZMARKERSPUB_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <locomotion_viewer/Geometry.hpp>
#include <locomotion_viewer/RvizPolygonsTools.hpp>


class RvizMarkersPub {

public:

  RvizMarkersPub(const unsigned int cycles);
	
  virtual ~RvizMarkersPub();

    struct triangle{
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
  };

  typedef struct triangle Triangle;

  void set_particles(const Eigen::Vector3d & avg_particle,
                                  const Eigen::Vector3d & com_measurements,
                                  const Eigen::MatrixXd & particles,
                                  const Eigen::VectorXd & weights,
                                  const unsigned int index);

  void set_triangles(const Geometry::Vertex3d v1,
                     const Geometry::Vertex3d v2,
                     const Geometry::Vertex3d v3,
                     const unsigned int index);

  void set_markers(const unsigned int marker_id, 
                    const char* marker_namespace,
                    const Eigen::Vector3d & values,
                    double weight,
                        visualization_msgs::Marker & marker);

  void set_markers(const unsigned int marker_id, 
                    const char* marker_namespace,
                    const Eigen::Vector3d & values,
                        visualization_msgs::Marker & marker);

  void set_triangle_markers(const unsigned int marker_id, 
                                    const char* marker_namespace,
                                    const Geometry::Vertex3d vertex_vals,
                                    visualization_msgs::Marker & marker);

  //void set_triangle_markers(const unsigned int marker_id, 
  //                                  const char* marker_namespace,
  //                                  const Geometry::Triangle3d triangle,
  //                                  visualization_msgs::Marker & marker);

  int publish_markers(int argc, char** argv);

  int publish_triangles(int argc, char** argv);

private:

  	std::default_random_engine  generator;
    visualization_msgs::Marker avg_particle_marker, com_marker;
    Eigen::MatrixXd particles_;
    Eigen::Matrix<Eigen::MatrixXd, 10, 1> particles_array;
    Eigen::Matrix<Geometry::Triangle3d, 10, 1> triangles_array;
    Eigen::Matrix<Eigen::VectorXd, 10, 1> weights_;
    Eigen::Matrix<Eigen::Vector3d, 10, 1> com_measurements_, avg_particle_;
    double particles_n_, iterations_num_, triangles_n_;
    unsigned int cycles_n_ = 0;
    geometry_msgs::Point p;

};


#endif /* DLS_RVIZMARKERSPUB_H_ */
