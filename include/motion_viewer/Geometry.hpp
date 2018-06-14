/*
 * OnlineID.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <chrono>

class Geometry {

public:
  
  Geometry();

	virtual ~Geometry();
        /**
        * @brief Generate a vector of normally distributed scalar values
        * @param num_of_points	Number of random points to be generated. This corresponds to the length of the eigen vector
        * @param mean	Mean value of the normal distribution
        * @param variance	Standard deviation of the normal distribution
        */
  typedef Eigen::Vector2d Vertex2d;

  typedef Eigen::Vector3d Vertex3d;

  struct Triangle2d { 
    Vertex2d v1;
    Vertex2d v2;
    Vertex2d v3;
  };

  typedef struct Triangle2d Triangle2d;

  struct Triangle3d { 
    Vertex3d v1;
    Vertex3d v2;
    Vertex3d v3;
  };

  typedef struct Triangle3d Triangle3d;

  struct Quadrilateral2d { 
    Vertex2d v1;
    Vertex2d v2;
    Vertex2d v3;
    Vertex2d v4;
  };

  typedef struct Quadrilateral2d Quadrilateral2d;

  struct Quadrilateral3d { 
    Vertex3d v1;
    Vertex3d v2;
    Vertex3d v3;
    Vertex3d v4;
  };

  typedef struct Quadrilateral3d Quadrilateral3d;

  struct Hexahedron {
    Quadrilateral3d plane1;
    Quadrilateral3d plane2;
    Quadrilateral3d plane3;
  };

  typedef struct Hexahedron Hexahedron;

private:


};


#endif /* DLS_ONLINEID_H_ */