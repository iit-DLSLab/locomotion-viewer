/*
 * OnlineID.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#ifndef DLS_ONLINEID_H_
#define DLS_ONLINEID_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <random>
#include <chrono>

class NormalDistribution {

public:
    NormalDistribution();
	virtual ~NormalDistribution();
        /**
        * @brief Generate a vector of normally distributed scalar values
        * @param num_of_points	Number of random points to be generated. This corresponds to the length of the eigen vector
        * @param mean	Mean value of the normal distribution
        * @param variance	Standard deviation of the normal distribution
        */
        Eigen::VectorXd generate_1D_points(const int & num_of_points,
                               const double & mean,
                               const double & variance);
	/**
	 * @brief Generate a vector of normally distributed 2D points
	 * @param num_of_points	Number of random points to be generated. This corresponds to the length of the eigen vector
	 * @param mean	Mean value of the normal distribution
	 * @param variance	Standard deviation of the normal distribution
	 */
        //Eigen::MatrixXd generate_2D_points(const int & num_of_points,
        //                                   const Eigen::Vector2d & mean,
        //                                   const Eigen::Matrix2d & variance);


private:

  	std::default_random_engine  generator;


};


#endif /* DLS_ONLINEID_H_ */
