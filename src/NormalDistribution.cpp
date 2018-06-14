/*
 * NormalDistribution.cpp
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#include "../include/motion_viewer/NormalDistribution.h"


NormalDistribution::NormalDistribution()
{

}


NormalDistribution::~NormalDistribution(){

}


Eigen::VectorXd NormalDistribution::generate_1D_points(const int & num_of_points, 
														const double & mean, 
														const double & variance){
	Eigen::VectorXd points;
	points.resize(num_of_points);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
	std::normal_distribution<double> distribution(mean,variance);

	for(int i=0; i<num_of_points; i++){
		points(i) = distribution(generator);
	}

	return points;
}

//Eigen::MatrixXd NormalDistribution::generate_2D_points(const int & num_of_points,
//														const Eigen::Vector2d & mean,
//														const Eigen::Matrix2d & variance){
//	Eigen::MatrixXd points;
//	// This is wrong, the covariance should be a sqaure matrix
//	//points.resize(num_of_points, 2);
//	//points.setZero();
//	//points.block(num_of_points,1,0,0) = generate_1D_points(num_of_points, mean(0), variance(0));
//	//points.block(num_of_points,1,0,1) = generate_1D_points(num_of_points, mean(1), variance(1));
//
//	return points;
//}

