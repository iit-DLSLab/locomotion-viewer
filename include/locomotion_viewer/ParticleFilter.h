/*
 * ParticleFilter.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */

#ifndef DLS_ParticleFilter_H_
#define DLS_ParticleFilter_H_

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <random>
#include "locomotion_viewer/NormalDistribution.h"

class ParticleFilter {


public:


    struct particle_set {
        double particle_num;
        Eigen::MatrixXd value;
        Eigen::VectorXd weight;
    };

    ParticleFilter();
	virtual ~ParticleFilter();

	/**
	 * @brief Update the values of the weights associated to the particles
	 * the rule should look like:
         *                          weight = (2*pi*signam^2)^(-1/2)*exp(-(p - mean)^T*(p - mean)/(2*sigma^2))
	 * @param z 		Measurement value
	 * @param p 		Value associated to the particle
	 * @param sigma		Variance associated to the normal distribution
	 * @param weight 	Probability associated to the corresponding particle (the returned value should be normalized)
	 */
    void init_weights(const Eigen::MatrixXd & p,
                      const Eigen::Vector3d & mean,
                      const double & sigma,
                      Eigen::VectorXd & weights);

    void normalize_weights(Eigen::VectorXd & weights);

    void update_weights(const Eigen::MatrixXd & p,
                            const Eigen::Vector3d & mean,
                            const Eigen::Matrix3d & sigma,
                            Eigen::VectorXd & weights);

    void init_particles_set(const unsigned int & particles_number,
                                                particle_set & particles_set);

    void low_variance_resampling(Eigen::VectorXd & weights, Eigen::MatrixXd & p);

    void uniform_distribution(Eigen::VectorXd & values);

    void measurement_model(const Eigen::Vector3d & com_dd,
                           Eigen::Vector3d & com_estimate);

    void prediction_model(const unsigned int particle_num, 
                            const double mean, 
                            const double variance, 
                            Eigen::Vector3d & value);

    void prediction_model(const unsigned int particle_num, 
                            const double mean, 
                            const double variance, 
                            Eigen::MatrixXd & value);

    void prediction_step();

    void update_step();

private:

    NormalDistribution nd;


};


#endif /* DLS_ParticleFilter_H_ */
