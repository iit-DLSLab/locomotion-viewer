/*
 * ParticleFilter.cpp
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */
#include "../include/motion_viewer/ParticleFilter.h"
#include <sys/time.h>


ParticleFilter::ParticleFilter()
{

}


ParticleFilter::~ParticleFilter(){

}

void ParticleFilter::init_particles_set(const unsigned int & particles_number,
                                        particle_set & ps){

    ps.particle_num = particles_number;
    ps.value.resize(ps.particle_num, 3);
    ps.weight.resize(ps.particle_num);
    ps.value.setZero();

    double mean_x, sigma_x, mean_y, sigma_y;
    mean_x = 0.0;
    mean_y = 0.0;
    sigma_x = 0.05;
    sigma_y = 0.05;

    ps.value.block(0, 0, ps.particle_num, 1) = nd.generate_1D_points(ps.particle_num, mean_x, sigma_x);
    ps.value.block(0, 1, ps.particle_num, 1) = nd.generate_1D_points(ps.particle_num, mean_y, sigma_y);

    init_weights(ps.value, Eigen::Vector3d(mean_x, mean_y, 0.0), sigma_x, ps.weight);
    //Eigen::VectorXd uniform_val;
    //uniform_val.resize(particles_number);
    //uniform_distribution(uniform_val);
    //ps.value.block(ps.particle_num, 1, 0, 0) = uniform_val;
    //uniform_distribution(uniform_val);
    //ps.value.block(ps.particle_num, 1, 0, 1) = uniform_val;
    ps.weight.setConstant(1.0/(double)particles_number);
}

void ParticleFilter::measurement_model(const Eigen::Vector3d & com_dd, Eigen::Vector3d & com_estimate){
    // LIP Model: com_xdd = grav / com_z * com_x
    double grav = 9.81;
    com_estimate(0) = com_dd(0) * com_estimate(2) / grav;   //  com_xdd = grav / com_z * com_x
    com_estimate(1) = com_dd(1) * com_estimate(2) / grav;     //  com_ydd = grav / com_z * com_y

}

void ParticleFilter::uniform_distribution(Eigen::VectorXd & values){
    double p_num = values.rows();
    /* initialize random seed: */
    struct timeval time;
    gettimeofday(&time,NULL);
    srand(time.tv_usec);
    for(int j = 0; j < p_num; j++){
        values(j) = rand() % 2000 -  1000;
        values(j) /= 1000;
    }
}
void ParticleFilter::prediction_model(const unsigned int particle_num, 
                                        const double mean, 
                                        const double variance, 
                                        Eigen::Vector3d & value){
        Eigen::Vector3d vel = Eigen::Vector3d(1.0,1.0,0.0);
        double delta_t = 1.0;
        Eigen::VectorXd white_noise_x, white_noise_y;
        white_noise_x = nd.generate_1D_points(particle_num, mean, variance);
        white_noise_y = nd.generate_1D_points(particle_num, mean, variance);
        //std::cout<<"Gaussian noise:"<<std::endl;
        //std::cout<<white_noise<<std::endl;
        
        value(0) += vel(0)*delta_t + white_noise_x(0);
        value(1) += vel(1)*delta_t + white_noise_y(0);

}

void ParticleFilter::prediction_model(const unsigned int particle_num, 
                                        const double mean, 
                                        const double variance, 
                                        Eigen::MatrixXd & value){
        Eigen::Vector3d vel = Eigen::Vector3d(1.0,1.0,0.0);
        double delta_t = 1.0;
        Eigen::VectorXd white_noise_x, white_noise_y, vel_x, vel_y;
        white_noise_x = nd.generate_1D_points(particle_num, mean, variance);
        white_noise_y = nd.generate_1D_points(particle_num, mean, variance);
        //std::cout<<"Gaussian noise:"<<std::endl;
        //std::cout<<white_noise<<std::endl;
        vel_x.resize(particle_num);
        vel_x.setConstant(vel(0));
        vel_y.resize(particle_num);
        vel_y.setConstant(vel(1));
            for(int j = 0; j < particle_num; j++){
                value(j,0) += vel(0)*delta_t + white_noise_x(j);
                value(j,1) += vel(1)*delta_t + white_noise_y(j);
            }
        //(ps.value).block(ps.particle_num,1,0,0) += vel_x*delta_t; //white_noise_x;

        //white_noise_y = nd.generate_1D_points(ps.particle_num, 0.0, 0.05);
        //ps.value.block(ps.particle_num,1,0,1) += vel_y*delta_t; //white_noise_y;
}

void ParticleFilter::init_weights(const Eigen::MatrixXd & p,
                        const Eigen::Vector3d & mean,
    					const double & sigma, 
                        Eigen::VectorXd & weights){
    //weight = exp(-(z - p)^T*(z - p)/(2*sigma^2))
    double p_num = weights.rows();
    /* initialize random seed: */
    srand((unsigned)time(0));
    for(int j = 0; j < p_num; j++){
        weights(j) = rand() % 1000 -  500;
        weights(j) /= 1000;
    }

}

void ParticleFilter::normalize_weights(Eigen::VectorXd & weights){
    double p_num = weights.rows();
    double weights_sum = weights.sum();
    for(int j = 0; j < p_num; j++){
        weights(j) = weights(j)/weights_sum;
    }

}

void ParticleFilter::update_weights(const Eigen::MatrixXd & p,
                        const Eigen::Vector3d & mean,
                        const Eigen::Matrix3d & sigma,
                        Eigen::VectorXd & weights){
    /* Update weights using the Normal distribution equation:
     * weight = (2*pi*signam^2)^(-1/2)*exp(-(p - mean)^T*(p - mean)/(2*sigma^2))    */
    double p_num = weights.rows();
    for(int j = 0; j < p_num; j++){
        //weights(j) = pow(2.0*M_PI*sigma*sigma,-0.5)*exp(-(p(j,0) - mean(0))*(p(j,0) - mean(0))/(2.0*sigma*sigma));
        Eigen::Vector3d particle = Eigen::Vector3d(p(j,0),p(j,1),p(j,2));
        Eigen::Vector3d deltap = particle - mean;
        double num = -0.5*deltap.transpose()*sigma.inverse()*deltap;
        num = exp(num);
        double denum = (2.0*M_PI*sigma).determinant();
        weights(j) = num/pow(denum, 0.5);
    }
    /*  Update weights using a value which is inverse to the distance of the particle from the measurement
     * */
    //for(int j = 0; j < p_num; j++){
    //    //weights(j) = pow(2.0*M_PI*sigma*sigma,-0.5)*exp(-(p(j,0) - mean(0))*(p(j,0) - mean(0))/(2.0*sigma*sigma));
    //    double deltap = pow(p(j,0) - mean(0),2.0) + pow(p(j,1) - mean(1),2.0) + pow(p(j,2) - mean(2),2.0);
    //    deltap = sqrt(deltap);
    //    weights(j) = 1.0/deltap;
    //}
    normalize_weights(weights);
}

void ParticleFilter::low_variance_resampling(Eigen::VectorXd & weights,
                                             Eigen::MatrixXd & values){
/* the code of this function implements the Low Variance Resampling algorithm explained at
    page 87 of the Probabilistic Robotics boot of Sebastian Thrun. */
double M = values.rows();
std::cout<<"Particle set: "<< values <<std::endl;
std::cout<<"weights set: "<< weights <<std::endl;
Eigen::VectorXd buffer, new_weights;
Eigen::MatrixXd new_particle;
buffer.resize(M);
std::cout<<"M: "<< M <<std::endl;
new_weights.resize(M);
new_particle.resize(M,3);
buffer.setZero();
new_weights.setZero();
new_particle.setZero();

buffer(0) = weights(0);
for(int m = 1; m < M; m++){
    buffer(m) = buffer(m-1) + weights(m);
}
//std::cout<<"weights buffer: "<<buffer.transpose()<<std::endl;

struct timeval time;
unsigned int counter = 0;
gettimeofday(&time,NULL);
srand(time.tv_usec);
for(int counter = 0; counter < M; counter++){
    double sample = rand() % 1000;
    sample /= 1000.0;
    //std::cout<<"current sample: "<<sample<<std::endl;
    Eigen::Vector3d first_p = Eigen::Vector3d(values(0,0),values(0,1),values(0,2));
    //std::cout<<"first particle: "<<first_p<<std::endl;
    if(sample<=buffer(0)){
        new_particle.block(counter,0,1,3) = first_p.transpose();
        new_weights(counter) = weights(0);
    }
    for(int k = 1; k < M; k++){

        if((sample<=buffer(k))&&(sample>buffer(k-1))){
        Eigen::Vector3d current_p = Eigen::Vector3d(values(k,0),values(k,1),values(k,2));
        new_particle.block(counter,0,1,3) = current_p.transpose();
        new_weights(counter) = weights(k);
        }
    }

}
values = new_particle;
normalize_weights(new_weights);
weights = new_weights;
}














