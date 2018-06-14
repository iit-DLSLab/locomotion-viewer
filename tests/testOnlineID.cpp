/*
 * test.cpp
 *
 *  Created on: Feb 11, 2018
 *      Author: Romeo Orsolino
 */
#include <locomotion-viewer/RvizMarkersPub.h>
#include <locomotion-viewer/ParticleFilter.h>


int main(int argc, char** argv)
{
    NormalDistribution nd;

    // For visualizing things in rviz
    ros::init(argc, argv, "triangles");
    ros::NodeHandle nh_triangles;
    ros::Rate r(1);

    //rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    //visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_frame","/rviz_visual_markers"));
    
    RvizPolygonsToolsPtr visual_poly_;
    ros::NodeHandle nh;
    visual_poly_.reset(new RvizPolygonsTools("base_frame","/rviz_visual_markers", nh));

    /*
	//std::cout<<"Generation of 1D points"<<std::endl;
	//Eigen::VectorXd norm_points;
	//norm_points = nd.generate_1D_points(10, 20.0, 1.0);
	//std::cout<<norm_points<<std::endl;

	//std::cout<<"Generation of normally distributed 2D points"<<std::endl;
	//Eigen::MatrixXd norm_2D_points;
	//Eigen::Vector2d mean, variance;
	//mean(0) = 10.0;
	//mean(1) = 20.0;
	//variance(0) = 1.0;
	//variance(1) = 2.0;
    //norm_2D_points = nd.generate_2D_points(10, mean, variance);
	//std::cout<<norm_2D_points<<std::endl;
    */

	//std::cout<<"Testing a fake particle filter process"<<std::endl;
    //ParticleFilter pf;
    //ParticleFilter::particle_set p;
    //const unsigned int cycles_num = 5;
    //Eigen::Matrix<ParticleFilter::particle_set,cycles_num+1,1> p_vec;
    //Eigen::Matrix<ParticleFilter::particle_set,cycles_num+1,1> before_resampling;
    //Eigen::Matrix<Eigen::Vector3d, cycles_num+1, 1> com_est, com_gt_; 
//
    //unsigned int p_num = 10;
    //for(int i=0; i<cycles_num; i++){
    //   pf.init_particles_set(p_num, p_vec[i]);
    //   pf.init_particles_set(p_num, before_resampling[i]);
    //   com_est[i].setZero();
    //   com_gt_[i].setZero();
    //}
    //std::cout<<"particles number:"<<std::endl;
    //std::cout<<p_vec[0].particle_num<<std::endl;
    //std::cout<<"values:"<<std::endl;
    //std::cout<<p_vec[0].value<<std::endl;
    //std::cout<<"initial weights:"<<std::endl;
    //std::cout<<p_vec[0].weight<<std::endl;
    //Eigen::MatrixXd p_before_resampling, p_after_resampling, com_gt;
    //Eigen::VectorXd w_before_resampling, w_after_resampling; 
    //com_gt.resize(p_num,3); com_gt.setZero();
    //p_before_resampling.resize(p_num,3); p_before_resampling.setZero();
    //p_after_resampling.resize(p_num,3); p_after_resampling.setZero();
    //w_before_resampling.resize(p_num); w_before_resampling.setZero();
    //w_after_resampling.resize(p_num); w_after_resampling.setZero();
    //Eigen::Vector3d mean_particle;
//
    //for(int i=0; i<cycles_num; i++){
    //    /* Propagate in time the particles according to the motion model*/
    //    pf.prediction_model(p_vec[i].particle_num, 0.0, 0.1, p_vec[i].value);
    //    pf.prediction_model(1 , 0.0, 0.0, com_gt_[i]);
//
    //    std::cout<<"New particle set time propagation..."<<std::endl;
    //    std::cout<<p_vec[i].value<<std::endl;
    //    std::cout<<"Ground truth time propagation..."<<std::endl;
    //    std::cout<<com_gt_[i]<<std::endl;
//
    //    /* Simulated GRFs */
    //    Eigen::Vector3d vel = Eigen::Vector3d(1.0,1.0,0.0);
    //    double delta_t = 1.0*(i+1);
    //    Eigen::Vector4d grfs_z = Eigen::Vector4d(1.0,1.0,1.0,1.0);
    //    Eigen::Vector3d foot_LF, foot_RF, foot_LH, foot_RH;
    //    foot_LF = Eigen::Vector3d(1.0,1.0,1.0) + vel*delta_t;
    //    foot_RF = Eigen::Vector3d(1.0,-1.0,1.0) + vel*delta_t;
    //    foot_LH = Eigen::Vector3d(-1.0,1.0,1.0) + vel*delta_t;
    //    foot_RH = Eigen::Vector3d(-1.0,-1.0,1.0) + vel*delta_t;
//
    //    /* Estimation of the CoM based on the GRFs: */
    //    double grfs_sum = grfs_z.sum();
    //    com_est[i](0) = foot_LF(0)*grfs_z(0) + foot_RF(0)*grfs_z(1) + foot_LH(0)*grfs_z(2) + foot_RH(0)*grfs_z(3);
    //    com_est[i](0) /= grfs_sum;
    //    com_est[i](1) = foot_LF(1)*grfs_z(0) + foot_RF(1)*grfs_z(1) + foot_LH(1)*grfs_z(2) + foot_RH(1)*grfs_z(3);
    //    com_est[i](1) /= grfs_sum;
    //    /* Add gaussian noise to the CoM estimation based on the measurements*/
    //    com_est[i] += nd.generate_1D_points(3, 0.0, 1.0);
    //    com_est[i](2) = 0.0;
    //    std::cout << "Estimated CoM:" << std::endl;
    //    std::cout<<com_est[i]<<std::endl;
//
    //    /* Update the weights according to the estimated CoM. */
    //    Eigen::Matrix3d sigma; 
    //    sigma.setZero();
    //    sigma(0,0) = 0.05;
    //    sigma(1,1) = 0.05;
    //    sigma(2,2) = 0.05;
    //    pf.update_weights(p_vec[i].value, com_est[i], sigma, p_vec[i].weight);
    //    std::cout<<"updated weights:"<<std::endl;
    //    std::cout<<p_vec[i].weight<<std::endl;
    //    p_before_resampling = p_vec[i].value;
    //    w_before_resampling = p_vec[i].weight;
    //    before_resampling[i].value = p_vec[i].value;
    //    before_resampling[i].weight = p_vec[i].weight;
//
    //    /* Perform resampling */
    //    std::cout<<"Low variance resampling:"<<std::endl;
    //    pf.low_variance_resampling(p_vec[i].weight, p_vec[i].value);
    //    std::cout<<"ground truth CoM"<<std::endl;
    //    std::cout<<com_gt_[i]<<std::endl;
    //    std::cout<<"New particle set AFTER resampling..."<<std::endl;
    //    std::cout<<p_vec[i].value<<std::endl;
    //    p_after_resampling = p_vec[i].value;
    //    std::cout<<"New weights AFTER resampling:"<<std::endl;
    //    std::cout<<p_vec[i].weight<<std::endl;
    //    w_after_resampling = p_vec[i].weight;
    //    mean_particle = Eigen::Vector3d(p_vec[i].value.block(0,0,p_vec[i].particle_num,1).sum(),
    //                                    p_vec[i].value.block(0,1,p_vec[i].particle_num,1).sum(),
    //                                    p_vec[i].value.block(0,2,p_vec[i].particle_num,1).sum());
    //    mean_particle /= p_vec[i].particle_num;
//
    //    double err_no_filter, err_with_filter;
    //    err_no_filter = sqrt(pow(com_est[i](0) - i-1 ,2.0) + pow(com_est[i](1) - i-1 ,2.0) + pow(com_est[i](2),2.0));
    //    err_with_filter = sqrt(pow(mean_particle(0) - i-1 ,2.0) + pow(mean_particle(1) - i-1 ,2.0) + pow(mean_particle(2),2.0));
    //    std::cout<<"END of loop number: "<<i<<std::endl;
    //    std::cout<<"(no filter)     CoM est:       "<<com_est[i].transpose()<<"  Error: "<<err_no_filter<<std::endl;
    //    std::cout<<"(with filter)   MEAN Particle: "<<mean_particle.transpose()<<"  Error: "<<err_with_filter<<std::endl;
    //    p_vec[i+1].value = p_vec[i].value;
    //    std::cout<<i<<std::endl;
    //    std::cout<<p_vec[i].value<<std::endl;
    //    std::cout<<"com_gt "<<com_gt_[i]<<std::endl;
    //    std::cout<<"com_est "<<com_est[i]<<std::endl;
    //    com_est[i+1] = com_est[i];
    //    com_gt_[i+1] = com_gt_[i];
    //}
//
    //std::cout<<"End of loop"<<std::endl;
    //RvizMarkersPub rviz_pub(cycles_num);        
    //for(int i=0; i<cycles_num; i++){
    //    rviz_pub.set_particles(com_gt_[i], com_est[i], before_resampling[i].value, before_resampling[i].weight, i);
    //    rviz_pub.set_triangles(Eigen::Vector3d(10.0*i, 0.0, 0.0),
    //                            Eigen::Vector3d(0.0, -10.0*i, 0.0),
    //                            Eigen::Vector3d(0.0, 10.0*i, 0.0), i);
    //}

    //rviz_pub.publish_markers(argc, argv);
    //rviz_pub.publish_triangles(argc, argv);
    // Create pose

    Eigen::Vector3d vertex1 = Eigen::Vector3d(1.0, 1.0, 0.0);
    Eigen::Vector3d vertex2 = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d vertex3 = Eigen::Vector3d(-1.0, 0.0, 0.0);
    Eigen::Vector3d vertex4 = Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Vector3d vertex5 = Eigen::Vector3d(1.0, 1.0, 1.0);
    Eigen::Vector3d vertex6 = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d vertex7 = Eigen::Vector3d(-1.0, 0.0,1.0);
    Eigen::Vector3d vertex8 = Eigen::Vector3d(0.0, 1.0, 1.0);

    while (ros::ok())
    {
    visual_poly_->deleteAllMarkers();
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 1.0, 1.0, 1.0 ); // translate x,y,z
    
    // Publish arrow vector of pose
    ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
    //visual_poly_->publishXYPlane(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

    //visual_poly_->publishArrow(pose, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    
    //visual_poly_->publishHexahedron(pose, vertex1, vertex2, vertex3, vertex4, vertex5, vertex6, vertex7, vertex8, true);

    visual_poly_->publishQuadrilateral(pose, vertex1, vertex2, vertex3, vertex6, true, rviz_visual_tools::GREEN);
    
    //vertex1(0)+=0.02;
    //visual_poly_->publishTriangle(pose, vertex1, vertex2, vertex3);

    //visual_poly_->publishQuadrilateral(pose, vertex1, vertex2, vertex3, vertex4);

    // Don't forget to trigger the publisher!
    visual_poly_->trigger();
    r.sleep();
    }
}
