// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <csignal>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

bool flag = false;
int choiceCont = choiceController(flag);
int choiceTraj = choiceTrajectory(flag);

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj);
            joint_torques_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            if (!robot_) {
                RCLCPP_ERROR(this->get_logger(), "robot_ is not initialized!");
            } else {
                RCLCPP_INFO(this->get_logger(), "robot_ initialized successfully.");
            }
            KDLController controller_(*robot_);  // Pass robot_ after it's initialized
            RCLCPP_INFO(this->get_logger(), "KDLController initialized.");


            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;


            // Definition of the effective initial position
            KDL::JntArray q(robot_->getNrJnts());
            // robot_->getInverseKinematics(init_cart_pose_, q);
            q.data(0) = 0.5;
            q.data(1) = -0.7854;
            q.data(2) = 0.0;
            q.data(3) = 1.3962;
            q.data(4) = 0.0;
            q.data(5) = 0.6109;
            q.data(6) = 0.0;
            robot_->getForwardKinematics(init_cart_pose_, q, -1);

            // Definition of the initial position + an offset
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];

            // Plan trajectory 
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, traj_radius = 0.08;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, traj_radius);
            planner_1 = KDLPlanner(traj_duration, acc_duration, init_position, end_position, 0.0);
            planner_2 = KDLPlanner(traj_duration, acc_duration, init_position, end_position, traj_radius);

            trajectory_point p;
            
            // Retrieve the first trajectory point
            switch (choiceTraj) {
                case 1:
                // Calcola la traiettoria rettilinea con interpolazione trapezoidale
                    p = planner_1.compute_rectilinear_trajectory_with_trapez(t);
                    break;

                case 2:
                    // Calcola la traiettoria circolare con interpolazione trapezoidale
                    p = planner_2.compute_circular_trajectory_with_trapez(t);
                    break;

                case 3:
                    // Calcola la traiettoria rettilinea con interpolazione cubica
                    p = planner_1.compute_rectilinear_trajectory_with_cubic(t);
                    break;

                case 4:
                    // Calcola la traiettoria circolare con interpolazione cubica
                    p = planner_2.compute_circular_trajectory_with_cubic(t);
                    break;
                    }            

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if (cmd_interface_ == "velocity") {
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(2), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }

            else {
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                // We increased the publishing rate of the effort commands to obtain a smoother behavior of the trajectory 
                switch(choiceCont) {
                    case 1:
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                        break;
                    case 2:
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                        break;
                
                }
                // Joint torques computed with a PD+ regulator, so that the robot attains the prescribed initial position
                Eigen::VectorXd gravity_torques = controller_.PDCntr(q,50,30);
                joint_torques_.data = gravity_torques;
                // Send joint torque commands
                for(long int i = 0; i < joint_torques_.data.size(); i++) {
                    desired_commands_[i] = joint_torques_(i);
                }
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; // 
            int trajectory_len;

            switch(choiceCont){
                case 1:
                    trajectory_len = 1500;
                    break;

                case 2:
                    trajectory_len = 150;
                    break;
                    }
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;

            static Eigen::Vector3d prev_ang_vel = Eigen::Vector3d::Zero(); // Initialize to zero
            static bool first_iteration = true; // To handle initialization on the first call

            if (t_ < total_time){
                KDLController _controller(*robot_);
                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point
                trajectory_point p;
                
                    switch (choiceTraj) {
                        case 1:
                            // Calcola la traiettoria rettilinea con interpolazione trapezoidale
                            p = planner_1.compute_rectilinear_trajectory_with_trapez(t_);
                            break;

                        case 2:
                            // Calcola la traiettoria circolare con interpolazione trapezoidale
                            p = planner_2.compute_circular_trajectory_with_trapez(t_);
                            break;

                        case 3:
                            // Calcola la traiettoria rettilinea con interpolazione cubica
                            p = planner_1.compute_rectilinear_trajectory_with_cubic(t_);
                            break;

                        case 4:
                            // Calcola la traiettoria circolare con interpolazione cubica
                            p = planner_2.compute_circular_trajectory_with_cubic(t_);
                            break;
                    }
                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();   
                KDL::Twist cartvel = robot_->getEEVelocity();                

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p.pos);
                KDL::Twist desTwist; desTwist.rot = cartvel.rot; desTwist.vel = toKDL(p.vel);


                Eigen::Vector3d des_angular_acc;
                if (!first_iteration) {
                //  Compute rotational acceleration via numerical differentiation
                des_angular_acc = (toEigen(desTwist.rot) - prev_ang_vel) / dt;
                } else {
                des_angular_acc = Eigen::Vector3d::Zero(); // First iteration: no previous velocity
                first_iteration = false;
                }

                prev_ang_vel = toEigen(desTwist.rot);

                KDL::Twist desAcc; 
                desAcc.rot = toKDL(des_angular_acc); 
                desAcc.vel = toKDL(p.acc);
                // How do we retrieve the current EE acceleration (rotational part)? 
                // I kept it at 0, probably not correct

                // compute errors (angle-axis representation)
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "velocity"){

                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                    
                }
                else {
                    // Definition of three vectors of desired joint positions, velocities and accelerations
                    KDL::JntArray qd(robot_->getNrJnts());
                    KDL::JntArray qd_dot(robot_->getNrJnts());
                    KDL::JntArray qd_ddot(robot_->getNrJnts());

                    // Compute operational space acceleration
                    Eigen::Matrix<double,6,1> acceleration = toEigen(desAcc); // Initialize with 6 for 3 translational and 3 rotational components

                    // Get the corresponding joint variables from open loop inverse kinematics
                    // Uncomment the following to try ID controller in the joint space
                    robot_->getInverseKinematics(desFrame,qd);
                    robot_->getInverseKinematicsVel(qd,desTwist,qd_dot);
                    // qd_ddot.data = (pseudoinverse(robot_->getEEJacobian().data))*(acceleration - robot_->getEEJacDot() * qd_dot.data);

                    // Get joint accelerations through derivation
                    // qd_ddot.data = qd_dot.data/dt;

                    // Gains for the ID controller in the operational space
                    double Kpp = 50;
                    double Kdp = sqrt(Kpp);
                    double Kpo = 50;
                    double Kdo = sqrt(Kpo);

                    // Gains for the PD+ regulator or ID controller in the joint space
                    double Kp = 50;
                    double Kd = sqrt(Kp);

                    Eigen::VectorXd tau;

                    switch(choiceCont){
                         case 1:
                            tau = _controller.PDCntr(qd,Kp,Kd); // PD+ in the joint space 
                            break;

                        case 2:
                            tau = _controller.idCntr(desFrame, desTwist, desAcc, Kpp, Kpo, Kdp, Kdo); // ID in the operational space
                            break;
                    }

                
                    joint_torques_.data = tau;


                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "velocity") {
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else{
                    // Send joint torque commands
                    for(long int i = 0; i < joint_torques_.data.size(); i++) {
                    desired_commands_[i] = joint_torques_(i);
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                RCLCPP_INFO_ONCE(this->get_logger(), "Press Ctrl + C to exit.");
                if(cmd_interface_ == "velocity") {
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                }
                else {
                    
                    // Send joint torque commands
                    // Keep sending effort commands through a PD+ regulator to keep the manipulator standing 
                    KDLController _controller(*robot_);
                    KDL::JntArray qd(robot_->getNrJnts());
                    KDL::Frame finalPose = robot_->getEEFrame();
                    
                    //qd.data = robot_->getJntValues();
                    robot_->getInverseKinematics(finalPose,qd);
                    Eigen::VectorXd tau = _controller.PDCntr(qd,50,sqrt(50));
                    joint_torques_.data = tau;

                        
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                    for(long int i = 0; i < joint_torques_.data.size(); i++) {
                        desired_commands_[i] = joint_torques_(i);
                    }

                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);                    
                }

            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
                joint_torques_.data[i] = sensor_msg.effort[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_torques_;
        std::shared_ptr<KDLRobot> robot_;
        KDLController controller_;
        KDLPlanner planner_;
        KDLPlanner planner_1;
        KDLPlanner planner_2;
        KDLController _controller;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        KDL::Frame init_cart_pose_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}