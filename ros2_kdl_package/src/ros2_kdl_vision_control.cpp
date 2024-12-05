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
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include <cv_bridge/cv_bridge.h>  // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp>  // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // OpenCV header for image processing

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

bool aruco_detected_ = false;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_vision_control"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter
            declare_parameter("cmd_interface", "velocity"); // defaults to "velocity"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            // declare task_mode_ parameter
            declare_parameter("task_mode_", "positioning"); // "positioning" or "look_at_point"
            get_parameter("task_mode_", task_mode_);
            RCLCPP_INFO(get_logger(), "Selected task mode: '%s'", task_mode_.c_str());

            // declare offset parameters
            declare_parameter("offset_x", 0.2); // default 0.2
            declare_parameter("offset_y", -0.2); // default -0.2
            declare_parameter("offset_z", 0.0); // default 0.0
            double offset_x = get_parameter("offset_x").as_double();
            double offset_y = get_parameter("offset_y").as_double();
            double offset_z = get_parameter("offset_z").as_double();
            RCLCPP_INFO(get_logger(), "Offset: (%f, %f, %f)", offset_x, offset_y, offset_z);


            if (!(cmd_interface_ == "velocity"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            if (task_mode_ != "positioning" && task_mode_ != "look_at_point") {
                RCLCPP_ERROR(get_logger(), "Invalid task parameter! Choose between 'positioning' or 'look-at-point'.");
                rclcpp::shutdown();
            }

            if (offset_x < 0.05 || offset_x > 0.35 || offset_y < -0.25 || offset_y > 0.0 || offset_z < -0.15 || offset_z > 0.0) {
                RCLCPP_ERROR(get_logger(), "Invalid offset parameters! x = [0.05, 0.35], y =[-0.25, 0.0], z = [-0.15, 0.0].");
                rclcpp::shutdown();
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

            // Subscriber to aruco pose
            arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_marker_subscriber, this, std::placeholders::_1));
            

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // while(!aruco_detected_) {
            //     RCLCPP_INFO(this->get_logger(), "Aruco not detected!...");
            //     rclcpp::spin_some(node_handle_);
            // }

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
            init_jnt_ = robot_->getJntValues();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;
            
            Eigen::Vector3d init_position = toEigen(init_cart_pose_.p); 
            
            // Eigen::Vector3d init_position ;
            // KDL::Rotation rotation_increment = KDL::Rotation::RotX(3.14/2);
            // init_cart_pose_.p = init_cart_pose_.p + KDL::Vector(init_position(0),init_position(1),init_position(2));
            

            KDL::JntArray q(robot_->getNrJnts());



            // KDL::JntArray q(robot_->getNrJnts());
            // robot_->getInverseKinematics(init_cart_pose_, q);

            // Define desired position and orientation offsets
            Eigen::Vector3d desired_position_offset(offset_x, offset_y, offset_z);
            Eigen::Matrix3d desired_orientation_offset = Eigen::Matrix3d::Identity(); // No orientation tilt

            // Eigen::Vector3d desired_position_world = toEigen(init_cart_pose_.p) + toEigen(init_cart_pose_.M) * desired_position_offset;
            // KDL::Rotation desired_orientation_world = init_cart_pose_.M * desired_orientation_offset;
            Eigen::Vector3d marker_position_ee = toEigen(aruco_pose_.p);
            Eigen::Matrix3d marker_orientation_ee = toEigen(aruco_pose_.M);


            Eigen::Vector3d marker_position_world = toEigen(init_cart_pose_.p) + toEigen(init_cart_pose_.M)*marker_position_ee;
            Eigen::Matrix3d marker_orientation_world = toEigen(init_cart_pose_.M)*marker_orientation_ee;

            // Compute the offset in the world frame
            Eigen::Vector3d offset_world = marker_orientation_world * desired_position_offset;
            // Compute the target position in the world frame
            Eigen::Vector3d desired_position_world = marker_position_world + offset_world;
            // KDL::Rotation desired_orientation_world = aruco_pose_.M * desired_orientation_offset;

            KDL::Frame desiredPose; desiredPose.p = toKDL(desired_position_world); desiredPose.M = init_cart_pose_.M;
            Eigen::Vector3d end_position = toEigen(desiredPose.p);

            // init_cart_pose_.M = init_cart_pose_.M * rotation_increment;
            // init_cart_pose_.p = init_cart_pose_.p + KDL::Vector(0.5,-0.6,0.2);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "Joint values: " << std::endl;
            // std::cout << q.data(0) << std::endl;
            // std::cout << q.data(1) << std::endl;
            // std::cout << q.data(2) << std::endl;
            // std::cout << q.data(3) << std::endl;
            // std::cout << q.data(4) << std::endl;
            // std::cout << q.data(5) << std::endl;
            // std::cout << q.data(6) << std::endl;

            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, traj_radius = 0.08;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, 0.0);

            trajectory_point p;

            p = planner_.compute_trajectory(t);
           
            if (cmd_interface_ == "velocity") {
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            if (!aruco_detected_) {
                std::fill(desired_commands_.begin(), desired_commands_.end(), 0.0);
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                RCLCPP_WARN(this->get_logger(), "ArUco Tag not detected. Robot stopped. ");
                return;
            }

            if (task_mode_ == "positioning") {
                RCLCPP_INFO_ONCE(this->get_logger(), "Executing positioning task...");
                positioning();
            } else if (task_mode_ == "look_at_point") {
                RCLCPP_INFO_ONCE(this->get_logger(), "Executing look_at_point task...");
                look_at_point();
            } else {
                RCLCPP_WARN(this->get_logger(), "Task mode not valid!");
            }
        }

        void positioning() {

            iteration_ = iteration_ + 1;
            double total_time = 1.5;
            int trajectory_len = 150;
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;

                if(t_< total_time) {

                    trajectory_point p = planner_.compute_trajectory(t_); 

                    // Compute EE frame
                    KDL::Frame cartpos = robot_->getEEFrame();   
                    KDL::Twist cartvel = robot_->getEEVelocity();
                    // Compute errors
                    Eigen::Vector3d linError = computeLinearError(p.pos, toEigen(cartpos.p));
                    Eigen::Vector3d orError = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));


                    
                    if(cmd_interface_ == "velocity"){
                            
                            // Compute differential IK
                            Vector6d cartvel; cartvel << p.vel + 5*linError, orError;
                            joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                            joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                            
                        }

                        // Update KDLrobot structure
                        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                        if(cmd_interface_ == "velocity") {
                            // Send joint velocity commands
                            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                                desired_commands_[i] = joint_velocities_(i);
                            }
                        }

                        // Create msg and publish
                        std_msgs::msg::Float64MultiArray cmd_msg;
                        cmd_msg.data = desired_commands_;
                        cmdPublisher_->publish(cmd_msg);
                    }

                    else {
                        RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                        if(cmd_interface_ == "velocity") {
                        // Send joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = 0.0;
                            }
                        }



                            // Create msg and publish
                        std_msgs::msg::Float64MultiArray cmd_msg;
                        cmd_msg.data = desired_commands_;
                        cmdPublisher_->publish(cmd_msg);

                        }

                        // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                        // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                        // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                        // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                        // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                        // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                        // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
        }

        void look_at_point() {
                Eigen::Matrix<double,3,1> cP_o = toEigen(aruco_pose_.p);
                double norm_cP_o = cP_o.norm();
                Eigen::Matrix<double,3,1> s = cP_o / norm_cP_o;

                // desired direction
                Eigen::Vector3d s_d(0,0,1);

                // Compute L
                Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
                Eigen::MatrixXd L = Eigen::Matrix<double,3,6>::Zero();


                L.block(0,0,3,3) = -(1.0/norm_cP_o)*(I - s*s.transpose());
                L.block(0,3,3,3) = skew(s);

                // Compute R
                Eigen::MatrixXd R = Eigen::Matrix<double,6,6>::Zero();

                Eigen::MatrixXd Rc = toEigen(robot_->getEEFrame().M);
                R.block(0,0,3,3) = Rc;
                R.block(3,3,3,3) = Rc;
                L = L*R.transpose();

                // Compute control law
                Eigen::MatrixXd I_ = Eigen::Matrix<double,7,7>::Identity();
                Eigen::MatrixXd Jc = robot_->getEEJacobian().data;
                // Eigen::MatrixXd LJ = L*Jc;
                // Eigen::MatrixXd LJpinv = pseudoinverse(LJ);
                Eigen::MatrixXd N = Eigen::MatrixXd::Identity(Jc.cols(), Jc.cols()) - pseudoinverse(L * Jc) * (L * Jc);
                double k = 10;


                Eigen::Vector3d error = s_d - s;
            
                joint_velocities_.data = k * pseudoinverse(L*Jc) * error + N*(init_jnt_ - joint_positions_.data);
                joint_positions_.data = joint_positions_.data + joint_velocities_.data*0.02;   
                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                
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

       void aruco_marker_subscriber(const geometry_msgs::msg::PoseStamped& geometry_msg) {
            aruco_detected_ = true;

            // Estrai la posizione e l'orientamento dal messaggio
            double position_x = geometry_msg.pose.position.x;
            double position_y = geometry_msg.pose.position.y;
            double position_z = geometry_msg.pose.position.z;

            double orientation_x = geometry_msg.pose.orientation.x;
            double orientation_y = geometry_msg.pose.orientation.y;
            double orientation_z = geometry_msg.pose.orientation.z;
            double orientation_w = geometry_msg.pose.orientation.w;

            // Crea un KDL::Vector per la posizione
            KDL::Vector position(position_x, position_y, position_z);

            // Crea un KDL::Rotation dall'orientamento (quaternion)
            KDL::Rotation rotation = KDL::Rotation::Quaternion(orientation_x, orientation_y, orientation_z, orientation_w);

            // Assegna la nuova posizione e orientamento a aruco_pose_
            aruco_pose_ = KDL::Frame(rotation, position);
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
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
        Eigen::VectorXd init_jnt_;
        KDL::Frame aruco_pose_;
        std::string task_mode_;

};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}