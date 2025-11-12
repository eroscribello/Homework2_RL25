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

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    Iiwa_pub_sub()
        : Node("ros2_kdl_node"),
          node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this,
            "ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, _1));

        declare_parameter("traj_duration", rclcpp::PARAMETER_DOUBLE);
        declare_parameter("total_time", rclcpp::PARAMETER_DOUBLE);
        declare_parameter("trajectory_len", rclcpp::PARAMETER_INTEGER);
        declare_parameter("Kp", rclcpp::PARAMETER_INTEGER);
        declare_parameter("end_position", rclcpp::PARAMETER_DOUBLE_ARRAY);
        declare_parameter("acc_duration", rclcpp::PARAMETER_DOUBLE);

        get_parameter("traj_duration", traj_duration_);
        get_parameter("total_time", total_time_);
        get_parameter("trajectory_len", trajectory_len_);
        get_parameter("Kp", Kp_);
        get_parameter("end_position", end_position_);
        get_parameter("acc_duration", acc_duration_);

        // declare cmd_interface parameter (position, velocity)
        declare_parameter("cmd_interface", "velocity"); // default to "position"
        get_parameter("cmd_interface", cmd_interface_);

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
        {
            RCLCPP_ERROR(get_logger(), "Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); 
            return;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Current cmd interface is: '%s'", cmd_interface_.c_str());
        }

        // declare ctrl parameter (velocity_ctrl, velocity_ctrl_null, vision)
        declare_parameter("ctrl", "velocity_ctrl"); // default to "velocity_ctrl"
        get_parameter("ctrl", ctrl_);

        if (!(ctrl_ == "velocity_ctrl" || ctrl_ == "velocity_ctrl_null" || ctrl_ == "vision"))
        {
            RCLCPP_ERROR(get_logger(), "Selected ctrl is not valid! Use 'velocity_ctrl' or 'velocity_ctrl_null instead..."); 
            return;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Current ctrl is: '%s'", ctrl_.c_str());
        }

        // declare traj_type parameter (linear, circular)
        declare_parameter("traj_type", "linear");
        get_parameter("traj_type", traj_type_);
        RCLCPP_INFO(get_logger(), "Current trajectory type is: '%s'", traj_type_.c_str());
        if (!(traj_type_ == "linear" || traj_type_ == "circular"))
        {
            RCLCPP_INFO(get_logger(), "Selected traj type is not valid!"); 
            return;
        }

        // declare s_type parameter (trapezoidal, cubic)
        declare_parameter("s_type", "trapezoidal");
        get_parameter("s_type", s_type_);
        RCLCPP_INFO(get_logger(), "Current s type is: '%s'", s_type_.c_str());
        if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
        {
            RCLCPP_INFO(get_logger(), "Selected s type is not valid!"); 
            return;
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
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        controller_ = KDLController(*robot_);

        // Create joint array
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96;
        robot_->setJntLimits(q_min, q_max);
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        MarkerPoseSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", qos_profile, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));

        // Subscriber to jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Update KDLrobot object
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();

        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        // Initialize controller (local variable removed; keep controller_ already set)
        // KDLController controller_(*robot_);

        // EE's trajectory initial position (just an offset)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0, 0, 0.1));

        // EE's trajectory end position (from parameter)
        Eigen::Vector3d end_position(end_position_[0], end_position_[1], end_position_[2]);

        // Plan trajectory
        double traj_duration = traj_duration_, acc_duration = acc_duration_, traj_radius = 0.15;

        // Retrieve the first trajectory point
        if (traj_type_ == "linear") {
            RCLCPP_INFO(get_logger(), " linear end [%f,%f,%f]", end_position(0), end_position(1), end_position(2));
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // trapezoidal by default
            if (s_type_ == "trapezoidal") {
                p_ = planner_.linear_traj_trapezoidal(t_);
            } else if (s_type_ == "cubic") {
                p_ = planner_.linear_traj_cubic(t_);
            }
        } else if (traj_type_ == "circular") {
            planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
            if (s_type_ == "trapezoidal") {
                p_ = planner_.circular_traj_trapezoidal(t_);
            } else if (s_type_ == "cubic") {
                p_ = planner_.circular_traj_cubic(t_);
            }
        }

        if (cmd_interface_ == "position") {
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        } else if (cmd_interface_ == "velocity") {
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        } else if (cmd_interface_ == "effort") {
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
        }

        // Create msg and publish initial command
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        RCLCPP_INFO(this->get_logger(), "Waiting for client request...");
    }

private:
    KDLController controller_;

    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;

    // Parameters
    double traj_duration_;
    double total_time_;
    int trajectory_len_;
    int Kp_;
    std::vector<double> end_position_;
    double acc_duration_;

    void cmd_publisher()
    {
        iteration_ = iteration_ + 1;

        // define trajectory
        double total_time = total_time_;
        int trajectory_len = trajectory_len_;
        int loop_rate = trajectory_len / total_time;
        double dt = 1.0 / loop_rate;
        int Kp = Kp_;
        t_ += dt;

        if (t_ < total_time) {
            // Retrieve the trajectory point based on the trajectory type
            if (traj_type_ == "linear") {
                if (s_type_ == "trapezoidal") {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                } else if (s_type_ == "cubic") {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } else if (traj_type_ == "circular") {
                if (s_type_ == "trapezoidal") {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                } else if (s_type_ == "cubic") {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }

            // Compute EE frame
            KDL::Frame cartpos = robot_->getEEFrame();

            // Compute desired Frame
            KDL::Frame desFrame;
            desFrame.M = cartpos.M;
            desFrame.p = toKDL(p_.pos);

            // compute errors
            Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
            std::cout << "The error norm is : " << error.norm() << std::endl;

            if (cmd_interface_ == "position") {
                // Next Frame
                KDL::Frame nextFrame;
                nextFrame.M = cartpos.M;
                nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp * error)) * dt;

                // Compute IK
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
            } else if (cmd_interface_ == "velocity") {
                if (ctrl_ == "velocity_ctrl") {
                    Vector6d cartvel;
                    cartvel << p_.vel + Kp * error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                } else if (ctrl_ == "velocity_ctrl_null") {
                    Eigen::Matrix<double, 6, 1> error_position;
                    error_position << error, o_error;
                    joint_velocities_cmd_ = controller_.velocity_ctrl_null(error_position, Kp);
                } else if (ctrl_ == "vision") {
                    Eigen::Vector3d sd(0, 0, 1); // desired direction
                    // joint_velocities_cmd_ = controller_.vision_ctrl(Kp, cPo_, sd);
                }
            } else if (cmd_interface_ == "effort") {
                joint_efforts_cmd_.data[0] = 0.1 * std::sin(2 * M_PI * t_ / total_time);
            }

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            if (cmd_interface_ == "position") {
                // Set joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            } else if (cmd_interface_ == "velocity") {
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
            } else if (cmd_interface_ == "effort") {
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

            // Send joint velocity/position/effort commands
            if (cmd_interface_ == "position") {
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            } else if (cmd_interface_ == "velocity") {
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
            } else if (cmd_interface_ == "effort") {
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < sensor_msg.position.size(); i++) {
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped& pose_stamped_msg)
    {
        cPo_(0) = pose_stamped_msg.pose.position.x;
        cPo_(1) = pose_stamped_msg.pose.position.y;
        cPo_(2) = pose_stamped_msg.pose.position.z;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MarkerPoseSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Node::SharedPtr node_handle_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;

    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    Eigen::Vector3d cPo_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;

    trajectory_point p_;

    int iteration_;
    bool joint_state_available_;
    double t_;
    std::string cmd_interface_;
    std::string ctrl_;
    std::string traj_type_;
    std::string s_type_;

    KDL::Frame init_cart_pose_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&Iiwa_pub_sub::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution (Action Server)...");

        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        auto result = std::make_shared<ExecuteTrajectory::Result>();

        rclcpp::Rate rate(50);  // 50 Hz update
        double total_time = total_time_;
        double dt = 1.0 / 50.0;
        double t = 0.0;
        int Kp = Kp_;

        while (rclcpp::ok() && t < total_time && ctrl_ != "vision") {
            // if the client asks to cancel
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled by client");
                goal_handle->canceled(result);
                return;
            }

            if (traj_type_ == "linear") {
                if (s_type_ == "trapezoidal")
                    p_ = planner_.linear_traj_trapezoidal(t);
                else
                    p_ = planner_.linear_traj_cubic(t);
            }

            KDL::Frame cartpos = robot_->getEEFrame();

            Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));

            // publish the error as feedback
            feedback->position_error = {error(0), error(1), error(2)};
            goal_handle->publish_feedback(feedback);

            if (ctrl_ == "velocity_ctrl") {
                Vector6d cartvel;
                cartvel << p_.vel + Kp * error, Eigen::Vector3d::Zero();
                joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
            } else if (ctrl_ == "velocity_ctrl_null") {
                Eigen::Matrix<double, 6, 1> error_position;
                error_position << error, Eigen::Vector3d::Zero();
                joint_velocities_cmd_ = controller_.velocity_ctrl_null(error_position, Kp);
            }

            // Publishing the command
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.assign(joint_velocities_cmd_.data.data(),
                                joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
            cmdPublisher_->publish(cmd_msg);

            t += dt;
            rate.sleep();
        }

        while (rclcpp::ok() && ctrl_ == "vision") {
            Eigen::Vector3d s;
            for (int i = 0; i < 3; i++) {
                s(i) = cPo_(i) / cPo_.norm();
            }

            Eigen::Vector3d sd(0, 0, 1); // desired direction
            Eigen::Vector3d dir_error = s - sd;

            joint_velocities_cmd_ = controller_.vision_ctrl(Kp, cPo_, sd);

            feedback->position_error = std::array<double, 3>{dir_error(0), dir_error(1), dir_error(2)};
            goal_handle->publish_feedback(feedback);

            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.assign(joint_velocities_cmd_.data.data(),
                                joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
            cmdPublisher_->publish(cmd_msg);
        }

        // Finish
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully.");

        // Stop
        std_msgs::msg::Float64MultiArray cmd_msg;

        if (cmd_interface_ == "position") {
            cmd_msg.data.assign(joint_positions_cmd_.data.data(),
                                joint_positions_cmd_.data.data() + joint_positions_cmd_.rows());
        } else if (cmd_interface_ == "velocity" || cmd_interface_ == "effort") {
            cmd_msg.data.resize(joint_velocities_cmd_.rows());
            std::fill(cmd_msg.data.begin(), cmd_msg.data.end(), 0.0);
        }

        cmdPublisher_->publish(cmd_msg);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}