#ifndef OROCOS_ROBOT_DATA_TEST_COMPONENT_HPP
#define OROCOS_ROBOT_DATA_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <Eigen/Core>
#include "QuinticPolynomial.hpp"

class Robot_data_test : public RTT::TaskContext{
public:
    Robot_data_test(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    RTT::InputPort<rstrt::robot::JointState> joint_state_in_port;
    RTT::FlowStatus joint_state_in_flow;
    rstrt::robot::JointState joint_state_in_data;

    RTT::InputPort<Eigen::VectorXf> grav_in_port;
    RTT::FlowStatus grav_in_flow;
    Eigen::VectorXf grav_in_data;

    RTT::InputPort<Eigen::VectorXf> coriolis_in_port;
    RTT::FlowStatus coriolis_in_flow;
    Eigen::VectorXf coriolis_in_data;

    RTT::OutputPort<rstrt::dynamics::JointTorques> out_trq_port;
    rstrt::dynamics::JointTorques out_trq_data;

    RTT::OutputPort<rstrt::kinematics::JointVelocities> out_vel_port;
    rstrt::kinematics::JointVelocities out_vel_data;

    RTT::OutputPort<rstrt::kinematics::JointAngles> out_pos_port;
    rstrt::kinematics::JointAngles out_pos_data;

    void setTrq(int idx, float val);
    void setVel(int idx, float val);
    void setPos(int idx, float val);

    QuinticPolynomial<float> qp;
    Eigen::VectorXf start_conf, end_conf;
    double current_time = 0, start_time = 0, end_time = 0, total_time = 0;
    int idx;
    bool lock;
    void ramp(int _idx, float _tau, double _tot);
};
#endif
