#include "robot_data_test-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Robot_data_test::Robot_data_test(std::string const& name) : TaskContext(name){
    joint_state_in_flow = RTT::NoData;
    joint_state_in_port.setName("joint_vals_in_port");
    joint_state_in_data.angles.setZero(7);
    ports()->addPort(joint_state_in_port);

    grav_in_flow = RTT::NoData;
    grav_in_port.setName("grav_in_port");
    grav_in_data.setZero(7);
    ports()->addPort(grav_in_port);

    coriolis_in_flow = RTT::NoData;
    coriolis_in_port.setName("coriolis_in_port");
    coriolis_in_data.setZero(7);
    ports()->addPort(coriolis_in_port);

    out_trq_port.setName("joint_trqs_out_port");
    out_trq_data.torques.setZero(7);
    out_trq_port.setDataSample(out_trq_data);
    ports()->addPort(out_trq_port);

    out_vel_port.setName("joint_vels_out_port");
    out_vel_data.velocities.setZero(7);
    out_vel_port.setDataSample(out_vel_data);
    this->addPort(out_vel_port);

    out_pos_port.setName("joint_pos_out_port");
    out_pos_data.angles.setZero(7);
    out_pos_port.setDataSample(out_pos_data);
    this->addPort(out_pos_port);

    addOperation("setTorque", &Robot_data_test::setTrq, this, RTT::ClientThread);
    addOperation("setVelocity", &Robot_data_test::setVel, this, RTT::ClientThread);
    addOperation("setPosition", &Robot_data_test::setPos, this, RTT::ClientThread);
    addOperation("ramp", &Robot_data_test::ramp, this, RTT::ClientThread);
    lock = false;
}

bool Robot_data_test::configureHook(){

    return true;
}

bool Robot_data_test::startHook(){
    if(out_pos_port.connected()) {
        RTT::log(RTT::Info) << "Starting test component in position mode" << RTT::endlog();
        ramp_input = &joint_state_in_data.angles;
        ramp_output = &out_pos_data.angles;
    } else if(out_vel_port.connected()) {
        RTT::log(RTT::Info) << "Starting test component in velocity mode" << RTT::endlog();
        ramp_input = &joint_state_in_data.velocities;
        ramp_output = &out_vel_data.velocities;
    } else {
        if(!out_trq_port.connected()) {
            RTT::log(RTT::Info) << "No output port connected, falling back to default" << RTT::endlog();
        }

        RTT::log(RTT::Info) << "Starting test component in torque mode" << RTT::endlog();
        ramp_input = &out_trq_data.torques;
        ramp_output = &out_trq_data.torques;
    }

    return true;
}

void Robot_data_test::updateHook(){
    // Get current time
    current_time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());

    // Read state from input ports
    joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);
    grav_in_flow = grav_in_port.read(grav_in_data);
    coriolis_in_flow = coriolis_in_port.read(coriolis_in_data);

    out_pos_data.angles = joint_state_in_data.angles;
    out_vel_data.velocities = joint_state_in_data.velocities;

    // Ramp up values
    if(current_time < end_time) {
        *ramp_output = qp.getQ(current_time);
    } else {
        lock = false;
    }

    // Write values to output ports
    out_trq_port.write(out_trq_data);
    out_vel_port.write(out_vel_data);
    out_pos_port.write(out_pos_data);
}

void Robot_data_test::stopHook() {

}

void Robot_data_test::cleanupHook() {
    out_trq_data.torques.setZero();
    out_vel_data.velocities.setZero();
    out_pos_data.angles.setZero();
}

void Robot_data_test::setTrq(int idx, float val) {
    this->out_trq_data.torques(idx) = val;
}

void Robot_data_test::setVel(int idx, float val) {
    this->out_vel_data.velocities(idx) = val;
}

void Robot_data_test::setPos(int idx, float val) {
    this->out_pos_data.angles(idx) = val;
}

void Robot_data_test::ramp(int idx, float target, double time) {
    if(lock) {
        return;
    }

    double total_time = time;
    double start_time = current_time;
    end_time = start_time + total_time;

    Eigen::VectorXf start_conf = *ramp_input;
    Eigen::VectorXf end_conf = start_conf;
    end_conf(idx) = target;

    qp = QuinticPolynomial<float>(start_time, end_time, start_conf, end_conf);

    lock = true;
}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Robot_data_test)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Robot_data_test)
