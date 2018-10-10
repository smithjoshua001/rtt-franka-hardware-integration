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
  done = false;
}

bool Robot_data_test::configureHook(){

  return true;
}

bool Robot_data_test::startHook(){

  return true;
}

void Robot_data_test::updateHook(){
    current_time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());


  joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);
  grav_in_flow = grav_in_port.read(grav_in_data);
  coriolis_in_flow = coriolis_in_port.read(coriolis_in_data);

  if(joint_state_in_flow != RTT::NoData) {
      //out_trq_data.torques = joint_state_in_data.torques;
      //out_trq_data.torques += grav_in_data;
      //out_trq_data.torques += coriolis_in_data;

//      out_vel_data.velocities = joint_state_in_data.velocities;
//      out_pos_data.angles = joint_state_in_data.angles;


  }

  if(current_time < end_time) {
    out_trq_data.torques(index) = tau * (current_time - start_time)/total_time;
    RTT::log(RTT::Info) << out_trq_data.torques(index) <<"\t"<< (current_time - start_time)/total_time<<RTT::endlog();
  } else {
      done = true;
  }

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

void Robot_data_test::ramp(int _idx, double _tau, double _tot) {
    if(!done) {
       return;
    }

    total_time = _tot;
    start_time = current_time;
    end_time = start_time + total_time;

    index = _idx;
    tau = _tau;
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
