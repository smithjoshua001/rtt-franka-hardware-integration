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

  addOperation("setTorques" , &Robot_data_test::setTrq, this, RTT::ClientThread);
}

bool Robot_data_test::configureHook(){

  return true;
}

bool Robot_data_test::startHook(){

  return true;
}

void Robot_data_test::updateHook(){
  joint_state_in_flow = joint_state_in_port.read(joint_state_in_data);
  grav_in_flow = grav_in_port.read(grav_in_data);

  out_trq_data.torques += grav_in_data;
  out_trq_data.torques += coriolis_in_data;

  RTT::log(RTT::Info) << out_trq_data.torques.transpose() << RTT::endlog();

  out_trq_port.write(out_trq_data);
}

void Robot_data_test::stopHook() {

}

void Robot_data_test::cleanupHook() {

}

void Robot_data_test::setTrq(int idx, double val){
  this->out_trq_data.torques(idx) = val;
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
