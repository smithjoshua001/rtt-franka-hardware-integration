#include <kinematic_chain.h>

constexpr research_interface::robot::Move::Deviation KinematicChain::kDefaultDeviation;

KinematicChain::KinematicChain(const std::string &chain_name,
                               const std::vector<std::string> &joint_names,
                               RTT::DataFlowInterface &ports_in,
                               std::unique_ptr < franka::Robot::Impl > franka_control)
    : kinematic_chain_name(chain_name),
    ports(ports_in) {
    dof = joint_names.size();
    franka_model = std::make_unique<franka::Model>(franka_control->loadModel());

    this->franka_control = std::move(franka_control);
    current_control_mode = franka::ControlModes::Torque;
    // motion_command = new research_interface::robot::MotionGeneratorCommand{};
    // control_command = new research_interface::robot::ControllerCommand{};
}

bool KinematicChain::initKinematicChain() {
    RTT::log(RTT::Info) << kinematic_chain_name << "\n";
    setFeedBack();
    setCollisionBehavior(
        {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
        {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}});
    setFilters(
      1000, 1000,
      1000, 1000,
      1000);
    return true;
}

bool KinematicChain::resetKinematicChain() {
    return true;
}

std::string KinematicChain::getKinematicChainName() {
    return kinematic_chain_name;
}

unsigned int KinematicChain::getNumberOfDOFs() {
    return dof;
}

const std::string &KinematicChain::getCurrentControlMode() {
    return franka::ControlModeMap.find(current_control_mode)->second;
}

void KinematicChain::setFeedBack() {
    jf = std::make_unique<franka::JointFeedback<rstrt::robot::JointState> >(kinematic_chain_name, this->ports, "_JointFeedback", [this](rstrt::robot::JointState &in) -> void {in = rstrt::robot::JointState(dof); in.angles.setZero(); in.velocities.setZero(); in.torques.setZero();});
    inertia_feedback = std::make_unique<franka::DynamicFeedback<Eigen::MatrixXf> >(kinematic_chain_name, this->ports, "_InertiaFeedback", [this](Eigen::MatrixXf &in) -> void {in(dof, dof); in.setZero();});
    coriolis_feedback = std::make_unique<franka::DynamicFeedback<Eigen::VectorXf> >(kinematic_chain_name, this->ports, "_CoriolisFeedback", [this](Eigen::VectorXf &in) -> void {in(dof); in.setZero();});
    gravity_feedback = std::make_unique<franka::DynamicFeedback<Eigen::VectorXf> >(kinematic_chain_name, this->ports, "_GravityFeedback", [this](Eigen::VectorXf &in) -> void {in(dof); in.setZero();});
    jacobian_feedback = std::make_unique<franka::DynamicFeedback<Eigen::MatrixXf> >(kinematic_chain_name, this->ports, "_JacobianFeedback", [this](Eigen::MatrixXf &in) -> void {in(6, dof); in.setZero();});
}

// bool KinematicChain::setController(const std::string &controller_type) {}
bool KinematicChain::setControlMode(const std::string &controlMode) {
    // if (!motion_command) {
    //     RTT::log(RTT::Info) << "FAILURE!!!!!!!!!" << RTT::endlog();
    // }
    RTT::log(RTT::Info) << "KC set control mode " << this->kinematic_chain_name << RTT::endlog();

    motion_command.q_d.fill(0);
    motion_command.dq_d.fill(0);
    motion_command.O_T_EE_d.fill(0);
    motion_command.O_dP_EE_d.fill(0);
    motion_command.elbow_d.fill(0);

    control_command.tau_J_d.fill(0);
    RTT::log(RTT::Info) << "fill 0 end " << franka::ControlModeMap.find(franka::ControlModes::Torque)->second << RTT::endlog();

    if (controlMode == franka::ControlModeMap.find(franka::ControlModes::Torque)->second) {
        RTT::log(RTT::Info) << "found map string" << RTT::endlog();
        current_control_mode = franka::ControlModes::Torque;
        RTT::log(RTT::Info) << "To torque!" << RTT::endlog();
        jc = std::make_unique<franka::JointController<rstrt::dynamics::JointTorques> >(kinematic_chain_name,
                                                                                       this->ports,
                                                                                       franka::ControlModes::Torque,
                                                                                       [](rstrt::dynamics::JointTorques &input) -> Eigen::VectorXf & {return input.torques;});
        RTT::log(RTT::Info) << "Created relavant controller" << RTT::endlog();
        current_control_input_var = &(control_command.tau_J_d);
    } else if(controlMode == franka::ControlModeMap.find(franka::ControlModes::Velocity)->second) {
        RTT::log(RTT::Info) << "Found mapping string" << RTT::endlog();
        current_control_mode = franka::ControlModes::Velocity;
        jc = std::make_unique<franka::JointController<rstrt::kinematics::JointVelocities>>(kinematic_chain_name,
                                                                                           this->ports,
                                                                                           franka::ControlModes::Velocity,
                                                                                           [](rstrt::kinematics::JointVelocities &input) -> Eigen::VectorXf& {return input.velocities;});
        RTT::log(RTT::Info) << "Set control mode to velocity" << RTT::endlog();
        current_control_input_var = &(motion_command.dq_d);
    } else if(controlMode == franka::ControlModeMap.find(franka::ControlModes::Position)->second) {
        RTT::log(RTT::Info) << "Found mapping string" << RTT::endlog();
        current_control_mode = franka::ControlModes::Position;
        jc = std::make_unique<franka::JointController<rstrt::kinematics::JointAngles>>(kinematic_chain_name,
                                                                                       this->ports,
                                                                                       franka::ControlModes::Position,
                                                                                       [](rstrt::kinematics::JointAngles &input) -> Eigen::VectorXf& {return input.angles;});
        RTT::log(RTT::Info) << "Set control mode to position" << RTT::endlog();
        current_control_input_var = &(motion_command.dq_d);
    } else {
        RTT::log(RTT::Error) << "Control Mode has not been implemented " << controlMode << RTT::endlog();
        return false;
    }
    return true;
}

bool KinematicChain::startKinematicChain() {
    if (!franka_control) {
        return false;
    }
    switch (current_control_mode) {
    case franka::ControlModes::Torque:
        RTT::log(RTT::Info) << "STARTED KINEMATIC CHAIN IN MODE: " << franka::ControlModeMap.find(franka::ControlModes::Torque)->second << RTT::endlog();
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kExternalController, franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, kDefaultDeviation, kDefaultDeviation);
        break;
    case franka::ControlModes::Velocity:
        RTT::log(RTT::Info) << "STARTED KINEMATIC CHAIN IN MODE: " << franka::ControlModeMap.find(franka::ControlModes::Velocity)->second << RTT::endlog();
        // TODO?
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kExternalController, franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, kDefaultDeviation, kDefaultDeviation);
        break;
    case franka::ControlModes::Position:
        RTT::log(RTT::Info) << "STARTED KINEMATIC CHAIN IN MODE: " << franka::ControlModeMap.find(franka::ControlModes::Velocity)->second << RTT::endlog();
        // TODO?
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kExternalController, franka::MotionGeneratorTraits<franka::JointPositions>::kMotionGeneratorMode, kDefaultDeviation, kDefaultDeviation);
        break;
    default:
        return false;
    }
    //franka_state = franka_control->update();
    return true;
}

bool KinematicChain::sense() {
    // RTT::log(RTT::Info) << "UPDATE!!" << RTT::endlog();

    // RTT::log(RTT::Info) << "MAP!!" << RTT::endlog();
    jf->joint_feedback.angles = Eigen::Map<Eigen::VectorXd>(franka_state.q.data(), dof).cast<float>();
    jf->joint_feedback.velocities = Eigen::Map<Eigen::VectorXd>(franka_state.dq.data(), dof).cast<float>();
    jf->joint_feedback.torques = Eigen::Map<Eigen::VectorXd>(franka_state.tau_J.data(), dof).cast<float>();
    // RTT::log(RTT::Info) << "MAP FINISH!!" << RTT::endlog();
    if (inertia_feedback->connected())
        inertia_feedback->dynamicFeedback = Eigen::Map<Eigen::MatrixXd>(franka_model->mass(franka_state).data(), dof, dof).cast<float>();

    if (coriolis_feedback->connected())
        coriolis_feedback->dynamicFeedback = Eigen::Map<Eigen::VectorXd>(franka_model->coriolis(franka_state).data(), dof).cast<float>();

    if (gravity_feedback->connected())
        gravity_feedback->dynamicFeedback = Eigen::Map<Eigen::VectorXd>(franka_model->gravity(franka_state).data(), dof).cast<float>();

    if (jacobian_feedback->connected())
        jacobian_feedback->dynamicFeedback = Eigen::Map<Eigen::MatrixXd>(franka_model->zeroJacobian(franka::Frame::kFlange,franka_state).data(), 6, dof).cast<float>();

    RTT::log(RTT::Info) << "WRITE!!"<< franka_state.control_command_success_rate << ", "<<std::to_string(franka_state.control_command_success_rate>0.8) << RTT::endlog();
    if(franka_state.control_command_success_rate>0.8){
	    jf->write();
	    if (inertia_feedback->connected())
	        inertia_feedback->write();
	    if (coriolis_feedback->connected())
	        coriolis_feedback->write();
	    if (gravity_feedback->connected())
	        gravity_feedback->write();
	    if (jacobian_feedback->connected())
	        jacobian_feedback->write();
	}


    return true;
}

void KinematicChain::getCommand() {
    if (jc->connected() && jc->read() != RTT::NoData) {
        //std::copy(jc->value().data(), jc->value().data() + 7, (*current_control_input_var).begin());
        // RTT::log(RTT::Info) << "COMMAND  ";
        for (size_t i = 0; i < 7; i++) {
	     (*current_control_input_var)[i] = jc->value()(i);
             //RTT::log(RTT::Info) << current_control_input_var->at(i) << " ";
        }
        //RTT::log(RTT::Info) << RTT::endlog();
    } else {
        RTT::log(RTT::Debug) << "No data from control input " << franka::ControlModeMap.find(current_control_mode)->second << RTT::endlog();
    }
}

void KinematicChain::stop() try {
    //franka_control->cancelMotion(motion_id);
    franka_control->finishMotion(motion_id, &motion_command, &control_command);
} catch (...) {
    try {
        franka_control->cancelMotion(motion_id);
    } catch (...) {}
    throw;
}

void KinematicChain::move() try {
    // if (current_control_mode == franka::ControlModes::Torque) {
    if (jc->connected() /*&& jc->read() != RTT::NoData*/) {
        franka_state = franka_control->update(&motion_command, &control_command);
    }else{
	franka_state = franka_control->update();
    }
    // }
    franka_control->throwOnMotionError(franka_state, motion_id);
} catch (const franka::NetworkException &exc) {
    RTT::log(RTT::Error)<<"NETWORK: "<<exc.what()<<RTT::endlog();
    throw;
} catch (const std::exception &exc) {
    RTT::log(RTT::Error)<<exc.what()<<RTT::endlog();
    //try {
    //    franka_control->cancelMotion(motion_id);
    //} catch (...) {}
    throw;
}

void KinematicChain::setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                                 const std::array<double, 7>& upper_torque_thresholds,
                                 const std::array<double, 6>& lower_force_thresholds,
                                 const std::array<double, 6>& upper_force_thresholds) {
  ((franka::Robot::Impl*)franka_control.get())->executeCommand<research_interface::robot::SetCollisionBehavior>(
      lower_torque_thresholds, upper_torque_thresholds, lower_torque_thresholds,
      upper_torque_thresholds, lower_force_thresholds, upper_force_thresholds,
      lower_force_thresholds, upper_force_thresholds);
}

void KinematicChain::setFilters(double joint_position_filter_frequency,
                       double joint_velocity_filter_frequency,
                       double cartesian_position_filter_frequency,
                       double cartesian_velocity_filter_frequency,
                       double controller_filter_frequency) {
  ((franka::Robot::Impl*)franka_control.get())->executeCommand<research_interface::robot::SetFilters>(
      joint_position_filter_frequency, joint_velocity_filter_frequency,
      cartesian_position_filter_frequency, cartesian_velocity_filter_frequency,
      controller_filter_frequency);
}

std::string KinematicChain::printKinematicChainInformation() {
    // std::stringstream joint_names_stream;
    // for (unsigned int i = 0; i < _joint_names.size(); ++i)
    //     joint_names_stream << _joint_names[i] << " ";
    //
    // auto &controller_names = getControllersAvailable();
    // std::stringstream controller_names_stream;
    // for (unsigned int i = 0; i < controller_names.size(); ++i)
    //     controller_names_stream << controller_names[i] << " ";
    //
    // std::stringstream info;
    // info << "Kinematic Chain: " << _kinematic_chain_name << std::endl;
    // info << "    Number of DOFs: " << _number_of_dofs << std::endl;
    // info << "    Joints:  [" << joint_names_stream.str() << "]" << std::endl;
    // info << "    Control Modes:  [ " << controller_names_stream.str() << "]"
    //      << std::endl;
    // info << "    Current Control Mode: " << _current_control_mode << std::endl;
    // info << "    Impedance: Stiffness = " << trqModeJntImpedance.stiffness << std::endl;
    // info << "    Impedance: Damping = " << trqModeJntImpedance.damping << std::endl;
    // info << "    overrideFakeImpedance = " << _overrideFakeImpedance << std::endl;

    return "TO IMPLEMENT!";
}
