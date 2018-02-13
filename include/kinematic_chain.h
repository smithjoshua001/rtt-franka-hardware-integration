#ifndef _KINEMATIC_FRANKA_CHAIN_H_
#define _KINEMATIC_FRANKA_CHAIN_H_

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/robot/JointState.hpp>

#include <robot_control.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <control_modes.h>
#include <robot_impl.h>
#include <motion_generator_traits.h>
#include <research_interface/robot/rbk_types.h>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <memory>
#include <array>

class KinematicChain {
    public:
        /**
         * Contructor to set up a kinematic chain.
         *
         * @param chain_name name of the chain.
         * @param joint_names list o joint names.
         * @param ports pointer to the ports of the enclosing component.
         * @param friInst remote interface of FRI.
         */
        KinematicChain(const std::string &chain_name,
                       const std::vector < std::string > &joint_names,
                       RTT::DataFlowInterface &ports,
                       std::unique_ptr < franka::Robot::Impl > franka_control);

        ~KinematicChain() {
            // delete motion_command;
            // delete control_command;
        }

        /**
         * Gets the name of the kinematic chain
         * @return Name of the kinematic chain
         */
        std::string getKinematicChainName();

        unsigned int getNumberOfDOFs();

        const std::string &getCurrentControlMode();

        // std::vector < std::string > &getJointNames();

        std::vector < std::string > getControllersAvailable() {
            std::vector < std::string > output;
            for (auto const &x : franka::ControlModeMap) {
                output.push_back(x.second);
            }
            return output;
        }

        bool initKinematicChain();

        bool resetKinematicChain();

        bool startKinematicChain();

        void setFeedBack();

        bool setControlMode(const std::string &controlMode);

        bool sense();

        void getCommand();

        void move();

        void stop();

        std::string printKinematicChainInformation();

	void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                                 const std::array<double, 7>& upper_torque_thresholds,
                                 const std::array<double, 6>& lower_force_thresholds,
                                 const std::array<double, 6>& upper_force_thresholds);

 void setFilters(double joint_position_filter_frequency,
                       double joint_velocity_filter_frequency,
                       double cartesian_position_filter_frequency,
                       double cartesian_velocity_filter_frequency,
                       double controller_filter_frequency);

    private:
        std::string kinematic_chain_name;
        int dof;
        franka::ControlModes current_control_mode;
        RTT::DataFlowInterface &ports;
        std::unique_ptr < franka::RobotControl > franka_control;
        static constexpr research_interface::robot::Move::Deviation kDefaultDeviation {
            10.0, 3.12, 2 * M_PI
        };
        research_interface::robot::MotionGeneratorCommand motion_command;
        research_interface::robot::ControllerCommand control_command;

        franka::RobotState franka_state;
        std::unique_ptr < franka::Model > franka_model;

        std::unique_ptr < franka::BaseJointController > jc;
        std::unique_ptr < franka::JointFeedback < rstrt::robot::JointState >> jf;
        std::unique_ptr < franka::DynamicFeedback < Eigen::MatrixXf >> inertia_feedback;
        std::unique_ptr < franka::DynamicFeedback < Eigen::VectorXf >> coriolis_feedback;
        std::unique_ptr < franka::DynamicFeedback < Eigen::VectorXf >> gravity_feedback;
        
        std::unique_ptr < franka::DynamicFeedback < Eigen::MatrixXf >> jacobian_feedback;
        std::array < double, 7 > *current_control_input_var;
        uint32_t motion_id;
};

#endif
