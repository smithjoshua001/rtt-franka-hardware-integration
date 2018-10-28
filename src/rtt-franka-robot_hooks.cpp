#include <rtt-franka-robot.hpp>
#include <Eigen/Dense>

using namespace franka;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void franka_robot::updateHook() {
    // check if the component is stopped, then do not execute the updateHook anymore.
    if (!isRunning())
        return;

    for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->sense();         // use return value to see if there is a connection!

    for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->getCommand();

    for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++){
        try { 
            it->second->move();
        } catch(...) {
            this->getActivity()->thread()->yield();
            this->stop();
        }
    }
    // execution action is called to trigger this updateHook again.
    if(!this->getActivity()->isPeriodic()){
    this->trigger();
    }
}

void franka_robot::stopHook() {
    // redirect the stop signal to the kinematic-chains to handle the actual stop.
    for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->stop();
}
