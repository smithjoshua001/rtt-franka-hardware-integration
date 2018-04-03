#include <rtt/Operation.hpp>
#include <rtt-franka-robot.hpp>

using namespace franka;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

franka_robot::franka_robot(const std::string &name) :
    TaskContext(name), is_configured(false) {
    this->addOperation("getModel", &franka_robot::getModel, this, ClientThread);

    this->addOperation("setControlMode", &franka_robot::setControlMode, this,
                       RTT::ClientThread);

    this->addOperation("setGravity", &franka_robot::setGravity, this,
                       RTT::ClientThread);

    // this->addOperation("setDebug", &franka_robot::setDebug, this,
    //                    RTT::ClientThread);

    this->addOperation("getKinematicChains", &franka_robot::getKinematicChains,
                       this, RTT::ClientThread);

    this->addOperation("printKinematicChainInformation",
                       &franka_robot::printKinematicChainInformation, this,
                       RTT::ClientThread);

    this->addOperation("getControlMode", &franka_robot::getControlMode, this,
                       RTT::ClientThread);

    this->addOperation("getAvailableControlMode",
                       &franka_robot::getControlAvailableMode, this, RTT::ClientThread);
    this->addOperation("loadURDFAndSRDF", &franka_robot::loadURDFAndSRDF, this,
                       RTT::ClientThread);

    this->addOperation("reset_model_configuration",
                       &franka_robot::resetModelConfiguration, this, RTT::ClientThread);
    this->addOperation("addChain", &franka_robot::addChain, this,
                       RTT::ClientThread);

    // Default IP of this computer
    this->ip_addr = "192.168.0.1";
    this->addProperty("ip_addr", ip_addr).doc("IP address of the computer");

    models_loaded = false;
}

bool franka_robot::resetModelConfiguration() {
    bool reset = true;
    std::map<std::string, std::shared_ptr<KinematicChain> >::iterator it;
    for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        reset = reset && it->second->resetKinematicChain();
    return reset;
}

std::string franka_robot::printKinematicChainInformation(
    const std::string &kinematic_chain) {
    std::vector<std::string> chain_names = getKinematicChains();
    if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
          != chain_names.end())) {
        log(Warning) << "Kinematic Chain " << kinematic_chain
                     << " is not available!" << endlog();
        return "";
    }

    return kinematic_chains[kinematic_chain]->printKinematicChainInformation();
}

std::string franka_robot::getControlMode(const std::string &kinematic_chain) {
    std::vector<std::string> chain_names = getKinematicChains();
    if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
          != chain_names.end())) {
        log(Warning) << "Kinematic Chain " << kinematic_chain
                     << " is not available!" << endlog();
        return "";
    }

    return kinematic_chains[kinematic_chain]->getCurrentControlMode();
}

std::vector<std::string> franka_robot::getControlAvailableMode(
    const std::string &kinematic_chain) {
    std::vector<std::string> control_modes;

    // std::vector<std::string> chain_names = getKinematicChains();
    if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
          != chain_names.end())) {
        log(Warning) << "Kinematic Chain " << kinematic_chain
                     << " is not available!" << endlog();
        control_modes.push_back("");
    } else
        control_modes =
            kinematic_chains[kinematic_chain]->getControllersAvailable();
    return control_modes;
}

std::vector<std::string> franka_robot::getKinematicChains() {
    std::vector<std::string> chains;
    for (std::map<std::string, std::shared_ptr<KinematicChain> >::iterator it =
             kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        chains.push_back(it->first);
    return chains;
}

bool franka_robot::setControlMode(const std::string &kinematic_chain,
                                  const std::string &controlMode) {
    // std::vector<std::string> chain_names = getKinematicChains();
    RTT::log(RTT::Info) << "Setting Control Mode" << RTT::endlog();
    if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
          != chain_names.end())) {
        log(Warning) << "Kinematic Chain " << kinematic_chain
                     << " is not available!" << endlog();
        return false;
    }
    RTT::log(RTT::Info) << "Setting Control Mode KC" << RTT::endlog();
    return kinematic_chains[kinematic_chain]->setControlMode(controlMode);
}

void franka_robot::setGravity(const std::string &kinematic_chain, const bool g) {
    // std::vector<std::string> chain_names = getKinematicChains();
    if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
          != chain_names.end())) {
        log(Warning) << "Kinematic Chain " << kinematic_chain
                     << " is not available!" << endlog();
    }
    // kinematic_chains[kinematic_chain]->setGravity(g);
}

bool franka_robot::getModel(const std::string &model_name) {
    if (model) {
        log(Warning) << "Model [" << model_name << "] already loaded !"
                     << endlog();
        return true;
    }
    //parse URDF
    model = urdf::parseURDFFile(model_name);
    return bool(model);
}

bool franka_robot::startHook() {
    for (const std::string &chain_name : chain_names) {
        if (!(kinematic_chains[chain_name]->startKinematicChain())) {
            RTT::log(RTT::Warning) << "Problem Starting Kinematic Chain"
                                   << chain_name << RTT::endlog();
            return false;
        }
    }
    return true;
}

bool franka_robot::configureHook() {
    //might not need the model stuff for this component or maybe have it in this component and provide to everything else?
    if (!models_loaded || !bool(model)) {
        RTT::log(RTT::Error)
            << "URDF and SRDF models has not been passed. Call loadURDFAndSRDF(URDF_path, SRDF_path)"
            << RTT::endlog();
        return false;
    }
    urdf_joints_ = model->joints_;
    urdf_links_ = model->links_;
    RTT::log(RTT::Info) << "Model name " << model->getName() << RTT::endlog();
    RTT::log(RTT::Info) << "Model has " << urdf_joints_.size() << " joints"
                        << RTT::endlog();
    //RTT::log(RTT::Info) << kinematic_chains["Franka"]->getKinematicChainName() << "Model has " << urdf_links_.size() << " links"<< RTT::endlog();

    for (const std::string &chain_name : chain_names) {
        if (!(kinematic_chains[chain_name]->initKinematicChain())) {
            RTT::log(RTT::Warning) << "Problem Init Kinematic Chain"
                                   << chain_name << RTT::endlog();
            return false;
        }
    }
    RTT::log(RTT::Info) << "Kinematic Chains Initialized!" << RTT::endlog();

    RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();
    return true;
}

bool franka_robot::loadURDFAndSRDF(const std::string &URDF_path,
                                   const std::string &SRDF_path) {
    RTT::log(RTT::Info) << "LOADING URDF AND SRDF!!!!" << RTT::endlog();
    if (!models_loaded) {
        // get the URDF and SRDF paths.
        // TODO check if the files exist!
        std::string _urdf_path = URDF_path;
        std::string _srdf_path = SRDF_path;

        RTT::log(RTT::Info) << "URDF path: " << _urdf_path << RTT::endlog();
        RTT::log(RTT::Info) << "SRDF path: " << _srdf_path << RTT::endlog();

        models_loaded = _xbotcore_model.init(_urdf_path, _srdf_path);

        for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i) {
            RTT::log(RTT::Info) << "chain #" << i << " " << _xbotcore_model.get_chain_names()[i] << RTT::endlog();

            // get the enabled joints from the parsed URDF and SRDF from xbotcoremodel.
            std::vector<std::string> enabled_joints_in_chain_i;
            _xbotcore_model.get_enabled_joints_in_chain(_xbotcore_model.get_chain_names()[i], enabled_joints_in_chain_i);

            for (unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j) {
                RTT::log(RTT::Info) << "  " << enabled_joints_in_chain_i[j] << RTT::endlog();
            }
        }
        // store the parsed URDF model from xbotcoremodel.
        model = _xbotcore_model.get_urdf_model();
    } else {
        RTT::log(RTT::Info) << "URDF and SRDF have been already loaded!" << RTT::endlog();
    }

    RTT::log(RTT::Info) << "MODEL LOADED" << RTT::endlog();
    return models_loaded;
}

bool franka_robot::addChain(std::string name, std::string robot_ip,
                            const std::string &URDF_path, const std::string &SRDF_path) {
    RTT::log(RTT::Info) << "ADDING CHAIN!!!!" << RTT::endlog();
    // load model and joints from the URDF and SRDF.
    loadURDFAndSRDF(URDF_path, SRDF_path);
    // get the names of the parsed chains from SRDF.
    std::string chain_name = _xbotcore_model.get_chain_names()[0];
    // for (size_t i = 0; i < _xbotcore_model.get_chain_names().size(); i++) {
    //     if (_xbotcore_model.get_chain_names()[i] == name) {
    //         chain_name = name;
    //         break;
    //     }
    // }
    // std::cout << "HERE!!!!\n";
    // if (chain_name == "") {
    //     RTT::log(RTT::Error) << "CHAIN WAS NOT FOUND!!!" << RTT::endlog();
    //     return false;
    // }

    RTT::log(RTT::Info) << chain_name << " :chain" << RTT::endlog();
    // get the enabled joints. ################################################################## TODO DUPLICATE!!!
    std::vector<std::string> enabled_joints_in_chain;
    _xbotcore_model.get_enabled_joints_in_chain(chain_name,
                                                enabled_joints_in_chain);
    RTT::log(RTT::Info) << "NUMBER OF ENABLED JOINTS" << enabled_joints_in_chain.size() << RTT::endlog();
    RTT::log(RTT::Info) << name << ": NAME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << RTT::endlog();
    chain_names.push_back(name);
    // get the enabled joints. ################################################################## TODO DUPLICATE!!!
    // kinematic_chains.insert(
    //     std::pair<std::string, std::shared_ptr<KinematicChain> >(name,
    //                                                              std::make_shared<KinematicChain>(
    //                                                                  /** choose a name. */
    //                                                                  chain_name,
    //                                                                  /** add the enabled joints to control. */
    //                                                                  enabled_joints_in_chain,
    //                                                                  /** pass the pointer to the ports to enable the kinematic chain object to add ports. */
    //                                                                  *(this->ports()),
    //                                                                  std::make_unique<franka::Robot::Impl>(std::make_unique<Network>(robot_ip, research_interface::robot::kCommandPort), RealtimeConfig::kEnforce)
    //                                                                  )
    //                                                              )
    //     );
    kinematic_chains[name] = std::make_shared<KinematicChain>(chain_name, enabled_joints_in_chain, *(this->ports()), std::make_unique<franka::Robot::Impl>(std::make_unique<Network>(robot_ip, research_interface::robot::kCommandPort),50, RealtimeConfig::kEnforce));
    return true;
}

ORO_CREATE_COMPONENT_LIBRARY() ORO_LIST_COMPONENT_TYPE(franka::franka_robot)
