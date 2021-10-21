//
// Created by coumarc9 on 7/24/17.
//

#include "Configuration.h"

namespace provider_hydrophone
{

    Configuration::Configuration(const ros::NodeHandlePtr &nh)
        : nh_(nh),
          snrThreshold(10),
          signalThreshold(35000),
          gain(4),
          ttyPort("/dev/ttyUSB0")
    {
        Deserialize();
    }

    Configuration::~Configuration() {}

    void Configuration::Deserialize() {

        ROS_DEBUG("Deserialize params");

        FindParameter("/hydrophone/snr_threshold", snrThreshold);
        FindParameter("/hydrophone/signal_threshold", signalThreshold);
        FindParameter("/hydrophone/gain", gain);
        FindParameter("/connection/tty_port", ttyPort);

        ROS_DEBUG("End deserialize params");
    }

    template <typename TType>
    void Configuration::FindParameter(const std::string &paramName, TType &attribute) {
        if (nh_->hasParam("/provider_underwater_com" + paramName)) {
            nh_->getParam("/provider_underwater_com" + paramName, attribute);
        } else {
            ROS_WARN_STREAM("Did not find /provider_hydrophone" + paramName
                                    << ". Using default.");
        }
    }
}