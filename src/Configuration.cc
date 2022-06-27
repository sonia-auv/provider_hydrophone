//
// Created by coumarc9 on 7/24/17.
// Updated on 2022-06-24
//

#include "Configuration.h"

namespace provider_hydrophone
{

    Configuration::Configuration(const ros::NodeHandlePtr &nh)
        : nh_(nh),
          gain(2),
          snrThreshold(10),
          highSignalThreshold(35000),
          lowSignalThreshold(30000),
          agcActivation(false),
          agcThreshold(35000),
          agcMaxThreshold(55000),
          ttyPort("/dev/ttyUSB0")
    {
        Deserialize();
    }

    Configuration::~Configuration() {}

    void Configuration::Deserialize() {

        ROS_DEBUG("Deserialize params");

        FindParameter("/hydrophone/gain", gain);

        FindParameter("/hydrophone/snr_threshold", snrThreshold);
        FindParameter("/hydrophone/signal_high_threshold", highSignalThreshold);
        FindParameter("/hydrophone/signal_low_threshold", lowSignalThreshold);

        FindParameter("/hydrophone/agc_on", agcActivation);
        FindParameter("/hydrophone/agc_threshold", agcThreshold);
        FindParameter("/hydrophone/agc_max_threshold", agcMaxThreshold);
        
        FindParameter("/connection/tty_port", ttyPort);

        ROS_DEBUG("End deserialize params");
    }

    template <typename TType>
    void Configuration::FindParameter(const std::string &paramName, TType &attribute) {
        if (nh_->hasParam("/provider_hydrophone" + paramName)) {
            nh_->getParam("/provider_hydrophone" + paramName, attribute);
        } else {
            ROS_WARN_STREAM("Did not find /provider_hydrophone" + paramName
                                    << ". Using default.");
        }
    }
}