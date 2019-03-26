//
// Created by coumarc9 on 7/24/17.
//

#include "Configuration.h"

namespace provider_hydrophone
{

    Configuration::Configuration(const ros::NodeHandlePtr &nh)
        : nh(nh),
          distanceBetweenHydrophone(0.015),
          soundSpeed(1484),
          threshold(4),
          gain(2),
          ttyPort("/dev/HYDRO")
    {
        Deserialize();
    }

    Configuration::~Configuration() {}

    void Configuration::Deserialize() {

        ROS_INFO("Deserialize params");

        FindParameter("/hydrophone/distance_between_hydrophone", distanceBetweenHydrophone);
        FindParameter("/hydrophone/sound_speed", soundSpeed);
        FindParameter("/hydrophone/threshold", threshold);
        FindParameter("/hydrophone/gain", gain);

        FindParameter("/connection/tty_port", ttyPort);

        ROS_INFO("End deserialize params");
    }

    template <typename TType>
    void Configuration::FindParameter(const std::string &paramName, TType &attribute) {
        if (nh->hasParam("/provider_hydrophone" + paramName)) {
            nh->getParam("/provider_hydrophone" + paramName, attribute);
        } else {
            ROS_WARN_STREAM("Did not find /provider_hydrophone" + paramName
                                    << ". Using default.");
        }
    }

}