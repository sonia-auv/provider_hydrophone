//
// Created by coumarc9 on 7/24/17.
//

#ifndef PROVIDER_HYDROPHONE_CONFIGURATION_H
#define PROVIDER_HYDROPHONE_CONFIGURATION_H

#include <cstdint>
#include <cmath>
#include <ros/ros.h>

namespace provider_hydrophone
{
    class Configuration {

    public:

        Configuration(const ros::NodeHandlePtr &nh);
        ~Configuration();

        uint16_t getsnrThreshold() const {return snrThreshold;};

        uint16_t getSignalThreshold() const {return signalThreshold;};

        uint8_t getGain() const {return gain;};

        std::string getTtyPort() const {return ttyPort;};

    private:

        ros::NodeHandlePtr nh_;

        int32_t snrThreshold;
        int32_t signalThreshold;
        int32_t gain;
        std::string ttyPort;

        void Deserialize();
        void SetParameter();

        template <typename TType>
        void FindParameter(const std::string &paramName, TType &attribute);
        };
}




#endif //PROVIDER_HYDROPHONE_CONFIGURATION_H
