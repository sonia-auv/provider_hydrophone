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

        double_t getDistanceBetweenHydrophone() const {return distanceBetweenHydrophone;};

        uint16_t getSoundSpeed() const {return soundSpeed;};

        uint8_t getThreshold() const {return threshold;};

        uint8_t getGain() const {return gain;};

        std::string getTtyPort() const {return ttyPort;}

    private:

        ros::NodeHandlePtr nh;

        double_t distanceBetweenHydrophone;
        int32_t soundSpeed;
        int32_t threshold;
        int32_t gain;

        std::string ttyPort;

        void Deserialize();
        void SetParameter();

        template <typename TType>
        void FindParameter(const std::string &paramName, TType &attribute);
        };
}




#endif //PROVIDER_HYDROPHONE_CONFIGURATION_H
