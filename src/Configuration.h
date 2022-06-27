//
// Created by coumarc9 on 7/24/17.
// Updated on 2022-06-24
//

#ifndef PROVIDER_HYDROPHONE_CONFIGURATION_H
#define PROVIDER_HYDROPHONE_CONFIGURATION_H

#include <cstdint>
#include <cmath>
#include <ros/ros.h>

namespace provider_hydrophone
{
    class Configuration 
    {

    public:

        Configuration(const ros::NodeHandlePtr &nh);
        ~Configuration();

        // For PGA gain
        uint8_t getGain() const {return gain;};

        // For DOA
        uint16_t getsnrThreshold() const {return snrThreshold;};
        uint16_t getHighSignalThreshold() const {return highSignalThreshold;};
        uint16_t getLowSignalThreshold() const {return lowSignalThreshold;};

        // For AGC
        bool getAGCactivation() const {return agcActivation;};
        uint16_t getAGCThreshold() const {return agcThreshold;};
        uint16_t getAGCMaxThreshold() const {return agcMaxThreshold;};

        // For USB
        std::string getTtyPort() const {return ttyPort;};

    private:

        ros::NodeHandlePtr nh_;

        int32_t gain;

        int32_t snrThreshold;
        int32_t highSignalThreshold;
        int32_t lowSignalThreshold;
        
        bool agcActivation;
        int32_t agcThreshold;
        int32_t agcMaxThreshold;

        std::string ttyPort;

        void Deserialize();
        void SetParameter();

        template <typename TType>
        void FindParameter(const std::string &paramName, TType &attribute);
    };
}




#endif //PROVIDER_HYDROPHONE_CONFIGURATION_H
