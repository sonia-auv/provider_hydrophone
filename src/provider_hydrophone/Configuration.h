//
// Created by coumarc9 on 7/24/17.
//

#ifndef PROVIDER_HYDROPHONE_CONFIGURATION_H
#define PROVIDER_HYDROPHONE_CONFIGURATION_H

#include <cstdint>
#include <cmath>

namespace provider_hydrophone
{
    class Configuration {

    public:

        Configuration();
        ~Configuration();

        double_t getDistanceBetweenHydrophone() const;

        uint16_t getSoundSpeed() const;

        uint8_t getThreshold() const;

        uint8_t getGain() const;

    private:

        double_t distanceBetweenHydrophone;
        uint16_t soundSpeed;
        uint8_t threshold;
        uint8_t gain;
    };
}




#endif //PROVIDER_HYDROPHONE_CONFIGURATION_H
