//
// Created by coumarc9 on 7/24/17.
//

#include "Configuration.h"

namespace provider_hydrophone
{

    Configuration::Configuration() {

    }

    Configuration::~Configuration() {

    }

    double_t Configuration::getDistanceBetweenHydrophone() const {
        return distanceBetweenHydrophone;
    }

    uint16_t Configuration::getSoundSpeed() const {
        return soundSpeed;
    }

    uint8_t Configuration::getThreshold() const {
        return threshold;
    }

    uint8_t Configuration::getGain() const {
        return gain;
    }
}