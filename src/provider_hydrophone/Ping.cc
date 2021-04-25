//
// Created by coumarc9 on 7/1/17.
//

#include "Ping.h"

namespace provider_hydrophone
{

    Ping::Ping(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3, double_t frequency)
    {
        frequency_ = frequency * 1.0;
    }

    Ping::~Ping() {

    }

    void Ping::calculateResults(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3)
    {

    }
}