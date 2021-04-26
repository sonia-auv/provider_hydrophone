//
// Created by coumarc9 on 7/1/17.
//

#include "Ping.h"

namespace provider_hydrophone
{

    Ping::Ping(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3, double_t index)
    {
        frequency_ = index * sample_rate / fft_length;
    }

    Ping::~Ping() {

    }

    void Ping::calculateResults(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3)
    {

    }
}