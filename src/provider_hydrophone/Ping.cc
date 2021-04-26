//
// Created by coumarc9 on 7/1/17.
//

#include "Ping.h"

namespace provider_hydrophone
{

    Ping::Ping(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3, double_t index)
    {
        hydrophone_position <<   47.1404520791032,  -94.2809041582063,  47.1404520791032
                                -47.1404520791032,  0,                  47.1404520791032
                                1000.0,             1000.0,             1000.0;
        
        dephasage <<    phase1 - phaseRef,
                        phase2 - phaseRef,
                        phase3 - phaseRef;

        frequency_ = index * sample_rate / fft_length;

        alpha = hydrophone_position*dephasage;

        double_t x = alpha(0), y = alpha(1);

        heading_ = atan2(y,x);

        calculateElevation(x,y);
    }

    Ping::~Ping() 
    {
    }

    void getResults(double_t *heading, double_t *elevation, double_t *frequency)
    {
        *heading = heading_;
        *elevation = elevation_;
        *frequency = frequency_;
    }

    void Ping::calculateElevation(double_t x, double_t y)
    {

    }
}