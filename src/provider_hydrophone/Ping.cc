//
// Created by coumarc9 on 7/1/17.
//

#include "Ping.h"

namespace provider_hydrophone
{

    Ping::Ping()
    {
        empty = true;
    }

    Ping::~Ping() 
    {
    }

    void Ping::FillPing(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3, double_t index)
    {
        empty = false;
        
        hydrophone_position<<   47.1404520791032,  -94.2809041582063,  47.1404520791032,
                                -47.1404520791032,  0,                  47.1404520791032,
                                1000.0,             1000.0,             1000.0;
        
        dephasage<<     phase1 - phaseRef,
                        phase2 - phaseRef,
                        phase3 - phaseRef;

        frequency_ = index * sample_rate / fft_length;

        alpha = hydrophone_position*dephasage;

        double_t x = alpha(0), y = alpha(1);

        heading_ = atan2(y,x);

        calculateElevation(x,y);
    }

    void Ping::FillPing(double_t heading, double_t x, double_t y, double_t frequency)
    {
        empty = false;

        frequency_ = frequency;
        heading_ = heading;

        calculateElevation(x,y);
    }

    void Ping::getResults(double_t *heading, double_t *elevation, double_t *frequency)
    {
        *heading = heading_;
        *elevation = elevation_;
        *frequency = frequency_;
    }

    bool Ping::isEmpty()
    {
        return empty;
    }

    void Ping::calculateElevation(double_t x, double_t y)
    {
        x = pow(x, 2.0);
        y = pow(y, 2.0);

        double_t frequency_2pi = 0, constant = 1500.0, t1 = 0.0, t2 = 0.0;

        frequency_2pi = frequency_ * 2 * M_PI;

        t1 = frequency_2pi / constant;
        t2 = pow(t1, 2.0);

        t2 = t2 - y - x;
        t2 = sqrt(t2);

        elevation_ = acos(t2/t1);
    }
}