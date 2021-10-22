//
// Created by coumarc9 on 7/1/17.
//

#include "Ping.h"

namespace provider_hydrophone
{

    Ping::Ping()
    {
    }

    Ping::~Ping() 
    {
    }

    float_t Ping::calculateElevation(float_t x, float_t y, float_t frequency)
    {
        x = pow(x, 2.0);
        y = pow(y, 2.0);

        float_t frequency_2pi = 0.0, t1 = 0.0, t2 = 0.0;

        frequency_2pi = frequency * 2 * M_PI;

        t1 = frequency_2pi / constant;
        t2 = pow(t1, 2.0);

        t2 = t2 - y - x;
        t2 = sqrt(t2);
 
        return acos(t2/t1);
    }

    float_t Ping::calculateHeading(float_t x, float_t y)
    {
        return atan2(y,x);
    }
}