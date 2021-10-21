//
// Created by coumarc9 on 7/1/17.
//

#ifndef PROVIDER_HYDROPHONE_PING_H
#define PROVIDER_HYDROPHONE_PING_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include "math.h"

namespace provider_hydrophone {

    class Ping {

    public:

        Ping();
        ~Ping();

    private:

        float_t calculateElevation(float_t x, float_t y, float_t frequency);
        float_t calculateHeading(float_t x, float_t y);
        
    //--------------------------------------------------------
    //-------------------------CONST--------------------------
    //--------------------------------------------------------

        float_t sample_rate = 256000.0;
        float_t fft_length = 256.0;
        float_t constant = 1500.0;
    };
}


#endif //PROVIDER_HYDROPHONE_PING_H
