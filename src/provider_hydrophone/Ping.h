//
// Created by coumarc9 on 7/1/17.
//

#ifndef PROVIDER_HYDROPHONE_PING_H
#define PROVIDER_HYDROPHONE_PING_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include "math.h"

namespace provider_hydrophone {

    class Ping {

    public:
        Ping(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3, double_t index);
        ~Ping();

        void getResults(double_t *heading, double_t *elevation, double_t *frequency);

    private:

        void calculateResults(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3);

        double_t frequency_; // frequency in kHz
        double_t heading_;
        double_t elevation_;

    //--------------------------------------------------------
    //-------------------------CONST--------------------------
    //--------------------------------------------------------

        const double_t sample_rate = 256000.0;
        const double_t fft_length = 256.0;

    };
}


#endif //PROVIDER_HYDROPHONE_PING_H
