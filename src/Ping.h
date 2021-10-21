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
        typedef Eigen::Matrix<double, 3, 3>Matrix3d;
        typedef Eigen::Matrix<double, 3, 1>Vector3d;

        Ping();
        ~Ping();

        void FillPing(double_t phaseRef, double_t phase1, double_t phase2, double_t phase3, double_t index); // For XC7S25 fpga
        void FillPing(double_t heading, double_t x, double_t y, double_t frequency); // For XC7S50 fpga
        void getResults(double_t *heading, double_t *elevation, double_t *frequency);
        bool isEmpty();

    private:

        void calculateElevation(double_t x, double_t y);

        double_t frequency_ = 0; // frequency in kHz
        double_t heading_ = 0; // heading in degrees
        double_t elevation_ = 0; // elevation in degres
        bool empty;

    //--------------------------------------------------------
    //-------------------------CONST--------------------------
    //--------------------------------------------------------

        double_t sample_rate = 256000.0;
        double_t fft_length = 256.0;
        double_t constant = 1500.0;

        Matrix3d hydrophone_position;
        Vector3d dephasage;
        Vector3d alpha;

    };
}


#endif //PROVIDER_HYDROPHONE_PING_H
