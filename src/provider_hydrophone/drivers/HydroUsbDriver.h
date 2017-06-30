//
// Created by coumarc9 on 6/30/17.
//

#ifndef PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H
#define PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H

#include <ros/ros.h>

namespace provider_hydrophone
{
    class HydroUsbDriver {

    public:

        HydroUsbDriver(char* deviceTty);
        ~HydroUsbDriver();

        bool isConnected();

        void closeConnection();

        void setThreshold(unsigned int threshold);
        void setGain(unsigned int gain);

        bool isAcquiringData();
        void startAcquireData();
        void stopAcquireData();



    private:

        int tty = 0;
        bool acquiringData = false;

        bool connect(char* deviceTty);
        void configurePortSetting();

        bool writeData(std::string data);
        std::string readData(unsigned int length);

        //--------------------------------------------------------
        //-------------------------CONST--------------------------
        //--------------------------------------------------------
        const unsigned int MIN_THRESHOLD_VALUE = 0;
        const unsigned int MAX_THRESHOLD_VALUE = 3;

        const unsigned int MIN_GAIN_VALUE = 0;
        const unsigned int MAX_GAIN_VALUE = 7;


    };
}
#endif //PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H
