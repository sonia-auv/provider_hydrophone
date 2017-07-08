//
// Created by coumarc9 on 6/30/17.
//

#ifndef PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H
#define PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H

#include <ros/ros.h>
#include <regex>
#include "Ping.h"

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

        std::shared_ptr<Ping> getPing();

    private:

        int tty = 0;
        bool acquiringData = false;

        bool connect(char* deviceTty);
        void configurePortSetting();

        bool writeData(std::string data);
        std::string readData(unsigned int length);
        std::string readLine();

        //--------------------------------------------------------
        //-------------------------CONST--------------------------
        //--------------------------------------------------------
        const unsigned int MAX_THRESHOLD_VALUE = 9;
        const unsigned int MAX_GAIN_VALUE = 7;


        const std::string ENTER_COMMAND_CHAR = "\r";

        const std::string SET_NORMAL_MODE_COMMAND = "3" + ENTER_COMMAND_CHAR;
        const std::string SET_THRESHOLD_COMMAND = "4" + ENTER_COMMAND_CHAR;
        const std::string SET_GAIN_COMMAND = "5" + ENTER_COMMAND_CHAR;

        const std::string EXIT_COMMAND = "q";

        const unsigned int WAITING_TIME = 100000;


    };
}
#endif //PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H
