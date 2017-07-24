//
// Created by coumarc9 on 6/30/17.
//

#ifndef PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H
#define PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H

#include <ros/ros.h>
#include <regex>
#include <provider_hydrophone/Configuration.h>
#include "Ping.h"

namespace provider_hydrophone
{
    class HydroUsbDriver {

    public:

        HydroUsbDriver(Configuration &configuration);
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

        Configuration configuration;

        int tty = 0;
        bool acquiringData = false;

        bool connect(const char* deviceTty);
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

        const char * REGEX = "(\\d+)kHz\\s*(\\d+)\\s*(\\d+)\\s*([-]?\\d+)\\/\\s*([-]?\\d+)\\s*([-]?\\d+)\\/\\s*([-]?\\d+)\\s*([-]?\\d+)\\/\\s*([-]?\\d+)";

        const unsigned char REGEX_FREQUENCY_ID = 1;
        const unsigned char REGEX_AMPLITUDE_ID = 2;
        const unsigned char REGEX_NOISE_ID = 3;
        const unsigned char REGEX_CHANNEL_REFERENCE_REAL_ID = 4;
        const unsigned char REGEX_CHANNEL_REFERENCE_IMAGE_ID = 5;
        const unsigned char REGEX_CHANNEL_1_REAL_ID = 6;
        const unsigned char REGEX_CHANNEL_1_IMAGE_ID = 7;
        const unsigned char REGEX_CHANNEL_2_REAL_ID = 8;
        const unsigned char REGEX_CHANNEL_2_IMAGE_ID = 9;

    };
}
#endif //PROVIDER_HYDROPHONE_HYDROUSBDRIVER_H
