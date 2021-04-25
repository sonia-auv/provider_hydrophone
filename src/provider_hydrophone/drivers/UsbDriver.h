//
// Created by coumarc9 on 6/30/17.
//

#ifndef PROVIDER_HYDROPHONE_USBDRIVER_H
#define PROVIDER_HYDROPHONE_USBDRIVER_H

#include <ros/ros.h>

namespace provider_hydrophone
{
    class UsbDriver {

    public:

        UsbDriver(const char* deviceTty);
        ~UsbDriver();

        bool isConnected();
        void closeConnection();
        bool writeData(std::string data);
        std::string readData(unsigned int length);
        std::string readLine();

    private:

        bool connect(const char* deviceTty);
        void configurePortSetting();

        int tty = 0;
    };
}
#endif //PROVIDER_HYDROPHONE_USBDRIVER_H
