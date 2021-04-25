//
// Created by coumarc9 on 6/30/17.
//

#include <fcntl.h>
#include <cstdio>
#include <termios.h>
#include <iostream>
#include <unistd.h>
#include "UsbDriver.h"
#include <boost/algorithm/string.hpp>

namespace provider_hydrophone
{

    UsbDriver::UsbDriver(const char* deviceTty) {

        if (!connect(deviceTty))
            return;

        configurePortSetting();

    }

    UsbDriver::~UsbDriver() {

    }

    bool UsbDriver::isConnected() {
        return tty != 0 && tty != -1;
    }

    bool UsbDriver::connect(const char *deviceTty) {

        if (isConnected())
            closeConnection();

        tty = open(deviceTty, O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (tty == -1)
        {
            ROS_ERROR("An error occured while opening connection to %s", deviceTty);
        }
        else
        {
            ROS_INFO("Connection to %s opened", deviceTty);
        }

        return isConnected();

    }

    void UsbDriver::configurePortSetting() {

        ROS_DEBUG("Configuring serial port settings");

        struct termios SerialPortSettings;

        tcgetattr(tty, &SerialPortSettings);

        cfsetispeed(&SerialPortSettings,B460800);
        cfsetospeed(&SerialPortSettings,B460800);

        cfmakeraw(&SerialPortSettings);
        SerialPortSettings.c_iflag |= IGNCR;
        tcsetattr(tty,TCSANOW,&SerialPortSettings);

        ROS_DEBUG("End configuring serial port settings");

    }

    void UsbDriver::closeConnection() {

        if (!isConnected())
        {
            return;
        }

        close(tty);

        tty = 0;

        ROS_INFO_STREAM("Connection with device closed");
    }

    bool UsbDriver::writeData(std::string data) {

        ROS_DEBUG("Sending data to board");

        int  bytes_written  =  0 ;

        bytes_written = write(tty, data.c_str(), data.length());

        ROS_DEBUG("End sending data to board");

        return bytes_written != -1;
    }

    std::string UsbDriver::readData(unsigned int length) {

        ROS_DEBUG("Reading data to board");

        char read_buffer[length];
        int  bytes_read = 0;

        bytes_read = read(tty,&read_buffer, sizeof(read_buffer));

        ROS_DEBUG("End reading data to board");

        if (bytes_read != -1)
            return std::string(read_buffer);
        else
            return "";
    }

    std::string UsbDriver::readLine() {

        ROS_DEBUG("Reading a line on hydrophone board");

        std::string string;
        std::string lastChar;

        do{

            lastChar = readData(1);
	    if (!lastChar.empty()){
            	string += lastChar.at(0);
	    }
        } while (lastChar.length() != 0 && lastChar.at(0) != '\n');

        if (string.length() > 0)
            string.resize(string.length() - 1);

        ROS_DEBUG("End reading a line on hydrophone board");

        return string;
    }
}
