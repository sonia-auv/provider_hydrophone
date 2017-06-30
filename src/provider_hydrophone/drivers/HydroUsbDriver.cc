//
// Created by coumarc9 on 6/30/17.
//

#include <fcntl.h>
#include <cstdio>
#include <termios.h>
#include <iostream>
#include <unistd.h>
#include "HydroUsbDriver.h"

namespace provider_hydrophone
{

    HydroUsbDriver::HydroUsbDriver(char* deviceTty) {

        if (!connect(deviceTty))
            return;

        configurePortSetting();


        writeData("4\r");

        readData(200);

        writeData("2");

        readData(200);

        // Setting gain
//        writeData("5\r");
//
//        readData(200);
//
//        writeData("7");
//
//        readData(200);

        closeConnection();

    }

    HydroUsbDriver::~HydroUsbDriver() {

    }

    bool HydroUsbDriver::isConnected() {
        return tty != 0 && tty != -1;
    }

    bool HydroUsbDriver::connect(char *deviceTty) {

        if (isConnected())
            closeConnection();

        tty = open(deviceTty,O_RDWR | O_NOCTTY);

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

    void HydroUsbDriver::configurePortSetting() {
        struct termios SerialPortSettings;

        tcgetattr(tty, &SerialPortSettings);

        cfsetispeed(&SerialPortSettings,B460800);
        cfsetospeed(&SerialPortSettings,B460800);

        cfmakeraw(&SerialPortSettings);
        SerialPortSettings.c_iflag |= ICRNL;
        tcsetattr(tty,TCSANOW,&SerialPortSettings);
    }

    void HydroUsbDriver::closeConnection() {

        if (!isConnected())
            return;

        close(tty);

        tty = 0;

        ROS_INFO("Connection with device closed");

    }

    void HydroUsbDriver::setThreshold(unsigned int threshold) {

        // TODO If aquisition mode, quit

        // set threshold

        // TODO If was in aquisition mode, restart

    }

    void HydroUsbDriver::setGain(unsigned int gain) {
// TODO If aquisition mode, quit

        // Set gain

        // TODO If was in aquisition mode, restart
    }

    bool HydroUsbDriver::writeData(std::string data) {

        int  bytes_written  =  0 ;

        std::cout << "Size : " << data.length() << std::endl;

        bytes_written = write(tty, data.c_str(), data.length());

        std::cout << bytes_written << " bytes wrote" << std::endl;

        return bytes_written != -1;
    }

    std::string HydroUsbDriver::readData(unsigned int length) {


        char read_buffer[length];
        int  bytes_read = 0;

        bytes_read = read(tty,&read_buffer,sizeof(read_buffer));

        std::cout << bytes_read << " bytes readed" << std::endl;

        std::cout << "Readed buffer : " << std::string(read_buffer) << std::endl;

        if (bytes_read != -1)
            return std::string(read_buffer);
        else
            return NULL;
    }

}