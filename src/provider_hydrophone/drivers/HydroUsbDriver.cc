//
// Created by coumarc9 on 6/30/17.
//

#include <fcntl.h>
#include <cstdio>
#include <termios.h>
#include <iostream>
#include <unistd.h>
#include "HydroUsbDriver.h"
#include <boost/algorithm/string.hpp>

namespace provider_hydrophone
{

    HydroUsbDriver::HydroUsbDriver(char* deviceTty) {

        if (!connect(deviceTty))
            return;

        configurePortSetting();

        // Be sure

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
        SerialPortSettings.c_iflag |= IGNCR;//ICRNL;
        tcsetattr(tty,TCSANOW,&SerialPortSettings);
    }

    void HydroUsbDriver::closeConnection() {

        if (!isConnected())
            return;

        stopAcquireData();

        close(tty);

        tty = 0;

        ROS_INFO("Connection with device closed");

    }

    void HydroUsbDriver::setThreshold(unsigned int threshold) {

        // TODO Check argument validation

        // TODO If aquisition mode, quit

        writeData("4\r");  // TODO Use constant

        readData(200);

        writeData(std::to_string(threshold));

        readData(200);

        // TODO If was in aquisition mode, restart

    }

    void HydroUsbDriver::setGain(unsigned int gain) {

        // TODO Check argument validation

        // TODO If aquisition mode, quit

        writeData("5\r");  // TODO Use constant

        readData(200);

        writeData(std::to_string(gain));

        readData(200);

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

        //std::cout << bytes_read << " bytes readed" << std::endl;

        //std::cout << "Readed buffer : " << std::string(read_buffer) << std::endl;

        if (bytes_read != -1)
            return std::string(read_buffer);
        else
            return NULL;
    }

    bool HydroUsbDriver::isAcquiringData() {
        return acquiringData;
    }

    void HydroUsbDriver::startAcquireData() {

        if (isAcquiringData())
            return;

        writeData("3\r");// TODO Const

        acquiringData = true;

    }

    void HydroUsbDriver::stopAcquireData() {

        if (!isAcquiringData())
            return;

        writeData("q");// TODO Const

        //std::cout<< "Size : " << readData(2000).size(); // TODO While no data?

        //writeData("s");// TODO Const

        acquiringData = false;
    }

    void HydroUsbDriver::test() {


        auto line = readLine();

        std::cout << "Line : " << line << std::endl;

//        auto data = readData(150);
//
//        std::vector<std::string> lines;
//
//        boost::split(lines, data, boost::is_any_of("\n\n"));
//
//        for (auto line : lines)
//        {
//            std::cout << "Line : " << line << std::endl;
//        }

        //std::cout << data << std::endl;

    }

    std::string HydroUsbDriver::readLine() {

        std::string string;
        std::string lastChar;

        //bool previousWasNewLine = false;

        do{

            //if (lastChar.at(0) == '\n')
                //previousWasNewLine = true;
            //else


            lastChar = readData(1);

            string += lastChar;

        } while (lastChar.at(0) != '\n');

        string.resize(string.length() - 1);

        return string;
    }

}