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

    void HydroUsbDriver::configurePortSetting() {
        struct termios SerialPortSettings;

        tcgetattr(tty, &SerialPortSettings);

        cfsetispeed(&SerialPortSettings,B460800);
        cfsetospeed(&SerialPortSettings,B460800);

        cfmakeraw(&SerialPortSettings);
         SerialPortSettings.c_iflag |= IGNCR;//ICRNL;
        //SerialPortSettings.c_iflag &= ~ICANON;
        //SerialPortSettings.c_cc[VTIME] = 1
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

        if (threshold > 9)
            threshold = 9;

        bool isAcquiringData = this->isAcquiringData();

        // If is acquiring data, stop
        if (isAcquiringData)
            stopAcquireData();

        writeData("4\r");  // TODO Use constant
        usleep(100000); // TODO TEMP Give time to board to execute command
        std::cout << "Setting threshold data return 1 : " << readData(200) << std::endl;
        //readData(200);
        usleep(100000);// TODO TEMP Give time to board to execute command
        writeData(std::to_string(threshold) + "\r");
        usleep(100000);// TODO TEMP Give time to board to execute command
        std::cout << "Setting threshold data return 2 : " << readData(200) << std::endl;
        //readData(200);

        // If we were acquiring data before, restart
        if (isAcquiringData)
            startAcquireData();

    }

    void HydroUsbDriver::setGain(unsigned int gain) {

        if (gain > 7)
            gain = 8;

        bool isAcquiringData = this->isAcquiringData();

        // If is acquiring data, stop
        if (isAcquiringData)
            stopAcquireData();

        writeData("5\r");  // TODO Use constant
        usleep(100000);// TODO TEMP Give time to board to execute command
        std::cout << "Setting gain data return 1 : " << readData(200) << std::endl;
        //readData(200);
        usleep(100000);// TODO TEMP Give time to board to execute command
        writeData(std::to_string(gain) + "\r");
        usleep(100000);// TODO TEMP Give time to board to execute command
        std::cout << "Setting gain data return 2 : " << readData(200) << std::endl;
        //readData(200);

        // If we were acquiring data before, restart
        if (isAcquiringData)
            startAcquireData();

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

        if (bytes_read != -1)
            return std::string(read_buffer);
        else
            return "";
    }

    bool HydroUsbDriver::isAcquiringData() {
        return acquiringData;
    }

    void HydroUsbDriver::startAcquireData() {

        if (isAcquiringData())
            return;

        writeData("3\r");// TODO Const

        usleep(100000);// TODO TEMP Give time to board to execute command

        std::cout << "Start acquire data return : " << readData(200) << std::endl;

        acquiringData = true;

    }

    void HydroUsbDriver::stopAcquireData() {

        if (!isAcquiringData())
            return;

        writeData("q");// TODO Const

        acquiringData = false;
    }

    std::string HydroUsbDriver::readLine() {

        std::string string;
        std::string lastChar;

        do{

            lastChar = readData(1);

            string += lastChar;

        } while (lastChar.length() != 0 && lastChar.at(0) != '\n');

        if (string.length() > 0)
            string.resize(string.length() - 1);

        return string;
    }

    std::shared_ptr<Ping> HydroUsbDriver::getPing() {

        auto line = readLine();

        std::cout << "Line : " << line << std::endl;
        std::smatch matcher;
        std::regex expression("(\\d+)kHz\\s*(\\d+)\\s*(\\d+)\\s*([-]?\\d+)\\/\\s*([-]?\\d+)\\s*([-]?\\d+)\\/\\s*([-]?\\d+)\\s*([-]?\\d+)\\/\\s*([-]?\\d+)");

        bool searchFound = std::regex_search(line, matcher, expression);

        if (searchFound)
        {

            std::shared_ptr<Ping> ping(new Ping());

            ping->setFrequency(std::stoi(matcher[1]));
            ping->setAmplitude(std::stoi(matcher[2]));
            ping->setNoise(std::stoi(matcher[3]));

            ping->setChannelReferenceReal(std::stoi(matcher[4]));
            ping->setChannelReferenceImage(std::stoi(matcher[5]));

            ping->setChannel1Real(std::stoi(matcher[6]));
            ping->setChannel1Image(std::stoi(matcher[7]));

            ping->setChannel2Real(std::stoi(matcher[8]));
            ping->setChannel2Image(std::stoi(matcher[9]));

            return ping;

        }

        return std::shared_ptr<Ping>(nullptr);
    }

}