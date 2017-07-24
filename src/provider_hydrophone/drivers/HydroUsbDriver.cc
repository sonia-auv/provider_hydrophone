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

    HydroUsbDriver::HydroUsbDriver(Configuration &configuration)
        : configuration(configuration)
    {

        if (!connect(configuration.getTtyPort().c_str()))
            return;

        configurePortSetting();

    }

    HydroUsbDriver::~HydroUsbDriver() {

    }

    bool HydroUsbDriver::isConnected() {
        return tty != 0 && tty != -1;
    }

    bool HydroUsbDriver::connect(const char *deviceTty) {

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

        if (threshold > MAX_THRESHOLD_VALUE)
            threshold = MAX_THRESHOLD_VALUE;

        bool isAcquiringData = this->isAcquiringData();

        // If is acquiring data, stop
        if (isAcquiringData)
            stopAcquireData();

        writeData(SET_THRESHOLD_COMMAND);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        std::cout << "Setting threshold data return 1 : " << readData(200) << std::endl;
        //readData(200);

        // Give time to board to execute command
        usleep(WAITING_TIME);
        writeData(std::to_string(threshold) + ENTER_COMMAND_CHAR);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        std::cout << "Setting threshold data return 2 : " << readData(200) << std::endl;
        //readData(200);

        // If we were acquiring data before, restart
        if (isAcquiringData)
            startAcquireData();

    }

    void HydroUsbDriver::setGain(unsigned int gain) {

        if (gain > MAX_GAIN_VALUE)
            gain = MAX_GAIN_VALUE;

        bool isAcquiringData = this->isAcquiringData();

        // If is acquiring data, stop
        if (isAcquiringData)
            stopAcquireData();

        writeData(SET_GAIN_COMMAND);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        std::cout << "Setting gain data return 1 : " << readData(200) << std::endl;
        //readData(200);

        // Give time to board to execute command
        usleep(WAITING_TIME);
        writeData(std::to_string(gain) + ENTER_COMMAND_CHAR);

        // Give time to board to execute command
        usleep(WAITING_TIME);

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

        writeData(SET_NORMAL_MODE_COMMAND);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        std::cout << "Start acquire data return : " << readData(200) << std::endl;

        acquiringData = true;

    }

    void HydroUsbDriver::stopAcquireData() {

        if (!isAcquiringData())
            return;

        writeData(EXIT_COMMAND);

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
        std::regex expression(REGEX);

        bool searchFound = std::regex_search(line, matcher, expression);

        if (searchFound)
        {

            std::shared_ptr<Ping> ping(new Ping());

            ping->setFrequency(std::stoi(matcher[REGEX_FREQUENCY_ID]));
            ping->setAmplitude(std::stoi(matcher[REGEX_AMPLITUDE_ID]));
            ping->setNoise(std::stoi(matcher[REGEX_NOISE_ID]));

            ping->setChannelReferenceReal(std::stoi(matcher[REGEX_CHANNEL_REFERENCE_REAL_ID]));
            ping->setChannelReferenceImage(std::stoi(matcher[REGEX_CHANNEL_REFERENCE_IMAGE_ID]));

            ping->setChannel1Real(std::stoi(matcher[REGEX_CHANNEL_1_REAL_ID]));
            ping->setChannel1Image(std::stoi(matcher[REGEX_CHANNEL_1_IMAGE_ID]));

            ping->setChannel2Real(std::stoi(matcher[REGEX_CHANNEL_2_REAL_ID]));
            ping->setChannel2Image(std::stoi(matcher[REGEX_CHANNEL_2_IMAGE_ID]));

            return ping;

        }

        return std::shared_ptr<Ping>(nullptr);
    }

}