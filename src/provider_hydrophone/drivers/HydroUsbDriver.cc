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

    HydroUsbDriver::HydroUsbDriver(const char* deviceTty) {

        if (!connect(deviceTty))
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

        ROS_INFO("Configuring serial port settings");

        struct termios SerialPortSettings;

        tcgetattr(tty, &SerialPortSettings);

        cfsetispeed(&SerialPortSettings,B460800);
        cfsetospeed(&SerialPortSettings,B460800);

        cfmakeraw(&SerialPortSettings);
         SerialPortSettings.c_iflag |= IGNCR;//ICRNL;
        //SerialPortSettings.c_iflag &= ~ICANON;
        //SerialPortSettings.c_cc[VTIME] = 1
        tcsetattr(tty,TCSANOW,&SerialPortSettings);

        ROS_INFO("End configuring serial port settings");

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

        ROS_INFO("Setting a new threshold on the hydrophone board");

        if (threshold > MAX_THRESHOLD_VALUE)
            threshold = MAX_THRESHOLD_VALUE;

        bool isAcquiringData = this->isAcquiringData();

        // If is acquiring data, stop
        if (isAcquiringData)
        {
            ROS_INFO("We were acquiring data. Acquisition will stop for a moment");
            stopAcquireData();
        }


        writeData(SET_THRESHOLD_COMMAND);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        readData(200);

        // Give time to board to execute command
        usleep(WAITING_TIME);
        writeData(std::to_string(threshold) + ENTER_COMMAND_CHAR);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        readData(200);

        ROS_INFO_STREAM("Threshold has been setted : " << threshold);

        // If we were acquiring data before, restart
        if (isAcquiringData)
        {
            ROS_INFO("Threshold has been setted. Acquisition restart");
            startAcquireData();
        }


        ROS_INFO("End of setting a threshold on the hydrophone board");

    }

    void HydroUsbDriver::setGain(unsigned int gain) {

        ROS_INFO("Setting a new gain on the hydrophone board");

        if (gain > MAX_GAIN_VALUE)
            gain = MAX_GAIN_VALUE;

        bool isAcquiringData = this->isAcquiringData();

        // If is acquiring data, stop
        if (isAcquiringData)
        {
            ROS_INFO("We were acquiring data. Acquisition will stop for a moment");
            stopAcquireData();
        }

        writeData(SET_GAIN_COMMAND);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        readData(200);

        // Give time to board to execute command
        usleep(WAITING_TIME);
        writeData(std::to_string(gain) + ENTER_COMMAND_CHAR);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        readData(200);

        ROS_INFO_STREAM("Gain has been setted : " << gain);

        // If we were acquiring data before, restart
        if (isAcquiringData)
        {
            ROS_INFO("Gain has been setted. Acquisition restart");
            startAcquireData();
        }

        ROS_INFO("End of setting a gain on the hydrophone board");


    }

    bool HydroUsbDriver::writeData(std::string data) {

        ROS_DEBUG("Sending data to board");

        int  bytes_written  =  0 ;

        bytes_written = write(tty, data.c_str(), data.length());

        ROS_DEBUG("End sending data to board");

        return bytes_written != -1;
    }

    std::string HydroUsbDriver::readData(unsigned int length) {

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

    bool HydroUsbDriver::isAcquiringData() {
        return acquiringData;
    }

    void HydroUsbDriver::startAcquireData() {

        ROS_DEBUG("Start acquiring data");

        if (isAcquiringData())
            return;

        writeData(SET_NORMAL_MODE_COMMAND);

        // Give time to board to execute command
        usleep(WAITING_TIME);

        readData(200);
        readData(200);

        acquiringData = true;

    }

    void HydroUsbDriver::stopAcquireData() {

        ROS_DEBUG("Stop acquiring data");

        if (!isAcquiringData())
            return;

        writeData(EXIT_COMMAND);

        acquiringData = false;
    }

    std::string HydroUsbDriver::readLine() {

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

    std::shared_ptr<Ping> HydroUsbDriver::getPing() {

        ROS_DEBUG("Try to get a ping");

        auto line = readLine();

        std::smatch matcher;
        std::regex expression(REGEX);
	
	//if (!line.empty())
	//	std::cout << line << std::endl;

        bool searchFound = std::regex_search(line, matcher, expression);

        if (searchFound)
        {
            ROS_DEBUG("Ping found with the REGEX");

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

        } else {
            ROS_DEBUG("No ping found with the REGEX");
        }

        return std::shared_ptr<Ping>(nullptr);
    }

}
