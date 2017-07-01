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

        // TODO Check argument validation

        // TODO If aquisition mode, quit

        writeData("4\r");  // TODO Use constant
        usleep(10000); // TODO TEMP Give time to board to execute command
        std::cout << "Setting threshold data return 1 : " << readData(200) << std::endl;
        //readData(200);
        usleep(10000);// TODO TEMP Give time to board to execute command
        writeData(std::to_string(threshold));
        usleep(10000);// TODO TEMP Give time to board to execute command
        std::cout << "Setting threshold data return 2 : " << readData(200) << std::endl;
        //readData(200);

        // TODO If was in aquisition mode, restart

    }

    void HydroUsbDriver::setGain(unsigned int gain) {

        // TODO Check argument validation

        // TODO If aquisition mode, quit

        writeData("5\r");  // TODO Use constant
        usleep(10000);// TODO TEMP Give time to board to execute command
        std::cout << "Setting gain data return 1 : " << readData(200) << std::endl;
        //readData(200);
        usleep(10000);// TODO TEMP Give time to board to execute command
        writeData(std::to_string(gain));
        usleep(10000);// TODO TEMP Give time to board to execute command
        std::cout << "Setting gain data return 2 : " << readData(200) << std::endl;
        //readData(200);

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
            return "";
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
        std::smatch matcher;
        std::regex expression("(\\d*)kHz\\s*(\\d*)\\s*(\\d*)\\s*([-]?\\d*)\\/\\s*([-]?\\d*)\\s*([-]?\\d*)\\/\\s*([-]?\\d*)\\s*([-]?\\d*)\\/\\s*([-]?\\d*)");

        bool searchFound = std::regex_search(line, matcher, expression);

        if (searchFound)
        {
            std::cout << "Something Found !!" << std::endl;

            std::cout << "Match : " << matcher[0] << std::endl;
            std::cout << "Freq : " << matcher[1] << std::endl;
            std::cout << "Amplitude : " << matcher[2] << std::endl;
            std::cout << "Noise : " << matcher[3] << std::endl;
            std::cout << "CRR : " << matcher[4] << std::endl;
            std::cout << "CRI : " << matcher[5] << std::endl;
            std::cout << "C1R : " << matcher[6] << std::endl;
            std::cout << "C1I : " << matcher[7] << std::endl;
            std::cout << "C2R : " << matcher[8] << std::endl;
            std::cout << "C2I : " << matcher[9] << std::endl;
        }
        else
        {
            std::cout << "Nothing Found !! :(" << std::endl;
        }



        //for (auto toto : m)
            //std::cout << "Toto : " << toto << std::endl;

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

        } while (lastChar.length() != 0 && lastChar.at(0) != '\n');

        if (string.length() > 0)
            string.resize(string.length() - 1);

        return string;
    }

}