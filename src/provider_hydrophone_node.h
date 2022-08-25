/**
 * \file	provider_hydrophone_node.h
 * \author	Francis Alonzo
 * \date	2021/10/20
 *
 * \copyright Copyright (c) 2021 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROVIDER_HYDROPHONE_NODE_H_
#define PROVIDER_HYDROPHONE_NODE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <provider_hydrophone/HydroConfig.h>

#include <thread>
#include <condition_variable>
#include <math.h>
#include <string>
#include <vector>

#include <std_msgs/UInt32MultiArray.h>
#include <sonia_common/PingMsg.h>
#include <sonia_common/HydroSettings.h>
#include "Configuration.h"
#include "drivers/serial.h"

#define FIXED_POINT_FRACTIONAL_BITS 19
#define SIGNED_MASK 0x80000000
#define FIXED_POINT_DATA_MASK 0x3FFFFFFF
#define H1_REGISTER "H1"
#define H6_REGISTER "H6"

typedef enum {idle, normalop, test_ping, raw_data} operation_mode;

namespace provider_hydrophone {

    class ProviderHydrophoneNode {
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        //==========================================================================
        // P U B L I C   C / D T O R S

        ProviderHydrophoneNode(const ros::NodeHandlePtr &nh);
        ~ProviderHydrophoneNode();

        void Spin();
        void CallBackDynamicReconfigure(provider_hydrophone::HydroConfig &config, uint32_t level);

    private:
        
        ros::NodeHandlePtr nh_;
        dynamic_reconfigure::Server<provider_hydrophone::HydroConfig> server;
        Configuration configuration_;
        Serial serialConnection_;
        ros::Publisher pingPublisher_;
        ros::Publisher debugPublisher_;
        ros::Subscriber settingsSubcriber_;

        void readThread();
        void h1RegisterThread();
        void h6RegisterThread();

        bool stopReadThread = false;
        bool stoph1RegisterThread = false;
        bool stoph6RegisterThread = false;

        void changeSettings(const sonia_common::HydroSettings::ConstPtr& msg);

        bool ConfirmChecksum(std::string data);
        uint8_t CalculateChecksum(std::string data);

        void sendCmd(std::string cmd, std::vector<uint16_t> *argv);

        bool isAcquiring();
        void stopAcquireData();
        bool changeMode(operation_mode mode);

        bool setGain(uint8_t gain);

        bool createDOACommand(uint8_t snr, uint16_t lowth, uint16_t highth);
        void createDOACommand();
        bool setSNRThreshold(uint8_t threshold);
        bool setSignalLowThreshold(uint16_t threshold);
        bool setSignalHighThreshold(uint16_t threshold);

        bool createAGCCommand(uint8_t toggle, uint16_t signalth, uint16_t limitth);
        void createAGCCommand();
        bool setAGCToggle(uint8_t toggle);
        bool setSignalThreshold(uint16_t threshold);
        bool setLimitSignalThreshold(uint16_t threshold);

        float_t fixedToFloat(uint32_t data);

        uint8_t gain_ = 0;

        uint8_t snrThreshold_ = 0;
        uint16_t signalLowThreshold_ = 0;
        uint16_t signalHighThreshold_ = 0;

        uint8_t agcToggleMode_ = 0;
        uint16_t agcThreshold_ = 0;
        uint16_t agcMaxThreshold_ = 0;

        operation_mode operation_mode_ = idle;

        std::string active_register = "";

        std::thread readerThread;
        std::thread h1ParseThread;
        std::thread h6ParseThread;

        std::condition_variable h1_cond;
        std::string h1_string;
        std::mutex h1_mutex;
        
        std::condition_variable h6_cond;
        std::string h6_string;
        std::mutex h6_mutex;

        std::mutex dynamic_reconfigure_mutex;
        
        //--------------------------------------------------------
        //-------------------------CONST--------------------------
        //--------------------------------------------------------

        const std::string ENTER_COMMAND = "\n";
        const std::string OPERATION_CMD = "op";
        const std::string PGA_CMD = "pga";
        const std::string DOA_CMD = "doa";
        const std::string AGC_CMD = "agc";

        const std::string EXIT_COMMAND = "q";

        const float_t sample_rate = 256000.0;
        const float_t fft_length = 256.0;
        const float_t constant = 1500.0;
    };

}  // namespace provider_hydrophone

#endif  // PROVIDER_HYDROPHONE_NODE_H_
