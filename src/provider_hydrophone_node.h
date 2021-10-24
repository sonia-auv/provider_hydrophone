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
#include <thread>
#include <condition_variable>

#include <sonia_common/PingMsg.h>
#include "Configuration.h"
#include "Ping.h"
#include "drivers/serial.h"

#define FIXED_POINT_FRACTIONAL_BITS 19

namespace provider_hydrophone {

    class ProviderHydrophoneNode {
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        //==========================================================================
        // P U B L I C   C / D T O R S

        explicit ProviderHydrophoneNode(const ros::NodeHandlePtr &nh);

        ~ProviderHydrophoneNode();

        void Spin();

    private:
        
        ros::NodeHandlePtr nh_;
        Configuration configuration_;
        Serial serialConnection_;
        ros::Publisher pingPublisher_;

        void readThread();
        void h1RegisterThread();

        bool isAcquiringData();
        void startAcquireNormalMode();
        void stopAcquireData();
        float_t fixedToFloat(uint32_t data);

        uint8_t gain_ = 0;
        uint8_t current_gain_ = 0;
        uint16_t snrThreshold_ = 0;
        uint16_t signalThreshold_ = 0;
        bool acquiringNormalData_ = false;

        std::thread readerThread;
        std::thread h1ParseThread;

        std::condition_variable h1_cond;
        std::string h1_string;
        std::mutex h1_mutex;
        
        std::condition_variable h6_cond;
        std::string h6_string;
        std::mutex h6_mutex;
        
        //--------------------------------------------------------
        //-------------------------CONST--------------------------
        //--------------------------------------------------------

        const char* H1_REGISTER = "H1";
        const char* H6_REGISTER = "H6";

        const std::string ENTER_COMMAND_CHAR = "\n";
        const std::string SET_NORMAL_MODE_COMMAND = "1" + ENTER_COMMAND_CHAR;
        const std::string SET_TEST_PING_MODE_COMMAND = "2" + ENTER_COMMAND_CHAR;
        const std::string SET_GAIN_COMMAND = "3" + ENTER_COMMAND_CHAR;
        const std::string SET_RAW_DATA_MODE_COMMAND = "4" + ENTER_COMMAND_CHAR;

        const std::string EXIT_COMMAND = "q";
    };

}  // namespace provider_hydrophone

#endif  // PROVIDER_HYDROPHONE_NODE_H_
