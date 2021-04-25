/**
 * \file	provider_hydrophone_node.h
 * \author	Marc-Antoine Couture <coumarc9@outlook.com>
 * \date	06/25/2017
 *
 * \copyright Copyright (c) 2017 S.O.N.I.A. All rights reserved.
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

#ifndef PROVIDER_HYDROPHONE_PROVIDER_HYDROPHONE_NODE_H_
#define PROVIDER_HYDROPHONE_PROVIDER_HYDROPHONE_NODE_H_

#include <ros/node_handle.h>
#include <dynamic_reconfigure/server.h>
#include <provider_hydrophone/HydroConfig.h>

#include "drivers/UsbDriver.h"
#include <sonia_common/PingMsg.h>
#include "math.h"
#include "Configuration.h"
#include "Ping.h"

namespace provider_hydrophone {

    class ProviderHydrophoneNode {
    public:
        //==========================================================================
        // T Y P E D E F   A N D   E N U M

        //==========================================================================
        // P U B L I C   C / D T O R S

        explicit ProviderHydrophoneNode(const ros::NodeHandlePtr &nh);

        ~ProviderHydrophoneNode();

        /// Taking care of the spinning of the ROS thread.
        /// Each iteration of the loop, this will take the objects in the object
        /// registery, empty it and publish the objects.
        void Spin();
        void CallBackDynamicReconfigure(provider_hydrophone::HydroConfig &config, uint32_t level);

    private:
        ros::NodeHandlePtr nh_;
        Configuration configuration;
        UsbDriver driver;
        ros::Publisher pingPub;
        dynamic_reconfigure::Server<provider_hydrophone::HydroConfig> server;

        void handlePing();
        void sendPing(std::shared_ptr<Ping> ping);

        bool isAcquiringData();
        void startAcquireData();
        void stopAcquireData();
        void setGain(uint32_t gain);

        uint16_t gain_ = 0;
        uint16_t current_gain_ = 0;
        uint16_t soundSpeed = 0;
        double_t distanceBetweenHydrophone = 0.0;
        bool isAcquiring = false;

        //--------------------------------------------------------
        //-------------------------CONST--------------------------
        //--------------------------------------------------------

        const uint16_t MAX_GAIN_VALUE = 7;

        const std::string ENTER_COMMAND_CHAR = "\r";
        const std::string SET_NORMAL_MODE_COMMAND = "1" + ENTER_COMMAND_CHAR;
        const std::string SET_TEST_PING_MODE_COMMAND = "2" + ENTER_COMMAND_CHAR;
        const std::string SET_GAIN_COMMAND = "3" + ENTER_COMMAND_CHAR;
        const std::string SET_RAW_DATA_MODE_COMMAND = "4" + ENTER_COMMAND_CHAR;

        const std::string EXIT_COMMAND = "q";

        const uint16_t WAITING_TIME = 100000;

        const int8_t * REGEX = "";

        const uint8_t REGEX_PHASEREF_ID = 1;
        const uint8_t REGEX_PHASE1_ID = 2;
        const uint8_t REGEX_PHASE2_ID = 3;
        const uint8_t REGEX_PHASE3_ID = 4;
        const uint8_t REGEX_FREQUENCY_ID = 5;
    };

}  // namespace provider_hydrophone

#endif  // PROVIDER_HYDROPHONE_PROVIDER_HYDROPHONE_NODE_H_
