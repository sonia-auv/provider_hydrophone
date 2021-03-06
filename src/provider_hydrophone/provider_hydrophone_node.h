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

#include "drivers/HydroUsbDriver.h"
#include <sonia_common/PingDebugMsg.h>
#include <sonia_common/PingMsg.h>
#include "math.h"
#include "Configuration.h"

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

        dynamic_reconfigure::Server<provider_hydrophone::HydroConfig> server;

        void handlePing();

        void sendPingDebug(std::shared_ptr<Ping> ping);
        void sendPing(std::shared_ptr<Ping> ping);

        HydroUsbDriver driver;

        ros::Publisher pingDebugPub;
        ros::Publisher pingPub;

        unsigned int threshold_ = 0;
        unsigned int current_threshold_ = 0;
        unsigned int gain_ = 0;
        unsigned int current_gain_ = 0;

        unsigned int seq = 0;
        unsigned int seqDebug = 0;

        uint16_t soundSpeed;
        double_t distanceBetweenHydrophone;

    };

}  // namespace provider_hydrophone

#endif  // PROVIDER_HYDROPHONE_PROVIDER_HYDROPHONE_NODE_H_
