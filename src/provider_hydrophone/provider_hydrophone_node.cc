/**
 * \file	provider_hydrophone_node.cc
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

#include <fcntl.h>
#include <termios.h>
#include "provider_hydrophone/provider_hydrophone_node.h"

namespace provider_hydrophone {

    //==============================================================================
    // C / D T O R S   S E C T I O N

    //------------------------------------------------------------------------------
    //
    ProviderHydrophoneNode::ProviderHydrophoneNode(const ros::NodeHandlePtr &nh)
        : nh_(nh),
          driver("/dev/ttyUSB0")
    {

        pingDebugPub = nh_->advertise<provider_hydrophone::PingDebugMsg>("/provider_hydrophone/debug/ping", 100);
        pingPub = nh_->advertise<provider_hydrophone::PingMsg>("/provider_hydrophone/ping", 100);

        driver.setThreshold(2);
        driver.setGain(7);

    }

    //------------------------------------------------------------------------------
    //
    ProviderHydrophoneNode::~ProviderHydrophoneNode() {
        driver.closeConnection();
    }

    //==============================================================================
    // M E T H O D   S E C T I O N
    //------------------------------------------------------------------------------
    //
    void ProviderHydrophoneNode::Spin() {

        ros::Rate r(100);  // 100 hz

        driver.startAcquireData();

        while (ros::ok()) {
            ros::spinOnce();

            handlePing();

            r.sleep();
        }
    }

    void ProviderHydrophoneNode::handlePing() {

        auto ping = driver.getPing();
        if (ping != nullptr)
        {

            sendPingDebug(ping);

            sendPing(ping);

        }
        else
        {
            std::cout << "!!!!!!!!!!!!We do not have a ping :( ! !!!!!!!!!!!" << std::endl;
        }

    }

    void ProviderHydrophoneNode::sendPingDebug(std::shared_ptr<Ping> ping) {
        provider_hydrophone::PingDebugMsg pingMsg;

        pingMsg.frequency = ping->getFrequency();
        pingMsg.amplitude = ping->getAmplitude();
        pingMsg.noise = ping->getNoise();
        pingMsg.channelReferenceReal = ping->getChannelReferenceReal();
        pingMsg.channelReferenceImage = ping->getChannelReferenceImage();
        pingMsg.channel1Real = ping->getChannel1Real();
        pingMsg.channel1Image = ping->getChannel1Image();
        pingMsg.channel2Real = ping->getChannel2Real();
        pingMsg.channel2Image = ping->getChannel2Image();

        pingDebugPub.publish(pingMsg);
    }

    void ProviderHydrophoneNode::sendPing(std::shared_ptr<Ping> ping) {

        unsigned int soundSpeed = 1484; //TODO Const
        double a = 0.015;//TODO Const

        PingMsg pingMsg;

        double phaseRef = atan2(ping->getChannelReferenceImage(), ping->getChannelReferenceReal());
        double phase1 = atan2(ping->getChannel1Image(), ping->getChannel1Real());
        double phase2 = atan2(ping->getChannel2Image(), ping->getChannel2Real());

        double dephase1 = phase1 - phaseRef;
        double dephase2 = phase2 - phaseRef;

        double heading = atan2(dephase1, dephase2);

        unsigned int fullFrequency = ping->getFrequency() * 1000;

        double lambda = soundSpeed / fullFrequency;


        double t2 = (dephase2 / (2 * M_PI)) * lambda;

        double elevation = acos(t2/(cos(heading) * a));


        pingMsg.frequency = ping->getFrequency();
        pingMsg.heading = heading;
        pingMsg.elevation = elevation;

        pingPub.publish(pingMsg);

    }

}  // namespace provider_hydrophone
