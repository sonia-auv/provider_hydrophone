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

            auto ping = driver.getPing();

            if (ping != nullptr)
            {
                std::cout << "=========We have a ping !===========" << std::endl;
                std::cout << "Frequency : " << (int) ping->getFrequency() << "kHz" << std::endl;
                std::cout << "Amplitude : " << (int) ping->getAmplitude() << std::endl;
                std::cout << "Noise : " << (int) ping->getNoise() << std::endl;
                std::cout << "CRR : " << (int) ping->getChannelReferenceReal() << std::endl;
                std::cout << "CRI : " << (int) ping->getChannelReferenceImage() << std::endl;
                std::cout << "C1R : " << (int) ping->getChannel1Real() << std::endl;
                std::cout << "C1I : " << (int) ping->getChannel1Image() << std::endl;
                std::cout << "C2R : " << (int) ping->getChannel2Real() << std::endl;
                std::cout << "C2I : " << (int) ping->getChannel2Image() << std::endl;
                std::cout << "====================================" << std::endl;
            }
            else
            {
                std::cout << "!!!!!!!!!!!!We do not have a ping :( ! !!!!!!!!!!!" << std::endl;
            }

            //driver.test();
            std::cout << "End of while" << std::endl;
            r.sleep();
        }
    }

}  // namespace provider_hydrophone
