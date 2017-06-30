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

    }

    //------------------------------------------------------------------------------
    //
    ProviderHydrophoneNode::~ProviderHydrophoneNode() {}

    //==============================================================================
    // M E T H O D   S E C T I O N
    //------------------------------------------------------------------------------
    //
    void ProviderHydrophoneNode::Spin() {

      ros::Rate r(0.5);  // 15 hz
      while (ros::ok()) {
        ros::spinOnce();

       r.sleep();
      }
    }

}  // namespace provider_hydrophone
