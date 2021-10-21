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
#include "provider_hydrophone_node.h"

namespace provider_hydrophone {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
  ProviderHydrophoneNode::ProviderHydrophoneNode(const ros::NodeHandlePtr &nh)
    : nh_(nh),
      configuration_(nh)
  {
    //server.setCallback(boost::bind(&ProviderHydrophoneNode::CallBackDynamicReconfigure, this, _1, _2));

    //HydroConfig config;
    //CallBackDynamicReconfigure(config, 0);

    pingPublisher_ = nh_->advertise<sonia_common::PingMsg>("/provider_hydrophone/ping", 100);

    //setGain(current_gain_);
  }

  //------------------------------------------------------------------------------
  //
  ProviderHydrophoneNode::~ProviderHydrophoneNode() {
      //driver.closeConnection();
  }

  //==============================================================================
  // M E T H O D   S E C T I O N
  //------------------------------------------------------------------------------
  //
  void ProviderHydrophoneNode::Spin() 
  {
    ros::Rate r(100);  // 100 hz

    //startAcquireData();

    while (ros::ok()) 
    {
      ros::spinOnce();

      /*if (gain_ != current_gain_) {
        //current_gain_ = gain_;
        //setGain(current_gain_);
      }*/

      //handlePing();

      r.sleep();
    }
  }

  /*void ProviderHydrophoneNode::CallBackDynamicReconfigure(provider_hydrophone::HydroConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("DynamicReconfigure callback. Old gain : " << gain_ << " new gain : " << config.Gain);
    gain_ = config.Gain;
  }

  void ProviderHydrophoneNode::handlePing() 
  {
    Ping ping;

    getPing(&ping);

    while (!(ping.isEmpty()))
    {
        sendPing(&ping);

        getPing(&ping);
    }
  }

  void ProviderHydrophoneNode::sendPing(Ping *ping)
  {
      sonia_common::PingMsg pingMsg;
      double_t heading, elevation, frequency;

      pingMsg.header.stamp = ros::Time::now();

      ping->getResults(&heading, &elevation, &frequency);

      pingPub.publish(pingMsg);
  }

  bool ProviderHydrophoneNode::isAcquiringData() 
  {
    return acquiringData;
  }

  void ProviderHydrophoneNode::startAcquireData() 
  {
    ROS_DEBUG("Start acquiring data");

    if (!isAcquiringData()) return;

    driver.writeData(SET_NORMAL_MODE_COMMAND);

    // Give time to board to execute command
    usleep(WAITING_TIME);
    /* TO TEST WITHOUT
    driver.readData(200);
    driver.readData(200);*/

    /*acquiringData = true;
  }

  void ProviderHydrophoneNode::stopAcquireData() {

    ROS_DEBUG("Stop acquiring data");

    if (!isAcquiringData()) return;

    driver.writeData(EXIT_COMMAND);

    acquiringData = false;
  }

  void ProviderHydrophoneNode::setGain(uint32_t gain) {

    ROS_DEBUG("Setting a new gain on the hydrophone board");

    if (gain > MAX_GAIN_VALUE)
    {
      gain = MAX_GAIN_VALUE;
    }
    
    // If is acquiring data, stop
    if (isAcquiringData())
    {
      ROS_DEBUG("We were acquiring data. Acquisition will stop for a moment");
      stopAcquireData();
    }

    driver.writeData(SET_GAIN_COMMAND);

    // Give time to board to execute command
    usleep(WAITING_TIME);

    driver.readData(200);

    // Give time to board to execute command
    usleep(WAITING_TIME);
    driver.writeData(std::to_string(gain) + ENTER_COMMAND_CHAR);

    // Give time to board to execute command
    usleep(WAITING_TIME);

    /*TO TEST WITHOUT
    driver.readData(200);*/

    /*ROS_INFO_STREAM("Gain has been setted : " << gain);

    // If we were acquiring data before, restart
    if (isAcquiringData())
    {
      ROS_DEBUG("Gain has been setted. Acquisition restart");
      startAcquireData();
    }

    ROS_DEBUG("End of setting a gain on the hydrophone board");
  }*/

}  // namespace provider_hydrophone
