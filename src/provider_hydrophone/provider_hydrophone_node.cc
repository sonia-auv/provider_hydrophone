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
#include "provider_hydrophone/provider_hydrophone_node.h"

namespace provider_hydrophone {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
  ProviderHydrophoneNode::ProviderHydrophoneNode(const ros::NodeHandlePtr &nh)
      : nh_(nh),
        configuration(nh),
        driver(configuration.getTtyPort().c_str()),
        threshold_(configuration.getThreshold()),
        current_threshold_(configuration.getThreshold()),
        gain_(configuration.getGain()),
        current_gain_(configuration.getGain()),
        soundSpeed(configuration.getSoundSpeed()),
        distanceBetweenHydrophone(configuration.getDistanceBetweenHydrophone())
      {

    server.setCallback(boost::bind(&ProviderHydrophoneNode::CallBackDynamicReconfigure, this, _1, _2));

    HydroConfig config;
    config.Threshold = current_threshold_;
    config.Gain = current_gain_;
    CallBackDynamicReconfigure(config, 0);

    pingPub = nh_->advertise<sonia_common::PingMsg>("/provider_hydrophone/ping", 100);

    driver.setThreshold(current_threshold_);
    driver.setGain(current_gain_);
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
  void ProviderHydrophoneNode::Spin() 
  {
    ros::Rate r(100);  // 100 hz

    driver.startAcquireData();

    while (ros::ok()) {
      ros::spinOnce();

      if (threshold_ != current_threshold_) {
        current_threshold_ = threshold_;
        driver.setThreshold(current_threshold_);
      }

      if (gain_ != current_gain_) {
        current_gain_ = gain_;
        driver.setGain(current_gain_);
      }

      handlePing();

      r.sleep();
    }
  }

  void ProviderHydrophoneNode::CallBackDynamicReconfigure(provider_hydrophone::HydroConfig &config, uint32_t level)
    {
      ROS_INFO_STREAM("DynamicReconfigure callback. Old threshold : " << threshold_ << " new threshold : " << config.Threshold);
      ROS_INFO_STREAM("DynamicReconfigure callback. Old gain : " << gain_ << " new gain : " << config.Gain);
      threshold_ = config.Threshold;
      gain_ = config.Gain;
    }

  void ProviderHydrophoneNode::handlePing() 
  {
    auto ping = driver.getPing();

    while (ping != nullptr)
    {
        sendPing(ping);

        ping = driver.getPing();

    }
  }

  void ProviderHydrophoneNode::sendPing(std::shared_ptr<Ping> ping)
  {
      sonia_common::PingMsg pingMsg;

      pingMsg.header.stamp = ros::Time::now();

      int chanRefReal = ping->getChannelReferenceReal();
      int chanRefImage = ping->getChannelReferenceImage();
      int chan1Real = ping->getChannel1Real();
      int chan1Image = ping->getChannel1Image();
      int chan2Real = ping->getChannel2Real();
      int chan2Image = ping->getChannel2Image();

      double phaseRef = atan2(chanRefImage, chanRefReal);
      double phase1 = atan2(chan1Image, chan1Real);
      double phase2 = atan2(chan2Image, chan2Real);

      double dephase1 = phase1 - phaseRef;
      double dephase2 = phase2 - phaseRef;

      double heading = atan2(dephase1, dephase2);

      unsigned int fullFrequency = ping->getFrequency() * 1000;

      double lambda = (double) soundSpeed / fullFrequency;


      double t2 = (dephase2 / (2 * M_PI)) * lambda;

      double elevation = acos(t2/(cos(heading) * distanceBetweenHydrophone));

      pingMsg.raw_data.channelReferenceReal = chanRefReal;
      pingMsg.raw_data.channelReferenceImage = chanRefImage;
      pingMsg.raw_data.channel1Real = chan1Real;
      pingMsg.raw_data.channel1Image = chan1Image;
      pingMsg.raw_data.channel2Real = chan2Real;
      pingMsg.raw_data.channel2Image = chan2Image;
      pingMsg.frequency = ping->getFrequency();
      pingMsg.heading = heading;
      pingMsg.elevation = elevation;
      pingMsg.amplitude = ping->getAmplitude();
      pingMsg.noise = ping->getNoise();

      pingPub.publish(pingMsg);
  }

bool ProviderHydrophoneNode::isAcquiringData() {
    return acquiringData;
}

void ProviderHydrophoneNode::startAcquireData() {

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

void ProviderHydrophoneNode::stopAcquireData() {

    ROS_DEBUG("Stop acquiring data");

    if (!isAcquiringData())
        return;

    writeData(EXIT_COMMAND);

    acquiringData = false;
}

  void ProviderHydrophoneNode::setGain(unsigned int gain) {

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

  std::shared_ptr<Ping> ProviderHydrophoneNode::getPing() 
  {
    ROS_DEBUG("Try to get a ping");

    auto line = readLine();

    std::smatch matcher;
    std::regex expression(REGEX);

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
    } 
    else 
    {
        ROS_DEBUG("No ping found with the REGEX");
        return std::shared_ptr<Ping>(nullptr);
    }
  }

}  // namespace provider_hydrophone
