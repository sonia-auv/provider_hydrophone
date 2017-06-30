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
        : nh_(nh)
    {

        int fd ;
        fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);

        if (fd == -1)
        {
            printf("\n  Error! in Opening ttyUSB0\n");
        }
        else
            printf("\n  ttyUSB0 Opened Successfully\n");

        struct termios SerialPortSettings;

        tcgetattr(fd, &SerialPortSettings);

        cfsetispeed(&SerialPortSettings,B460800);
        cfsetospeed(&SerialPortSettings,B460800);

        cfmakeraw(&SerialPortSettings);
        SerialPortSettings.c_iflag |= ICRNL;
        tcsetattr(fd,TCSANOW,&SerialPortSettings);

        std::string write_buffer = "4\r";

        int  bytes_written  =  0 ;

        std::cout << "Size : " << write_buffer.length() << std::endl;

        bytes_written = write(fd,write_buffer.c_str(),write_buffer.length());

        std::cout << bytes_written << " bytes wrote" << std::endl;

        char read_buffer[200];
        int  bytes_read = 0;

        bytes_read = read(fd,&read_buffer,sizeof(read_buffer));

        std::cout << bytes_read << " bytes readed" << std::endl;

        std::cout << "Readed buffer : " << std::string(read_buffer) << std::endl;

        std::string write_buffer2 = "2";

        bytes_written = write(fd,write_buffer2.c_str(),write_buffer2.length());

        std::cout << bytes_written << " bytes wrote" << std::endl;

        char read_buffer2[200];

        bytes_read = read(fd,&read_buffer2,sizeof(read_buffer2));

        std::cout << bytes_read << " bytes readed" << std::endl;

        std::cout << "Readed buffer2 : " << read_buffer2 << std::endl;

        close(fd);

        std::cout << "Connection closed" << std::endl;

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
