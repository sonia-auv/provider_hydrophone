#!/usr/bin/env python

import rospy
import os
from provider_hydrophone.msg import PingMsg


class PingMsgToCsv:

    def __init__(self):

        home = os.path.expanduser("~")

        self.csv_file = open(home + "/parse.csv", "w")

        self.write_header()

        rospy.init_node("provider_hydrophone_ping_csv")

        self.subscriber = rospy.Subscriber("/provider_hydrophone/ping", PingMsg, self.callback)

        self.loop()

    def loop(self):

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            rate.sleep()

        self.csv_file.close()

    def write_header(self):
        self.write_line("seq;stamp;frequency;amplitude;noise;heading;elevation;raw_data_chanRefReal;raw_data_chanRefImage;raw_data_chan1Real;raw_data_chan1Image;raw_data_chan2Real;raw_data_chan2Image")

    def write_line(self, line):
        self.csv_file.write(line + "\n")


    def callback(self, object):

        seq = object.header.seq
        stamp = object.header.stamp.secs
        frequency = object.frequency
        amplitude = object.amplitude
        noise = object.noise
        heading = object.heading
        elevation = object.elevation
        chan_ref_real = object.raw_data.channelReferenceReal
        chan_ref_image = object.raw_data.channelReferenceImage
        chan_1_real = object.raw_data.channel1Real
        chan_1_image = object.raw_data.channel1Image
        chan_2_real = object.raw_data.channel2Real
        chan_2_image = object.raw_data.channel2Image

        elements = [seq,stamp,frequency,amplitude, noise, heading, elevation, chan_ref_real, chan_ref_image, chan_1_real, chan_1_image, chan_2_real, chan_2_image]

        line = ';'.join(map(str, elements))

        self.write_line(line)



if __name__ == "__main__":
    element = PingMsgToCsv()