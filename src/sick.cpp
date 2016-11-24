/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Dehmani Souheil, Marek Felix, Reckling Reno
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include<iostream>
#include "limits"
#include "ros/time.h"
#include "ros/duration.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "math.h"
#include "SickPLS.hh"
#include <string>

#define DEG_CIRCLE 360
#define DEG_TO_RAD (M_PI / (DEG_CIRCLE / 2))

int main(int argc, char** argv)
{
    //Init ROS Publisher.
    ros::init(argc, argv, "Sick3000");
    ros::NodeHandle n;
    std::string topic;
    n.getParam("topic", topic);
    ros::Publisher scanner_pub = n.advertise<sensor_msgs::LaserScan>(topic, 100);
    ros::Rate loop_rate(100);
    int baudrate;
    n.param("baudrate", baudrate, 500000);
    std::string serial;
    n.getParam("serial", serial);

    int init_attempts = 1;
    n.param("init_attempts", init_attempts, 1);

    //Create new SickPLS scanner class.
    SickToolbox::SickPLS *scanner;
    scanner = new SickToolbox::SickPLS(serial);

    //Init scanner with 500000baud.
    int init_counter = 0;
    while(init_counter < init_attempts){
        if(scanner -> Initialize(SickToolbox::SickPLS::IntToSickBaud(baudrate))){
            ROS_INFO("Init successful");
            break;
        }else{
            ROS_WARN("Init failed. Failed inits: %d, Start new.", init_counter+1);
            sleep(2);
            scanner = new SickToolbox::SickPLS(serial);
            sleep(1);
            init_counter++;
        }
    }

    //Count of measurements.
    int count = 0;
    //Min and max measured disctance
    double minrange = std::numeric_limits<double>::max(), maxrange = 0;
    //angel 180, resolution 0.5: 0.5 * 180= 360
    unsigned int num_measurement_values = 360;

    while( ros::ok() )
    {
        unsigned int scan_data[num_measurement_values];
        unsigned int *scan_data_pointer = scan_data;
        scanner -> GetSickScan(scan_data_pointer,num_measurement_values);
        sensor_msgs::LaserScan scan;
        scan.header.seq = count;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "base_laser";
        scan.angle_min = -90.0*DEG_TO_RAD;
        scan.angle_max = 90.0*DEG_TO_RAD;
        scan.scan_time = 0.1;
        for(int i=0; i<num_measurement_values; i++)
        {
            if(scan_data[i]>maxrange)
                maxrange = scan_data[i];
            if(scan_data[i]<minrange)
                minrange = scan_data[i];
            scan.ranges.push_back((double)(scan_data[i])/100.0);
        }
        //Scanner measures in cm -> convert to meter
        scan.range_min = minrange/100;
        scan.range_max = maxrange/100;
        //resolution
        scan.angle_increment = (0.5)*DEG_TO_RAD;

        scanner_pub.publish(scan);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
