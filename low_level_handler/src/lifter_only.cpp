#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "ArduinoJson.h"
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace std_msgs;
using namespace ros;

serial::Serial ser;
Publisher write_pub;
int lift = 0;

void pushdata()
{
    string root3;
    stringstream ss;
    ss << lift;
    root3 = ss.str();
    cout <<"To NANO:" << root3 << endl;
    ser.write(root3);
    String res;
    res.data = root3;
    write_pub.publish(res);


}


void lifterCallback(const std_msgs::Int8::ConstPtr &msg){
    lift = msg->data;
}



int main (int argc, char** argv){


    init(argc, argv, "lifter_only");
    NodeHandle nh("~");

//    Publisher read_pub = nh.advertise<String>("/low_level/status", 1000);
    write_pub = nh.advertise<String>("write", 1000);
    Subscriber lifter= nh.subscribe("/lifter/cmd", 1000, lifterCallback);



    try
    {
    	// /dev/edeozyro
        ser.setPort("/dev/dfrduino_nano");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        //ROS_INFO_STREAM(to);
        ser.setTimeout(to);
        ser.open();

    }
    catch (serial::IOException& e)
    {
         cerr << e.what() << endl;
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    Rate loop_rate(10);
    while(ok()){
	pushdata();
//        while(ser.available()){
//            // ROS_INFO_STREAM("Reading from serial port");
//            String result;
//            result.data = ser.readline();
//            cout <<"From NANO:" << result.data << "---------------------------" << endl;
//            read_pub.publish(result);
//        }
    	spinOnce();
        loop_rate.sleep();

    } 
    spin();
    return 0;
}


