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
//int sr_led = 0;
//int led_R = 0;
//int led_G = 0;
//int led_B = 0;
float sr_led = 0;
float led_R = 0;
float led_G = 0;
float led_B = 0;
int lift = 0;
float x,z=0.0;

void pushdata()
{
    ros::Time begin = ros::Time::now();
    DynamicJsonBuffer jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    
    //Read from LLC
    JsonObject& cmd = root.createNestedObject("CMD!");
    cmd["X"] = x;
    cmd["Z"] = z;
//    cmd["L"] = lift;
    cmd["TID"] = begin.nsec;   //time sampling
    root["MOV?"] = 1;
    root["ENC?"] = 1;
    root["BAT?"] = 0;
    root["LIFTER"]= lift;
    root["MTR?"] = 1;
    root["STA?"] = 1;
    root["DI?"] = 1;
    root["DEBUG?"] = 1;

    //Write to LLC
    JsonArray& arr = root.createNestedArray("DO!");
    arr.add(sr_led);
    arr.add(led_R);
    arr.add(led_G);
    arr.add(led_B);


    string root2;
    root.printTo(root2);
    root2 = root2 + "\n";
//    cout <<"To EDEO:" << root2 << endl;
    cout <<"To TURTRONIK:" << root2 << endl;
    ser.write(root2);
    String res;
    res.data = root2;
    write_pub.publish(res);


}


void moveCallback(const geometry_msgs::Twist& msg){

    x = msg.linear.x;
    z = msg.angular.z;

}

//void lightCallback(const std_msgs::String::ConstPtr &msg)
void lightCallback(const std_msgs::ColorRGBA &msg)
{//color call back in RGB using SR,R,G,B Key
//    string sr_data = msg->data.c_str();
//    DynamicJsonBuffer jsonBuffer;
//    JsonObject& rd = jsonBuffer.parseObject(sr_data); 
//    sr_led = rd["SR"];
//    led_R = rd["R"];
//    led_G = rd["G"];
//    led_B = rd["B"]; 
    sr_led = msg.a;
    led_R = msg.r;
    led_G = msg.g;
    led_B = msg.b;   
//
}

void lifterCallback(const std_msgs::Int8::ConstPtr &msg)
{//color call back in RGB using SR,R,G,B Key
    lift = msg->data;
}



int main (int argc, char** argv){


    init(argc, argv, "low_level_handler");
    NodeHandle nh("~");

    Publisher read_pub = nh.advertise<String>("/low_level/status", 1000);
    write_pub = nh.advertise<String>("/low_level/write", 1000);
    Subscriber light = nh.subscribe("/led/cmd", 1000, lightCallback);
    Subscriber lifter= nh.subscribe("/lifter/cmd", 1000, lifterCallback);
    Subscriber move = nh.subscribe("/cmd_vel", 1, moveCallback); 
//    ros::Subscriber healthSubs = nh.subscribe("/read", 1000, agv_health_pub);



    try
    {
    	// /dev/edeozyro
//        ser.setPort("/dev/arduino_due");
        ser.setPort("/dev/turktronik");
        ser.setBaudrate(115200);
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

    Rate loop_rate(50);
    while(ok()){
	pushdata();
        while(ser.available()){
            // ROS_INFO_STREAM("Reading from serial port");
            String result;
            result.data = ser.readline();
//            cout <<"From EDEO:" << result.data << "---------------------------" << endl;
            cout <<"From TURTRONIK:" << result.data << "---------------------------" << endl;
            read_pub.publish(result);
        }
    	spinOnce();
        loop_rate.sleep();

    } 
    spin();
    return 0;
}


