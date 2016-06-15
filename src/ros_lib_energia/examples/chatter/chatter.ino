

/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include <Ethernet.h>
#include <ros.h>
#include <std_msgs/String.h>

byte mac[] = {  0x00, 0x1A, 0xB6, 0x02, 0xD3, 0x23 };
IPAddress server(192,168,1,135);

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  Serial.begin(115200);

  // Required for setting network details
  nh.getHardware()->setConnection(mac, server);
  nh.initNode();
  
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  
  nh.advertise(chatter);
}

void loop()
{
  if (nh.connected()) {
    str_msg.data = hello;
    chatter.publish( &str_msg );
  }
  nh.spinOnce();
  delay(1000);
}

