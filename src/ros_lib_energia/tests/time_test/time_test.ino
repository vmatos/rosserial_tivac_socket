/* 
 * rosserial::std_msgs::Time Test
 * Publishes current time
 */
#include <Ethernet.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>

byte mac[] = {  0x00, 0x1A, 0xB6, 0x02, 0xD3, 0x23 };
IPAddress server(192,168,1,135);

ros::NodeHandle  nh;

std_msgs::Time test;
ros::Publisher p("my_topic", &test);

void setup()
{
  pinMode(RED_LED, OUTPUT);
  nh.getHardware()->setConnection(mac, server);
  nh.initNode();
  nh.advertise(p);
}

void loop()
{  
  test.data = nh.now();
  p.publish( &test );
  nh.spinOnce();
  delay(10);
}

