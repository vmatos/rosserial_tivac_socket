/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */
#include <Ethernet.h>
#include <ros.h>
#include <std_msgs/Float64.h>

byte mac[] = {  0x00, 0x1A, 0xB6, 0x02, 0xD3, 0x23 };
IPAddress server(192,168,1,135);

ros::NodeHandle nh;

float x; 

void messageCb( const std_msgs::Float64& msg){
  x = msg.data - 1.0;
  digitalWrite(RED_LED, HIGH-digitalRead(RED_LED));   // blink the led
}

std_msgs::Float64 test;
ros::Subscriber<std_msgs::Float64> s("your_topic", &messageCb);
ros::Publisher p("my_topic", &test);

void setup()
{
  pinMode(RED_LED, OUTPUT);
  nh.getHardware()->setConnection(mac, server);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  test.data = x;
  p.publish( &test );
  nh.spinOnce();
  delay(10);
}

