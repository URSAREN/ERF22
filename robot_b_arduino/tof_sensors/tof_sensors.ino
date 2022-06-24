
#include "Adafruit_VL53L0X.h"

#include <ros.h>
// #include <ros/time.h>
#include <sensor_msgs/Range.h>

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
int sensor1,sensor2;

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 6

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// Create range ROS messages and publishers
sensor_msgs::Range range_ir1;
sensor_msgs::Range range_ir2; 

ros::Publisher pub_range_ir1("/ir1", &range_ir1);
ros::Publisher pub_range_ir2("/ir2", &range_ir2);

uint8_t ir1Sensor;
uint8_t ir2Sensor;

ros::NodeHandle nh;

void read_tof_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  if(measure1.RangeStatus != 4) {     // if not out of range
    ir1Sensor = measure1.RangeMilliMeter;  
  }

  // print sensor two reading
  if(measure2.RangeStatus != 4) {
    ir2Sensor = measure2.RangeMilliMeter;
  }
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    while(1);
  }
}

// init tof message
void tof_sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::INFRARED;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 1.0;
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // sensor one reading
  if(measure1.RangeStatus != 4) {     // if not out of range
    sensor1 = measure1.RangeMilliMeter;    
  } else {
    // out of range
  }

  // sensor two reading
  if(measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
  } else {
    // out of range
  }
}

void setup() {

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  nh.initNode();
  tof_sensor_msg_init(range_ir1, "/ir1");
  tof_sensor_msg_init(range_ir2, "/ir2");

  setID();
 
}

void loop() {
   
  read_dual_sensors();
  range_ir1.header.stamp = nh.now();
  range_ir2.header.stamp = nh.now();

  pub_range_ir1.publish(&range_ir1);
  pub_range_ir2.publish(&range_ir2);
  
  delay(100);
}
