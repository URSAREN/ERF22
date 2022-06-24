#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "Adafruit_VL53L0X.h"

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

/*
    Reset all ToF-sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
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

#define SONAR_NUM 2          //The number of sensors. 
#define MAX_DISTANCE 200     //Mad distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.
 
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
 
unsigned long _timerStart = 0;
 
int LOOPING = 40; //Loop for every 40 milliseconds.
 
uint8_t oldSensorReading[SONAR_NUM];    //Store last valid value of the sensors.
 
uint8_t leftSensor;             //Store raw sensor's value.
uint8_t centerSensor;
 
uint8_t leftSensorKalman;       //Store filtered sensor's value.
uint8_t centerSensorKalman;
 
 
NewPing sonar[SONAR_NUM] = {
  NewPing(2,3, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE)
};
 
/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
SimpleKalmanFilter KF_Left(2, 2, 0.01);
SimpleKalmanFilter KF_Center(2, 2, 0.01);

void tof_sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::INFRARED;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 1.0;
}

//Create instances for range messages.
sensor_msgs::Range range_ir1;
sensor_msgs::Range range_ir2; 
//Create publisher objects for all sensors
ros::Publisher pub_range_ir1("/ir1", &range_ir1);
ros::Publisher pub_range_ir2("/ir2", &range_ir2);

uint8_t ir1Sensor;             //Store raw sensor's value.
uint8_t ir2Sensor;

void read_tof_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  if(measure1.RangeStatus != 4) {     // if not out of range
    ir1Sensor = measure1.RangeMilliMeter;  
  }

  // print sensor two reading
  Serial.print("2: ");
  if(measure2.RangeStatus != 4) {
    ir2Sensor = measure2.RangeMilliMeter;
  }
}

ros::NodeHandle nh; //create an object which represents the ROS node.
 
//looping the sensors (ultraachall)
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}
 
// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
 
//Return the last valid value from the sensor.
void oneSensorCycle() {
  leftSensor   = returnLastValidRead(0, cm[0]);
  centerSensor = returnLastValidRead(1, cm[1]);
}
 
//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}
 
//Apply Kalman Filter to sensor reading.
void applyKF() {
  leftSensorKalman   = KF_Left.updateEstimate(leftSensor);
  centerSensorKalman = KF_Center.updateEstimate(centerSensor);
}
 
void startTimer() {
  _timerStart = millis();
}
 
bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}
 
void us_sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.0;
}
 
//Create instances for range messages.
sensor_msgs::Range range_left;
sensor_msgs::Range range_center;
 
//Create publisher onjects for all sensors
ros::Publisher pub_range_left("/ultrasound_left", &range_left);
ros::Publisher pub_range_center("/ultrasound_center", &range_center);

void setup() {
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  setID();

 pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
 
  nh.initNode();
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_center);
  nh.advertise(pub_range_ir1);
  nh.advertise(pub_range_ir2);
 
  us_sensor_msg_init(range_left, "/ultrasound_left");
  us_sensor_msg_init(range_center, "/ultrasound_center");
  tof_sensor_msg_init(range_ir1, "/ir1");
  tof_sensor_msg_init(range_ir2, "/ir2");
}

void loop() {
  if (isTimeForLoop(LOOPING)) {
    read_tof_sensors();

    sensorCycle();
    oneSensorCycle();
    applyKF();
    range_left.range   = leftSensorKalman;
    range_center.range = centerSensorKalman;
 
    range_left.header.stamp = nh.now();
    range_center.header.stamp = nh.now();
    range_ir1.header.stamp = nh.now();
    range_ir2.header.stamp = nh.now();

    pub_range_left.publish(&range_left);
    pub_range_center.publish(&range_center);
    pub_range_ir1.publish(&range_ir1);
    pub_range_ir2.publish(&range_ir2);

    startTimer();
  }
  nh.spinOnce();//Handle ROS events
}
