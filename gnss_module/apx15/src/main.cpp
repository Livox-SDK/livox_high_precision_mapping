#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include "serial/serial.h"
#include "comms.h"

//#include <limits>

class apx15RosDriver
{
  public: 
  apx15RosDriver(std::string port, int32_t baud, ros::NodeHandle* gnss_inertial_nh);
  ~apx15RosDriver();

  private: 
  ros::Publisher imu_pub;
  ros::Publisher gnss_pub;

  void publishMsgs(apx15::apx15Gsof& myApx15Gsof, ros::NodeHandle* gnss_inertial_nh);
  
  uint32_t utc_offset_second;
  void recordOffsetTime(apx15::apx15Gsof& myApx15Gsof);

};


apx15RosDriver::apx15RosDriver(std::string port, int32_t baud, ros::NodeHandle* gnss_inertial_nh)
{
  imu_pub  = gnss_inertial_nh->advertise<sensor_msgs::Imu>("imu", 1, false);
  gnss_pub = gnss_inertial_nh->advertise<sensor_msgs::NavSatFix>("navsatfix", 1, false);

  serial::Serial mySerial;
  mySerial.setPort(port);
  mySerial.setBaudrate(baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  mySerial.setTimeout(to);

  bool first_failure = true;
  while (ros::ok())
  {
    try
    {
      mySerial.open();
    }
    catch(const serial::IOException& e)
    {
      ROS_DEBUG("Unable to connect to port.");
    }

    if (mySerial.isOpen())
    {
      ROS_INFO("Successfully connected to serial port.");
      first_failure = true;
      try
      {
        apx15::Comms commApx15(&mySerial);
        apx15::apx15Gsof myApx15;

        while (ros::ok())
        {
          if (commApx15.receive(&myApx15) == 1)
          {
            recordOffsetTime(myApx15);
            publishMsgs(myApx15, gnss_inertial_nh);
            ros::spinOnce();
          }
        }
      }
      catch(const std::exception& e)
      {
        if (mySerial.isOpen()) mySerial.close();
        ROS_ERROR_STREAM(e.what());
        ROS_INFO("trying reconnection after error.");
        ros::Duration(1.0).sleep();
      }
    }
    else
    {
      ROS_WARN_STREAM_COND(first_failure, "Could not connect to serial device "
                          << port << ". Trying again every 1 second.");
      first_failure = false;
      ros::Duration(1.0).sleep();
    }

  } // end of while (ros::ok())


}


apx15RosDriver::~apx15RosDriver()
{

}


#define APX_TIME_DEBUG 1


void apx15RosDriver::recordOffsetTime(apx15::apx15Gsof& myApx15Gsof)
{
  static int calcOffsetFlag = 0;

  if (calcOffsetFlag == 0)
  {
    int32_t actualSec = myApx15Gsof.gpsTimeInMilliSec/1000 - 18; //myApx15Gsof.leapSecond;
    uint64_t actualNsec = (myApx15Gsof.gpsTimeInMilliSec - actualSec*1000)*1000000;

    time_t pcTime = time(NULL);
    tm *pcGMTime = gmtime(&pcTime);
    if (APX_TIME_DEBUG)
    {
      printf("  pc: y:%d m:%d d:%d wd:%d h:%d m:%d s:%d\n", 
            pcGMTime->tm_year, pcGMTime->tm_mon, pcGMTime->tm_mday,
            pcGMTime->tm_wday, pcGMTime->tm_hour, pcGMTime->tm_min, pcGMTime->tm_sec);
    }

    time_t tTime = pcTime - actualSec + 40;
    tm *tGMTime = gmtime(&tTime);
    if (APX_TIME_DEBUG)
    {
      printf("temp: y:%d m:%d d:%d wd:%d h:%d m:%d s:%d\n", 
            tGMTime->tm_year, tGMTime->tm_mon, tGMTime->tm_mday,
            tGMTime->tm_wday, tGMTime->tm_hour, tGMTime->tm_min, tGMTime->tm_sec);
    }

    if ((tGMTime->tm_wday == 0) && 
        (tGMTime->tm_hour == 0) && 
        (tGMTime->tm_min == 0))
    {
      tGMTime->tm_sec = 0;
      utc_offset_second = timegm(tGMTime);

      calcOffsetFlag = 1;

      printf("****** offset convert success! ******\n");
    }
    else
    {
      printf("\033[1m\033[31m time offset convert error!\033[0m\n");
      utc_offset_second = 0;
    }

    if (APX_TIME_DEBUG)
    {
      printf("week offset second:%d\n", utc_offset_second);
    }
  }

}


void apx15RosDriver::publishMsgs(apx15::apx15Gsof& myApx15Gsof, ros::NodeHandle* gnss_inertial_nh)
{
  sensor_msgs::Imu imu_msg;
  sensor_msgs::NavSatFix navfix_msg;
  
  //leap second is 18
  int32_t weekSec = myApx15Gsof.gpsTimeInMilliSec/1000;
  int32_t utcSec = utc_offset_second + weekSec - 18;
  uint64_t utcNsec = (myApx15Gsof.gpsTimeInMilliSec%1000)*1000000;

  imu_msg.header.frame_id = "apx";
  imu_msg.header.stamp.sec  = utcSec;
  imu_msg.header.stamp.nsec = utcNsec;


  if (imu_pub.getNumSubscribers() > 0)
  {
    imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(myApx15Gsof.roll*DEGREE2PI,
                                                                  myApx15Gsof.pitch*DEGREE2PI,
                                                                  myApx15Gsof.yaw*DEGREE2PI);

    imu_msg.orientation_covariance[0] = myApx15Gsof.roll;
    imu_msg.orientation_covariance[1] = myApx15Gsof.pitch;
    imu_msg.orientation_covariance[2] = myApx15Gsof.yaw;
    imu_msg.orientation_covariance[3] = myApx15Gsof.rollRMS;
    imu_msg.orientation_covariance[4] = myApx15Gsof.pitchRMS;
    imu_msg.orientation_covariance[5] = myApx15Gsof.yawRMS;

    imu_msg.angular_velocity.x = myApx15Gsof.angularRate[0]*DEGREE2PI;
    imu_msg.angular_velocity.y = myApx15Gsof.angularRate[1]*DEGREE2PI;
    imu_msg.angular_velocity.z = myApx15Gsof.angularRate[2]*DEGREE2PI;

    imu_msg.linear_acceleration.x = myApx15Gsof.acceleration[0];
    imu_msg.linear_acceleration.y = myApx15Gsof.acceleration[1];
    imu_msg.linear_acceleration.z = myApx15Gsof.acceleration[2];

    imu_pub.publish(imu_msg);
  }

  if (gnss_pub.getNumSubscribers() > 0)
  {
    navfix_msg.header.stamp.sec = imu_msg.header.stamp.sec;
    navfix_msg.header.stamp.nsec = imu_msg.header.stamp.nsec;

    navfix_msg.status.status  = myApx15Gsof.GPS_Quality;
    navfix_msg.status.service = myApx15Gsof.IMU_AlignmentStatus;

    navfix_msg.latitude  = myApx15Gsof.latitude;
    navfix_msg.longitude = myApx15Gsof.longitude;
    navfix_msg.altitude  = myApx15Gsof.altitude;

    navfix_msg.position_covariance[0] = myApx15Gsof.northPositionRMS;
    navfix_msg.position_covariance[1] = myApx15Gsof.eastPositionRMS;
    navfix_msg.position_covariance[2] = myApx15Gsof.downPositionRMS;

    gnss_pub.publish(navfix_msg);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apx15_gsof_driver");

  std::string port;
  int32_t baud;

  ros::NodeHandle gnss_inertial_nh("gnss_inertial"), private_nh("~");
  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<int32_t>("baud", baud, 115200);

  apx15RosDriver myApx15RosDriver(port, baud, &gnss_inertial_nh);

  return 0;
}
