#include <chrono>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "C3iroboticsLidar.h"
#include "../sdk/include/CSerialConnection.h"

#define DEG2RAD(x) ((x)*M_PI/180.)
//#ifndef _countof
//#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
//#endif

typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 10; //0
        angle = 4.0; //0
        distance = 5.0; //0
    }
    uint8_t signal;
    float   angle;
    float   distance;
}RslidarDataComplete;

using namespace std::chrono_literals;
using namespace std;
using namespace everest::hwdrivers;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DeltaLidar : public rclcpp::Node{
public: DeltaLidar() : Node("minimal_publisher"), count_(0){
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    
    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple()){
        printf("[AuxCtrl] Open serial port sucessful!\n");
    }else{
        printf("[AuxCtrl] Open serial port %s failed! \n", opt_com_path.c_str());
        exit(0);
    }

    printf("3iRoboticsLidar connected\n");

    robotics_lidar.initilize(&serial_connect);
    start_scan_time =this->now();
    
    timer_ = this->create_wall_timer(
      10ms, std::bind(&DeltaLidar::timer_callback, this));
  }

private:
  void publish_scan(_rslidar_data *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time,
                  float angle_min, float angle_max,
                  std::string frame_id)
{
    sensor_msgs::msg::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (360.0f - 1.0f);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = 5.0;

    scan_msg.ranges.resize(360, std::numeric_limits<float>::infinity());
    scan_msg.intensities.resize(360, 0.0);

    //Unpack data
    for (size_t i = 0; i < node_count; i++)
    {
        size_t current_angle = floor(nodes[i].angle);
        if(current_angle > 360.0)
        {
            printf("Lidar angle is out of range %d\n", (int)current_angle);
            continue;
        }
        float read_value = (float) nodes[i].distance;
        if (read_value < scan_msg.range_min || read_value > scan_msg.range_max)
            scan_msg.ranges[360- 1- current_angle] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[360 -1- current_angle] = read_value;

		float intensities = (float) nodes[i].signal;
		scan_msg.intensities[360 -1- current_angle] = intensities;

	}

    this->publisher_->publish(scan_msg);
}
  void timer_callback(){
    	TLidarGrabResult result = robotics_lidar.getScanData();
        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            case LIDAR_GRAB_SUCESS:
            {
                TLidarScan lidar_scan = robotics_lidar.getLidarScan();
                size_t lidar_scan_size = lidar_scan.getSize();
                std::vector<RslidarDataComplete> send_lidar_scan_data;
                send_lidar_scan_data.resize(lidar_scan_size);
                RslidarDataComplete one_lidar_data;
                for(size_t i = 0; i < lidar_scan_size; i++)
                {
                    one_lidar_data.signal = lidar_scan.signal[i];
                    one_lidar_data.angle = lidar_scan.angle[i];
                    one_lidar_data.distance = lidar_scan.distance[i];
                    send_lidar_scan_data[i] = one_lidar_data;
                }

            	float angle_min = DEG2RAD(0.0f);
            	float angle_max = DEG2RAD(359.0f);

				end_scan_time = this->now();
				scan_duration = (end_scan_time - start_scan_time).nanoseconds()*1e-9;
                //printf("Receive Lidar count %u!\n", lidar_scan_size);

                //if successful, publish lidar scan
                int start_node = 0, end_node = 359;
                publish_scan(&send_lidar_scan_data[0], lidar_scan_size,
                         start_scan_time, scan_duration,
                         angle_min, angle_max,
                         frame_id);

				start_scan_time = end_scan_time;

                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                printf("[Main] LIDAR_GRAB_ELSE!\n");
                break;
            }
        }
        //usleep(50);
  }
  
  int    opt_com_baudrate = 230400;
  string opt_com_path = "/dev/ttyUSB0";
  string frame_id = "laser_scan";
  CSerialConnection serial_connect;
  C3iroboticsLidar robotics_lidar;
  rclcpp::Time start_scan_time;
  rclcpp::Time end_scan_time;
  double scan_duration;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaLidar>());
  rclcpp::shutdown();
  return 0;
}
