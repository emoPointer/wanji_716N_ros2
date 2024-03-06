#ifndef WJ_716N_LIDAR_PROTOCOL_H
#define WJ_716N_LIDAR_PROTOCOL_H
#include "rclcpp/rclcpp.hpp"
#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/laser_scan.hpp"
#include "string.h"
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
// #include <dynamic_reconfigure/server.h>
#include <iostream>
// #include <sensor_msgs/LaserScan.h>
// #include <visualization_msgs/Marker.h>
//#include "rviz_common/msg/marker.hpp"
//#include <wj_716N_lidar/wj_716N_lidarConfig.h>
using namespace std;
namespace wj_lidar
{
#define MAX_LENGTH_DATA_PROCESS 200000
typedef struct TagDataCache
{
    unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
} DataCache;

class wj_716N_lidar_protocol
{
  public:
    wj_716N_lidar_protocol();
    bool dataProcess(unsigned char *data, const int reclen);
    bool protocl(unsigned char *data, const int len);
    bool OnRecvProcess(unsigned char *data, int len);
    bool checkXor(unsigned char *recvbuf, int recvlen);
    void send_scan(const char *data, const int len);
    // ros::NodeHandle nh;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("wj_716N_lidar_01");
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr marker_pub;
    sensor_msgs::msg::LaserScan scan;
    // bool setConfig(wj_716N_lidar::wj_716N_lidarConfig &new_config, uint32_t level);
    bool heartstate;

  private:
    void movedata(DataCache &sdata);
    DataCache m_sdata;
    // wj_716N_lidar::wj_716N_lidarConfig config_;
    unsigned int m_u32PreFrameNo;
    unsigned int m_u32ExpectedPackageNo;
    int m_n32currentDataNo;
    float scandata[1081];
    float scanintensity[1081];
    int total_point;
    int index_start;
    int index_end;
    int freq_scan;
};

} // namespace wj_lidar
#endif // WJ_716N_LIDAR_PROTOCOL_H
