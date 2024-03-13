#include "async_client.h"
#include "rclcpp/rclcpp.hpp"
#include "wj_716N_lidar_protocol.h"
using namespace wj_lidar;

wj_716N_lidar_protocol *protocol;
Async_Client *client;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wj_716N_lidar_01");
    auto logger = node->get_logger();
    std::string port ;
    std::string hostname ;
    node->declare_parameter("hostname", "192.168.0.2");
    node->declare_parameter("port", "2110");
    if (node->get_parameter("hostname", hostname))
    {
        RCLCPP_INFO(logger, "Got hostname parameter: %s", hostname.c_str());
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to get hostname parameter");
    }

    if (node->get_parameter("port", port))
    {
        RCLCPP_INFO(logger, "Got port parameter: %s", port.c_str());
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to get port parameter");
    }
    cout << "laser ip: " << hostname << ", port:" << port << endl;

    protocol = new wj_716N_lidar_protocol();

    client = new Async_Client(protocol);
    protocol->heartstate = false;
    while (!client->m_bConnected)
    {
        RCLCPP_INFO(logger,"Start connecting laser!");
        if (client->connect(hostname.c_str(), atoi(port.c_str())))
        {
            RCLCPP_INFO(logger,"Succesfully connected. Hello wj_716N_lidar!");
        }
        else
        {
            RCLCPP_INFO(logger,"Failed to connect to laser. Waiting 5s to reconnect!");
        }
        rclcpp::sleep_for(std::chrono::seconds(5));
    }
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::seconds(2));
        if (client->m_bConnected)
        {
            if (protocol->heartstate)
            {
                protocol->heartstate = false;
            }
            else
            {
                client->m_bConnected = false;
            }
        }
        else
        {
            // reconnect
            if (!client->m_bReconnecting)
            {
                boost::thread t(boost::bind(&Async_Client::reconnect, client));
            }
        }
    }
}
