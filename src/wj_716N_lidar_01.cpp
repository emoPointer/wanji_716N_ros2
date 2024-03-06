#include "async_client.h"
#include "rclcpp/rclcpp.hpp"
#include "wj_716N_lidar_protocol.h"
using namespace wj_lidar;

wj_716N_lidar_protocol *protocol;
Async_Client *client;

// void callback(wj_716N_lidar::wj_716N_lidarConfig &config, uint32_t level)
// {
//     protocol->setConfig(config, level);
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // ros::NodeHandle nh("~");
    auto node = rclcpp::Node::make_shared("wj_716N_lidar_01");
    auto logger = node->get_logger();
    std::string hostname;
    // nh.getParam("hostname", hostname);
    node->get_parameter("hostname", hostname);
    std::string port;
    // nh.getParam("port", port);
    node->get_parameter("port", port);
    cout << "laser ip: " << hostname << ", port:" << port << endl;

    protocol = new wj_716N_lidar_protocol();//���ﻹ��Ҫ��̬���ز�������Ϊ��������һ�Σ����ڵķ���ֻ���ڵ�һ�μ��ص�ʱ������ò��������������ǰѲ���д���ڴ������棬�ò�����ʱ��Ҫ�ǵ�
    // dynamic_reconfigure::Server<wj_716N_lidar::wj_716N_lidarConfig> server;//�����
    // dynamic_reconfigure::Server<wj_716N_lidar::wj_716N_lidarConfig>::CallbackType f;
    // f = boost::bind(&callback, _1, _2);
    // server.setCallback(f);

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

// #include "async_client.h"
// #include "wj_716N_lidar_protocol.h"
// #include <chrono>
// #include <iostream>
// #include <rcl_interfaces/msg/parameter_event.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <wj_716N_lidar/wj_716N_lidarConfig.h>

// using namespace std;
// using namespace std::chrono_literals;
// using namespace wj_lidar;

// wj_716N_lidar_protocol *protocol;
// Async_Client *client;

// void parameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
// {
//     // ����������µĻص������߼�
//     auto parameters = event->changed_parameters;
//     for (const auto &parameter : parameters)
//     {
//         if (parameter.name == "wj_716N_lidarConfig")
//         {
//             // ִ����Ӧ�����ø��²���
//             auto config = parameter.value.get<wj_716N_lidar::wj_716N_lidarConfig>();
//             protocol->setConfig(config, 0);
//         }
//     }
// }

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("wj_716N_lidar_01");
//     auto parameters = node->get_parameters({"hostname", "port"});
//     std::string hostname = parameters["hostname"].as_string();
//     std::string port = parameters["port"].as_string();
//     cout << "laser ip: " << hostname << ", port:" << port << endl;

//     protocol = new wj_716N_lidar_protocol();

//     // �������������������������¼�
//     auto parameter_subscriber =
//         node->create_subscription<rcl_interfaces::msg::ParameterEvent>("/parameter_events", 10, parameterCallback);

//     client = new Async_Client(protocol);
//     protocol->heartstate = false;
//     while (!client->m_bConnected)
//     {
//         RCLCPP_INFO(node->get_logger(), "Start connecting laser!");
//         if (client->connect(hostname.c_str(), atoi(port.c_str())))
//         {
//             RCLCPP_INFO(node->get_logger(), "Successfully connected. Hello wj_716N_lidar!");
//         }
//         else
//         {
//             RCLCPP_INFO(node->get_logger(), "Failed to connect to laser. Waiting 5s to reconnect!");
//         }
//         rclcpp::sleep_for(5s);
//     }
//     while (rclcpp::ok())
//     {
//         rclcpp::spin_some(node);
//         rclcpp::sleep_for(2s);
//         if (client->m_bConnected)
//         {
//             if (protocol->heartstate)
//             {
//                 protocol->heartstate = false;
//             }
//             else
//             {
//                 client->m_bConnected = false;
//             }
//         }
//         else
//         {
//             // reconnect
//             if (!client->m_bReconnecting)
//             {
//                 boost::thread t(boost::bind(&Async_Client::reconnect, client));
//             }
//         }
//     }

//     rclcpp::shutdown();
//     return 0;
// }