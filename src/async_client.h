#ifndef ASYNC_CLIENT_H
#define ASYNC_CLIENT_H
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>
#include <unistd.h>
#include <vector>
#include "wj_716N_lidar_protocol.h"

using namespace std;
using boost::asio::ip::tcp;
using namespace boost::asio;
using namespace wj_lidar;

#define MAX_LENGTH 50000

class Async_Client
{
public:
    Async_Client(wj_716N_lidar_protocol *protocol);
    ~Async_Client();
    bool connect(string ip, int port);
    bool disconnect();
    void recvData();
    void reconnect();
    bool SendData(unsigned char buf[], int length);

    bool m_bConnected;
    bool m_bReconnecting;

private:
    wj_716N_lidar_protocol *m_pProtocol;
    string m_sServerIp;
    int m_iServerPort;
    io_service m_io;
    ip::tcp::endpoint m_ep;
    boost::shared_ptr<ip::tcp::socket> m_pSocket;
    boost::system::error_code ec;

    unsigned char m_aucReceiveBuffer[MAX_LENGTH];
};

#endif // ASYNC_CLIENT_H
