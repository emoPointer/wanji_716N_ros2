#include "wj_716N_lidar_protocol.h"
#include <iostream>

namespace wj_lidar
{
// bool wj_716N_lidar_protocol::setConfig(wj_716N_lidar::wj_716N_lidarConfig &new_config, uint32_t level)
// {
//     config_ = new_config;
//     scan.header.frame_id = config_.frame_id;
//     scan.angle_min = config_.min_ang;
//     scan.angle_max = config_.max_ang;
//     scan.range_min = config_.range_min;
//     scan.range_max = config_.range_max;
//     freq_scan = config_.frequency_scan;

//     scan.angle_increment = 0.017453 / 4;
//     if (freq_scan == 1) // 0.25°_15hz
//     {
//         scan.time_increment = 1 / 15.00000000 / 1440;
//         total_point = 1081;
//     }
//     else if (freq_scan == 2) // 0.25°_25hz
//     {
//         scan.time_increment = 1 / 25.00000000 / 1440;
//         total_point = 1081;
//     }

//     // adjust angle_min to min_ang config param
//     index_start = (config_.min_ang + 2.35619449) / scan.angle_increment;
//     // adjust angle_max to max_ang config param
//     index_end = 1081 - ((2.35619449 - config_.max_ang) / scan.angle_increment);
//     int samples = index_end - index_start;
//     scan.ranges.resize(samples);
//     scan.intensities.resize(samples);

//     cout << "frame_id:" << scan.header.frame_id << endl;
//     cout << "min_ang:" << scan.angle_min << endl;
//     cout << "max_ang:" << scan.angle_max << endl;
//     cout << "angle_increment:" << scan.angle_increment << endl;
//     cout << "time_increment:" << scan.time_increment << endl;
//     cout << "range_min:" << scan.range_min << endl;
//     cout << "range_max:" << scan.range_max << endl;
//     cout << "samples_per_scan:" << samples << endl;
//     return true;
// }

wj_716N_lidar_protocol::wj_716N_lidar_protocol()
{
    memset(&m_sdata, 0, sizeof(m_sdata));
    marker_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 50);
    rclcpp::Time scan_time = node->now();
    scan.header.stamp = scan_time;

    freq_scan = 1;
    node->get_parameter("frequency_scan", freq_scan);
    m_u32PreFrameNo = 0;
    m_u32ExpectedPackageNo = 0;
    m_n32currentDataNo = 0;
    total_point = 1081;

    scan.header.frame_id = "laser";
    node->get_parameter("frequency_scan", scan.header.frame_id);
    scan.angle_min = -2.35619449;
    node->get_parameter("min_ang", scan.angle_min);
    scan.angle_max = 2.35619449;
    node->get_parameter("max_ang", scan.angle_max);
    scan.range_min = 0;
    node->get_parameter("range_min", scan.range_min);
    scan.range_max = 30;
    node->get_parameter("range_max", scan.range_max);
    scan.angle_increment = 0.017453 / 4;
    if (freq_scan == 1) // 0.25°_15hz
    {
        scan.time_increment = 1 / 15.00000000 / 1440;
        total_point = 1081;
    }
    else if (freq_scan == 2) // 0.25°_25hz
    {
        scan.time_increment = 1 / 25.00000000 / 1440;
        total_point = 1081;
    }
    // adjust angle_min to min_ang config param
    index_start = (scan.angle_min + 2.35619449) / scan.angle_increment;
    // adjust angle_max to max_ang config param
    index_end = 1081 - ((2.35619449 - scan.angle_max) / scan.angle_increment);
    int samples = index_end - index_start;
    scan.ranges.resize(samples);
    scan.intensities.resize(samples);

    cout << "wj_716N_lidar_protocl start success" << endl;
    cout << "frame_id:" << scan.header.frame_id << endl;
    cout << "min_ang:" << scan.angle_min << endl;
    cout << "max_ang:" << scan.angle_max << endl;
    cout << "angle_increment:" << scan.angle_increment << endl;
    cout << "time_increment:" << scan.time_increment << endl;
    cout << "range_min:" << scan.range_min << endl;
    cout << "range_max:" << scan.range_max << endl;
    cout << "samples_per_scan:" << samples << endl;
}

bool wj_716N_lidar_protocol::dataProcess(unsigned char *data, const int reclen)
{
    if (reclen > MAX_LENGTH_DATA_PROCESS)
    {
        m_sdata.m_u32out = 0;
        m_sdata.m_u32in = 0;
        return false;
    }

    if (m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
    {
        m_sdata.m_u32out = 0;
        m_sdata.m_u32in = 0;
        return false;
    }
    memcpy(&m_sdata.m_acdata[m_sdata.m_u32in], data, reclen * sizeof(char));
    m_sdata.m_u32in += reclen;
    while (m_sdata.m_u32out < m_sdata.m_u32in)
    {
        if (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0xAA)
        {
            unsigned l_u32reallen =
                (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8) | (m_sdata.m_acdata[m_sdata.m_u32out + 3] << 0);
            l_u32reallen = l_u32reallen + 4;

            if (l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
            {
                if (OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out], l_u32reallen))
                {
                    m_sdata.m_u32out += l_u32reallen;
                }
                else
                {
                    cout << "continuous search frame header" << endl;
                    m_sdata.m_u32out++;
                }
            }
            else if (l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
            {
                m_sdata.m_u32out++;
            }
            else
            {
                break;
            }
        }
        else
        {
            m_sdata.m_u32out++;
        }
    } // end while(m_sdata.m_u32out < m_sdata.m_u32in)

    if (m_sdata.m_u32out >= m_sdata.m_u32in)
    {
        m_sdata.m_u32out = 0;
        m_sdata.m_u32in = 0;
    }
    else if (m_sdata.m_u32out < m_sdata.m_u32in && m_sdata.m_u32out != 0)
    {
        movedata(m_sdata);
    }
    return true;
}

void wj_716N_lidar_protocol::movedata(DataCache &sdata)
{
    for (int i = sdata.m_u32out; i < sdata.m_u32in; i++)
    {
        sdata.m_acdata[i - sdata.m_u32out] = sdata.m_acdata[i];
    }
    sdata.m_u32in = sdata.m_u32in - sdata.m_u32out;
    sdata.m_u32out = 0;
}

bool wj_716N_lidar_protocol::OnRecvProcess(unsigned char *data, int len)
{
    if (len > 0)
    {
        if (checkXor(data, len))
        {
            protocl(data, len);
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool wj_716N_lidar_protocol::protocl(unsigned char *data, const int len)
{
    if ((data[22] == 0x02 && data[23] == 0x02) || (data[22] == 0x02 && data[23] == 0x01)) // command type:0x02 0x01/0X02
    {
        heartstate = true;
        int l_n32TotalPackage = data[80];
        int l_n32PackageNo = data[81];
        unsigned int l_u32FrameNo = (data[75] << 24) + (data[76] << 16) + (data[77] << 8) + data[78];
        int l_n32PointNum = (data[83] << 8) + data[84];
        int l_n32Frequency = data[79];

        if (l_n32Frequency != freq_scan)
        {
            cout << "The scan frequency does not match the one you setted!" << endl;
            return false;
        }

        if (m_u32PreFrameNo != l_u32FrameNo)
        {
            m_u32PreFrameNo = l_u32FrameNo;
            m_u32ExpectedPackageNo = 1;
            m_n32currentDataNo = 0;
        }

        if (l_n32PackageNo == m_u32ExpectedPackageNo && m_u32PreFrameNo == l_u32FrameNo)
        {
            if (data[82] == 0x00) // Dist
            {
                for (int j = 0; j < l_n32PointNum; j++)
                {
                    scandata[m_n32currentDataNo] =
                        (((unsigned char)data[85 + j * 2]) << 8) + ((unsigned char)data[86 + j * 2]);
                    scandata[m_n32currentDataNo] /= 1000.0;
                    scanintensity[m_n32currentDataNo] = 0;
                    if (scandata[m_n32currentDataNo] > scan.range_max ||
                        scandata[m_n32currentDataNo] < scan.range_min || scandata[m_n32currentDataNo] == 0)
                    {
                        scandata[m_n32currentDataNo] = NAN;
                    }
                    m_n32currentDataNo++;
                }
                m_u32ExpectedPackageNo++;
            }
            else if (data[82] == 0x01 && m_n32currentDataNo >= total_point) // intensities
            {
                for (int j = 0; j < l_n32PointNum; j++)
                {
                    scanintensity[m_n32currentDataNo - total_point] =
                        (((unsigned char)data[85 + j * 2]) << 8) + ((unsigned char)data[86 + j * 2]);
                    m_n32currentDataNo++;
                }
                m_u32ExpectedPackageNo++;
            }

            if (m_u32ExpectedPackageNo - 1 == l_n32TotalPackage)
            {

                for (int i = index_start; i < index_end; i++)
                {
                    scan.ranges[i - index_start] = scandata[i];
                    if (scandata[i - index_start] == NAN)
                    {
                        scan.intensities[i - index_start] = 0;
                    }
                    else
                    {
                        scan.intensities[i - index_start] = scanintensity[i];
                    }
                }

                rclcpp::Time scan_time = node->now();
                scan.header.stamp = scan_time;
                marker_pub->publish(scan); // ros::
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool wj_716N_lidar_protocol::checkXor(unsigned char *recvbuf, int recvlen)
{
    int i = 0;
    unsigned char check = 0;
    unsigned char *p = recvbuf;
    int len;
    if (*p == 0xFF)
    {
        p = p + 2;
        len = recvlen - 6;
        for (i = 0; i < len; i++)
        {
            check ^= *p++;
        }
        p++;
        if (check == *p)
        {
            return true;
        }
        else
            return false;
    }
    else
    {
        return false;
    }
}

} // namespace wj_lidar
