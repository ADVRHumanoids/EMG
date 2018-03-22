/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef EMG_PLUGIN_H_
#define EMG_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace XBotPlugin {

/**
 * @brief EMG XBot RT Plugin
 *
 **/
class EMG : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~EMG();
    
protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;

    ros::Publisher _pubEMG;
    std::shared_ptr<ros::NodeHandle> _nh;
    double _start_time;

    Eigen::VectorXd _q0;

    XBot::MatLogger::Ptr _logger;
    struct sockaddr_in myaddr; /* our address */ 
    struct sockaddr_in remaddr; /* remote address */ 
    socklen_t addrlen = sizeof(remaddr); /* length of addresses */ 
    int recvlen; /* # bytes received */ 
    int fd; /* our socket */ 
    float value, max_val;
    int count;
    int state;

};

}

#endif // EMG_PLUGIN_H_
