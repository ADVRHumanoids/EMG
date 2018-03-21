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

#include <EMG_plugin.h>
#define PORT 41001

/* Specify that the class XBotPlugin::EMG is a XBot RT plugin with name "EMG" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::EMG)

namespace XBotPlugin {

bool EMG::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/EMG_log");

    int argc = 1;
    const char *arg = "dummy_arg";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;
     // ROS init
    ros::init(argc, argv, "EMG");    
    _nh = std::make_shared<ros::NodeHandle>();     
    _pubEMG = _nh->advertise<std_msgs::Float32>("/emg_value", 1);    
    
    /* create a UDP socket */ 
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { perror("cannot create socket\n"); return false; }
    /* bind the socket to any valid IP address and a specific port */ 
    memset((char *)&myaddr, 0, sizeof(myaddr)); 
    myaddr.sin_family = AF_INET; 
    myaddr.sin_addr.s_addr = inet_addr("10.255.32.103");// htonl(INADDR_ANY); 
    myaddr.sin_port = htons(PORT); 
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) { perror("bind failed"); return false; }

    std::cout<<"waiting on port"<< PORT << std::endl;
    
    return true;


}

void EMG::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /EMG_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the robot starting config to a class member */
    _start_time = time;
}

void EMG::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /EMG_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void EMG::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /EMG_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */

    if(!current_command.str().empty()){

        if(current_command.str() == "MY_COMMAND_1"){
            /* Handle command */
        }

        if(current_command.str() == "MY_COMMAND_2"){
            /* Handle command */
        }

    }
       
    recvlen = recvfrom(fd, &value, sizeof(float), 0, (struct sockaddr *)&remaddr, &addrlen); 
    if (recvlen > 0) {
      std::cout<<"received message: "<< value <<std::endl;  
      std_msgs::Float32 emgMsg;
      emgMsg.data= value ;   
      _pubEMG.publish(emgMsg);      
    }

}

bool EMG::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

EMG::~EMG()
{
  
}

}
