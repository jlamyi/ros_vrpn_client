/*
# Copyright (c) 2011, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
*/

//== This application listens for a rigid body named 'Tracker' on a remote machine
//== and publishes & tf it's position and orientation through ROS.


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>
#include <math.h>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h> 

#include <tf/LinearMath/Quaternion.h>

void VRPN_CALLBACK track_target (void *, const vrpn_TRACKERCB t);

class Rigid_Body {
    private:
        ros::Publisher m_target_pub;
        tf::TransformBroadcaster m_br;
        vrpn_Connection* m_connection;
        vrpn_Tracker_Remote* m_tracker;
        geometry_msgs::TransformStamped m_target;
        std::string m_frame_id;

    public:
        Rigid_Body(
            ros::NodeHandle& nh,
            const std::string& server_ip,
            int port,
            const std::string& frame_id)
            : m_frame_id(frame_id)
        {
            m_target_pub = nh.advertise<geometry_msgs::TransformStamped>("pose", 100);
            std::string connec_nm = server_ip + ":" + boost::lexical_cast<std::string>(port);
            m_connection = vrpn_get_connection_by_name(connec_nm.c_str());
            std::string target_name = nh.getNamespace().substr(1);
            m_tracker = new vrpn_Tracker_Remote(target_name.c_str(), m_connection);
            m_tracker->register_change_handler(this, track_target);
        }

        void step_vrpn()
        {
            m_tracker->mainloop();
            m_connection->mainloop();
        }

        void on_change(const vrpn_TRACKERCB t)
        {
            m_target.transform.translation.x = t.pos[0];
            m_target.transform.translation.y = t.pos[1];
            m_target.transform.translation.z = t.pos[2];

            m_target.transform.rotation.x = t.quat[0];
            m_target.transform.rotation.y = t.quat[1];
            m_target.transform.rotation.z = t.quat[2];
            m_target.transform.rotation.w = t.quat[3];

            m_target.header.frame_id = "vrpn";
            m_target.child_frame_id = m_frame_id;
            m_target.header.stamp = ros::Time::now();

            m_br.sendTransform(m_target);
            m_target_pub.publish(m_target);
        }
};

//== Tracker Position/Orientation Callback ==--
void VRPN_CALLBACK track_target (void * userData, const vrpn_TRACKERCB t)
{
    Rigid_Body* r = static_cast<Rigid_Body*>(userData);
    r->on_change(t);
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_vrpn_client");
    ros::NodeHandle nh("~");

    std::string frame_id = nh.getNamespace();

    std::string vrpn_server_ip;
    int vrpn_port;
    std::string tracked_object_name;

    nh.param<std::string>("vrpn_server_ip", vrpn_server_ip, std::string());
    nh.param<int>("vrpn_port", vrpn_port, 3883);

    std::cout<<"vrpn_server_ip:"<<vrpn_server_ip<<std::endl;
    std::cout<<"vrpn_port:"<<vrpn_port<<std::endl;

    Rigid_Body tool(nh, vrpn_server_ip, vrpn_port, frame_id);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        tool.step_vrpn();
        loop_rate.sleep();
    }

    return 0;
}



