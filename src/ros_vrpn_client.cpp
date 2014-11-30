/*
# Copyright (c) 2014, Automatic Coordination of Teams Laboratory (ACT-Lab), 
#                     University of Souther California
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

## author Wolfgang Hoenig (ACT-Lab, USC)
## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>
#include <math.h>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h> 

#include <tf/LinearMath/Quaternion.h>

void VRPN_CALLBACK track_target(void*, const vrpn_TRACKERCB);

class RigidBody {
    public:
        RigidBody(
            ros::NodeHandle& nh,
            const std::string& ip,
            int port,
            const std::string& frame_id,
            const std::string& child_frame_id)
            : m_br()
            , m_connection()
            , m_tracker()
            , m_target()
        {
            std::string connec_nm = ip + ":" + boost::lexical_cast<std::string>(port);
            m_connection = vrpn_get_connection_by_name(connec_nm.c_str());

            // child_frame_id is the same as remote object name
            m_tracker = new vrpn_Tracker_Remote(child_frame_id.c_str(), m_connection);
            m_tracker->register_change_handler(this, track_target);

            m_target.header.frame_id = frame_id;
            m_target.child_frame_id = child_frame_id;
        }

        ~RigidBody()
        {
            // There is no way to close the connection in VRPN.
            // Leak both m_tracker and m_connection to avoid crashes on shutdown.
        }

        void step_vrpn()
        {
            m_tracker->mainloop();
            m_connection->mainloop();
        }

    private:
        friend void VRPN_CALLBACK track_target(void*, const vrpn_TRACKERCB);

        void on_change(const vrpn_TRACKERCB t)
        {
            m_target.transform.translation.x = t.pos[0];
            m_target.transform.translation.y = t.pos[1];
            m_target.transform.translation.z = t.pos[2];

            m_target.transform.rotation.x = t.quat[0];
            m_target.transform.rotation.y = t.quat[1];
            m_target.transform.rotation.z = t.quat[2];
            m_target.transform.rotation.w = t.quat[3];

            m_target.header.stamp = ros::Time::now();

            m_br.sendTransform(m_target);
        }

    private:
        tf::TransformBroadcaster m_br;
        vrpn_Connection* m_connection;
        vrpn_Tracker_Remote* m_tracker;
        geometry_msgs::TransformStamped m_target;
};

void VRPN_CALLBACK track_target(void* userData, const vrpn_TRACKERCB t)
{
    RigidBody* r = static_cast<RigidBody*>(userData);
    r->on_change(t);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_vrpn_client");
    ros::NodeHandle nh("~");

    std::string ip;
    int port;
    std::string frame_id;
    std::string child_frame_id;

    nh.param<std::string>("ip", ip, "localhost");
    nh.param<int>("port", port, 3883);
    nh.param<std::string>("frame_id", frame_id, "world");
    nh.param<std::string>("child_frame_id", child_frame_id, "Tracker0");

    RigidBody body(nh, ip, port, frame_id, child_frame_id);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        body.step_vrpn();
        loop_rate.sleep();
    }

    return 0;
}



