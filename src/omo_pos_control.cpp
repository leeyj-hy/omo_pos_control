#include "ros/ros.h"
#include "robot_msgs/mrkrPos.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace ros;

class omo_pos_con
{
    private:
    
        ros::NodeHandle n;
        ros::ServiceClient cli1;
        ros::Publisher pub1;
        ros::Timer timer;

        robot_msgs::mrkrPos mrkr_pos;
        geometry_msgs::Twist omo_con_twist;
        
        float rot_z;
        float rate = 0.1; //timer interrupt rate
        unsigned int duration = 0;
        bool timer_avialbler = false;
        float track = 0.5; //length between wheels
        float angular_vel = 0.1;
        float TP = 0;

    public:
        omo_pos_con()
        {
            cli1 = n.serviceClient<robot_msgs::mrkrPos>("/marker_pose_srv");
            pub1 = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
            timer = n.createTimer(Duration(rate), &omo_pos_con::_callback, this, false);
            omo_con();
        }

        void omo_con()
        {
            ROS_INFO("OMO_CLI started");
            if(cli1.call(mrkr_pos))
            {
                this -> rot_z = -1*mrkr_pos.response.rot_y;
                omo_mv_cal(this -> rot_z);
                ROS_INFO("MRKR_POS received");
            }
        }

        void omo_mv_cal(float angle_d)
        {
            float angle_r = 3.1415/180*angle_d;
            float angle_dist = angle_r*track/2;
            TP = angle_dist/angular_vel;
            timer_avialbler = true;
            omo_run();
        }

        void omo_run()
        {
            if(duration )
                omo_con_twist.angular.z = angular_vel;
            else
                omo_con_twist.angular.z = 0;
            
            ROS_INFO("OMO POS PUBLISHED : %f", omo_con_twist.angular.z);
            pub1.publish(omo_con_twist);
        }
        
        void _callback(const ros::TimerEvent& event) //mSec
        {
            if(timer_avialbler)
                this -> duration ++;
        }
        
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omo_pos_con");
    omo_pos_con OPC_obj;
    
    ros::spin();
    return 0;
}