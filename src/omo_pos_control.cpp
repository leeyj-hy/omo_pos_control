#include "ros/ros.h"
#include "robot_msgs/mrkrPos.h"
#include "robot_msgs/omoalign.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace ros;

class omo_pos_con
{
    private:
    
        ros::NodeHandle n;
        ros::ServiceClient cli1;
        ros::Publisher pub1;
        ros::ServiceServer srv1;
        robot_msgs::omoalign oa;
        ros::Timer timer;

        robot_msgs::mrkrPos mrkr_pos;
        geometry_msgs::Twist omo_con_twist;
        
        float rot_z;
        float rate = 0.1; //timer interrupt rate
        bool timer_avialbler = false;
        float track = 0.5; //length between wheels
        float angular_vel = 0.01;
        float TP = 0;

    public:
        unsigned int duration = 0;


        omo_pos_con()
        {
            cli1 = n.serviceClient<robot_msgs::mrkrPos>("/marker_pose_srv");
            pub1 = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
            timer = n.createTimer(Duration(rate), &omo_pos_con::_callback, this, false);
            srv1 = n.advertiseService("/omo_con_srv", &omo_pos_con::is_omo_aligned, this);

            omo_con();
        }

        void omo_con()
        {
            ROS_INFO("OMO_CLI started");
            if(cli1.call(mrkr_pos)&&mrkr_pos.response.is_pos_return)
            {
                this -> rot_z = -1*mrkr_pos.response.rot_y;
                ROS_INFO("MRKR_POS received %f", this -> rot_z);
                omo_mv_cal(this -> rot_z);
            }
        }

        void omo_mv_cal(float angle_d)
        {
            float angle_r = 0.017452777*angle_d;
            float angle_dist = angle_r*track/2;
            TP = angle_dist/angular_vel/rate;
            ROS_INFO("target point : %f", TP);
            timer_avialbler = true;
            omo_run();
        }

        void omo_run()
        {
            if(duration < 0)
                while(duration < TP)
                    omo_con_twist.angular.z = angular_vel;
            else
                while(duration > -1 * TP)
                    omo_con_twist.angular.z = -1 * angular_vel;
            
            omo_con_twist.angular.z = 0;
            
            ROS_INFO("OMO POS PUBLISHED : %f", omo_con_twist.angular.z);
            pub1.publish(omo_con_twist);
        }
        
        void _callback(const ros::TimerEvent& event) //mSec
        {
            if(timer_avialbler)
                this -> duration ++;
        }

        bool is_omo_aligned(robot_msgs::omoalign::Request &req,
                            robot_msgs::omoalign::Response &res)
        {
            this -> rot_z = -1*mrkr_pos.response.rot_y;
            ROS_INFO("MRKR_POS received2");
            omo_mv_cal(this -> rot_z);
        }
        
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omo_pos_con");
    omo_pos_con OPC_obj;
    ROS_INFO("%d", OPC_obj.duration);
    // ros::spin();
    return 0;
}