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
            cli1 = n.serviceClient<robot_msgs::mrkrPos>("omo_pos_con_srv_cli");
            pub1 = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
            timer = n.createTimer(Duration(rate), _callback);
        }

        void omo_con()
        {
            if(cli1.call(mrkr_pos))
            {
                this -> rot_z = mrkr_pos.response.rot_z;
                omo_mv_cal(this -> rot_z);
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
            
            pub1.publish(omo_con_twist);
        }
        
        void _callback(const ros::TimerEvent&) //mSec
        {
            if(timer_avialbler)
                this -> duration ++;
        }
        
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omo_pos_con");
    omo_pos_con OPC_obj;
    
    ros::spinOnce();
    return 0;
}