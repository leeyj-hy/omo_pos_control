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
        float track = 0.5; //length between wheels
        float angular_vel = 0.1;
        float angular_tol = 0.1;
        
        double TP=0;
        double EP=0;
        int i = 0;

    public:
        unsigned int duration = 0;
        bool timer_avialbler = true;


        omo_pos_con()
        {
            cli1 = n.serviceClient<robot_msgs::mrkrPos>("/marker_pose_srv");
            pub1 = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
            // timer = n.createTimer(Duration(rate), &omo_pos_con::_callback, this);
            srv1 = n.advertiseService("/omo_con_srv", &omo_pos_con::is_omo_aligned, this);

            omo_con();
        }

        void omo_con()
        {
            ROS_INFO("OMO_CLI started");
            // if(cli1.call(mrkr_pos)&&mrkr_pos.response.is_pos_return)
            // {
            //     this -> rot_z = mrkr_pos.response.rot_y;
            //     ROS_INFO("MRKR_POS received %f", this -> rot_z);
            //     omo_mv_cal(this -> rot_z);
            // }
        }

        void omo_mv_cal(double angle_r)
        {
            ROS_INFO("angle_r : %f", angle_r);
            // double angle_dist = angle_r*(track/2);
            double angle_dist = angle_r;
            ROS_INFO("angle_dist = %f", angle_dist);
            TP = angle_dist/angular_vel;
            ROS_INFO("target point : %f", TP);
            omo_run(TP);
        }

        void omo_run(double TP_tmp)
        {
            //timer_avialbler = true;
            double SP = ros::Time::now().toSec();
            //TP=abs(TP);
            ROS_INFO("SP : %f", SP);
            if(TP_tmp > 0)
            {
                ROS_INFO("TP > 0, duration : %f", TP_tmp);
                omo_con_twist.angular.z = -1 * angular_vel;
                
            }

            else
            {
                ROS_INFO("TP < 0, duration : %f", TP_tmp);
                omo_con_twist.angular.z =  angular_vel;
                
            }
            
            
        
            //while(duration < -1 * TP&&ros::ok())
            //{
            //    ROS_INFO("TP < 0, duration : %d", duration);
            //    omo_con_twist.angular.z = -1 * angular_vel;
            //    pub1.publish(omo_con_twist);
            //}
            while (ros::ok())
            {
                pub1.publish(omo_con_twist);
                if(abs(TP_tmp) < ros::Time::now().toSec() - SP)
                {
                    omo_con_twist.angular.z = 0;
                    ROS_INFO("vel = 0");
                    break;
                }
            }        
            
            
            ROS_INFO("OMO POS PUBLISHED : %f", omo_con_twist.angular.z);
            pub1.publish(omo_con_twist);
            timer_avialbler = false;
        }
        
        // void _callback(const ros::TimerEvent& event) //mSec
        // {
        //     if(timer_avialbler)
        //         this -> duration ++;
        // }

        bool is_omo_aligned(robot_msgs::omoalign::Request &req,
                            robot_msgs::omoalign::Response &res)
        {
            while (ros::ok()&&cli1.call(mrkr_pos)&&mrkr_pos.response.is_pos_return)
            {
                i++;
                this -> rot_z = mrkr_pos.response.rot_y;
                ROS_INFO("MRKR_POS received %f", this -> rot_z);
                ROS_INFO("iter = %d", this -> i);
                if(abs(mrkr_pos.response.rot_y)<angular_tol) 
                {
                    ROS_WARN("auto align success");
                    res.is_aligned = true;
                    break;
                }
                if(i>10)
                {
                    ROS_WARN("can't auto align");
                    res.is_aligned = false;
                    break;
                }
                omo_mv_cal(this -> rot_z);
            } 
            
            return true;
        }
        
        
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omo_pos_con");
    omo_pos_con OPC_obj;
    //ROS_INFO("%d", OPC_obj.duration);
    // if(OPC_obj.timer_avialbler)
    ros::spin();
    return 0;
}