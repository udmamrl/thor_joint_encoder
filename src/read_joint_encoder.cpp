/* This program is distributed under the GPL, version 2 */
/*!
***********************************************************************
* file read_joint_encoder.cpp
* Thro Pro Joint 8 bit Encoder driver , IC: FT245RL FIFO mode
* Encoder http://www.encoder.com/literature/datasheet-960.pdf
* 8 bit Graycode P/N 960-T-05-G-8-PP-FA-G/4-N
* using libftdi1 library
* install udev file to /etc/udev/rules.d/99-libftdi_encoder.rules
* author Cheng-Lung Lee. University of Detroit Mercy
* Advanced mobile Robotics Lab
* date 2013/09/04
*
************************************************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ftdi.h>

// ROS stuff
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

uint8_t GrayDecode(uint8_t gray);

int fifo_vid;
int fifo_pid;
std::string parent_link_str;
std::string child_link_str;
std::string joint_name_str;
double ROS_loop_rate_Hz;


int main(int argc, char **argv)
{
    struct ftdi_context *ftdi;
    int f,i, joint_position;
    unsigned char buf[1];
    int retval = 0;
    double joint_angle_rad;
    double angle_Offset=-M_PI;
    double angle_Scale=2.0*M_PI/256.0;
    int MaxErrors = 10;
    int ErrorCounter =0;


    // setup ROS parameter here
    ros::init(argc, argv, "ThorPro_joint_encoder");
    
    ros::NodeHandle n("~");
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
/*
    tf::TransformBroadcaster broadcaster;
*/
    // read parameters
    n.param("fifo_vid", fifo_vid, 0x0403);
    n.param("fifo_pid", fifo_pid, 0x6999);
    n.param("ROS_loop_rate_Hz", ROS_loop_rate_Hz, 20.0);

 /*
   TODO : set up jointstate publisher , joint_states (sensor_msgs/JointState) 
   reference:http://wiki.ros.org/joint_state_publisher
   example code :https://github.com/ros/robot_model/blob/indigo-devel/joint_state_publisher/joint_state_publisher/joint_state_publisher
*/
/*
    n.param<std::string>("parent_link", parent_link_str, "base_link");
    n.param<std::string>("child_link",  child_link_str , "rear_link");
    n.param<std::string>("joint_name",  joint_name_str , "thor_joint");
*/



    // message declarations
    sensor_msgs::JointState joint_state;
/*
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";
    */



// setup FIFO
    if ((ftdi = ftdi_new()) == 0)
    {
        ROS_ERROR( "Thor Joint Encoder, ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    f = ftdi_usb_open(ftdi, fifo_vid, fifo_pid);

    if (f < 0 && f != -5)
    {
        ROS_ERROR( "Thor Joint Encoder, unable to open FTDI FIFO device: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        ROS_ERROR( "VID: %04X PID: %04X\n", fifo_vid, fifo_pid);
        retval = 1;
        return EXIT_FAILURE;
    }

    ROS_INFO("Thor Joint Encoder FIFO open succeeded! Code: %d\n",f);




// enter ROS loop
    ros::Rate loop_rate(ROS_loop_rate_Hz);
    ROS_INFO("Thor Joint Encoder FIFO Reading Loop Start!");
    while(ros::ok())
    {

        buf[0] =  0;
        f = ftdi_read_pins(ftdi, buf);
        if (f < 0)
        {
            ROS_ERROR("Thor Joint Encoder FIFO read failed , error %d (%s)\n",f, ftdi_get_error_string(ftdi));
            ErrorCounter++;
            if (ErrorCounter>MaxErrors)
            { // if too much back to back errors
              // quit ROS try again
              ROS_ERROR("Thor Joint Encoder FIFO read failed %d times! Shutdown node! \n",MaxErrors);
              ros::shutdown();
            }
        }
        else
        {
            //ROS_INFO("FIFO read data: 0x%02hhx \n",buf[0]);
            joint_position=GrayDecode(buf[0]) ;
            joint_angle_rad=angle_Scale*joint_position+angle_Offset;
            //ROS_INFO("FIFO joint position: %8.3f rad \n",joint_angle_rad);
            //ROS_INFO("FIFO joint position: %8.3f degree\n",joint_angle_rad/degree);
            ErrorCounter=0;
        }
    //


        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] ="joint_angle";
        joint_state.position[0] = joint_angle_rad;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        /*
        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);


        broadcaster.sendTransform(odom_trans);
        */



    loop_rate.sleep();
    }

// Exit ROS loopc, clean up
    ROS_INFO("Thor Joint Encoder, Closing FTDI FIFO!\n");
    printf("\nExit.... Closing Thor Joint Encoder FTDI FIFO!....\r\n");
    
    ftdi_usb_close(ftdi);
done:
    ftdi_free(ftdi);

return 0;


}
uint8_t GrayDecode(uint8_t gray) {
    uint8_t out;
    uint8_t i;

    out = gray;
    for(i = out >> 1; i; i >>= 1)
        out ^= i;
    
    return out;
}
