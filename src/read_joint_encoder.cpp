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


uint8_t GrayDecode(uint8_t gray);

int fifo_vid;
int fifo_pid;
char parent_link_str[128];
char child_link_str[128];
char joint_name_str[128];
double ROS_loop_rate_Hz;


int main(int argc, char **argv)
{
    struct ftdi_context *ftdi;
    int f,i, joint_position;
    unsigned char buf[1];
    int retval = 0;
    




    // setup ROS parameter here
    ros::init(argc, argv, "ThorPro_joint_encoder");
    
    ros::NodeHandle n("~");
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



// setup FIFO
    if ((ftdi = ftdi_new()) == 0)
    {
        ROS_ERROR( "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    f = ftdi_usb_open(ftdi, fifo_vid, fifo_pid);

    if (f < 0 && f != -5)
    {
        ROS_ERROR( "unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        ROS_ERROR( "VID: %04X PID: %04X\n", fifo_vid, fifo_pid);
        retval = 1;
        return EXIT_FAILURE;
    }

    ROS_INFO("FTDI FIFO open succeeded: %d\n",f);




// enter ROS loop
    ros::Rate loop_rate(ROS_loop_rate_Hz);
    ROS_INFO("ROS FIFO Reading Loop Start!");
    while(ros::ok())
    {

        buf[0] =  0;
        f = ftdi_read_pins(ftdi, buf);
        if (f < 0)
        {
            ROS_ERROR("FIFO read failed , error %d (%s)\n",f, ftdi_get_error_string(ftdi));
        }
        else
        {
            ROS_INFO("FIFO read data: 0x%02hhx \n",buf[0]);
            joint_position=GrayDecode(buf[0]) ;
            ROS_INFO("FIFO joint position: %d \n",joint_position);
        }





    loop_rate.sleep();
    }

// Exit ROS loopc, clean up
    ROS_INFO("Closing FTDI FIFO!\n");
    printf("\nExit.... Closing FTDI FIFO!....\r\n");
    
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
