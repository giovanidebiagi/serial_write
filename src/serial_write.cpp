#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string>

using namespace std;

float roll, pitch, heading;

void callback_roll(const std_msgs::Float64& msg)
{
    roll = msg.data;
}

void callback_pitch(const std_msgs::Float64& msg)
{
    pitch = msg.data;
}

void callback_heading(const std_msgs::Float64& msg)
{
    heading = msg.data;
}

int main(int argc, char **argv)
{
    ros::init (argc,argv,"serial_write");

    ros::NodeHandle node;

    ros::Subscriber sub_roll = node.subscribe("roll_glasses", 1, callback_roll);
    ros::Subscriber sub_pitch = node.subscribe("pitch_glasses", 1, callback_pitch);
    ros::Subscriber sub_heading = node.subscribe("heading_glasses", 1, callback_heading);


    int fd, n;          /*!< Auxiliary variables used in the reading function */
    string aux;    /*!< Refers to the serial port */

    ros::Rate loop_rate(50);

    /*! Initialize serial communication with Arduino */
    struct termios toptions;

    aux = "/dev/ttyUSB0";

    ROS_INFO_STREAM(aux);

    /*! Open file (serial port) */
    fd = open(aux.c_str(), O_RDWR | O_NOCTTY);

    /*! Wait for Arduino to reboot */
    usleep(1000000);



    /*! Get current serial port settings */
    tcgetattr(fd, &toptions);

    /*! Set 115200 baud both ways */
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);

    /*! 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    /*! Canonical mode */
    toptions.c_lflag |= ICANON;

    //float accel_x = 0, accel_y = 0, accel_z = 0, gyro_z = 6.8, push_button = 5.9, joy_x = 4.0, joy_y = 4.3;

    char buffer[64];
    char buffer_space[1] = {'J'};
    
    char buffer_roll[8];
    char buffer_pitch[8];
    char buffer_heading[8];
    
    /*
    char buffer_accel_x[5];
    char buffer_accel_y[5];
    char buffer_accel_z[10];
    char buffer_gyro_z[10];
    char buffer_push_button[10];
    char buffer_joy_x[10];
    char buffer_joy_y[10];
    */


    /*! Commit serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);

    while(ros::ok())
    {
        /*! Convert float to char */

        //cout << "Roll: " << roll << "   Pitch: " << pitch << "  Heading: " << heading << endl;

        snprintf(buffer_roll, sizeof(buffer_roll), "%f", roll);
        snprintf(buffer_pitch, sizeof(buffer_pitch), "%f", pitch);
        snprintf(buffer_heading, sizeof(buffer_heading), "%f", heading);
        
        /*
        snprintf(buffer_accel_x, sizeof(buffer_accel_x), "%f", accel_x);
        snprintf(buffer_accel_y, sizeof(buffer_accel_y), "%f", accel_y);
        snprintf(buffer_accel_z, sizeof(buffer_accel_z), "%f", accel_z);
        snprintf(buffer_gyro_z, sizeof(buffer_gyro_z), "%f", gyro_z);
        snprintf(buffer_push_button, sizeof(buffer_push_button), "%f", push_button);
        snprintf(buffer_joy_x, sizeof(buffer_joy_x), "%f", joy_x);
        snprintf(buffer_joy_y, sizeof(buffer_joy_y), "%f", joy_y);
        */

        /*! Start buffer to be sent to serial */
        strcpy(buffer, buffer_roll);
        //strcpy(buffer, buffer_accel_x);

        /*! Concatenate all data in one buffer */
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_pitch);
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_heading);

        /*
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_accel_y);
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_accel_z);
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_gyro_z);
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_push_button);
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_joy_x);
        strcat(buffer, buffer_space);
        strcat(buffer, buffer_joy_y);
        */
        
        //write(fd, "I", strlen("I"));

        /*! Write data to serial */
        write(fd, buffer, strlen(buffer));

        if(fd < 0)
        {
            cout << "Unable to open serial port!";
        }

        /*! Tell Arduino to start receiveing data */
        write(fd, "I", 1);     

        //usleep(500000);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}