#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the robot");
    
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
    
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
   
    

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int size = img.height * img.width * 3; // each pixel having 3 values.
    int left_size = img.width / 3;
    int right_size_begin = 2 * img.width / 3;
    
    bool left = false;
    bool right = false;
    bool center  = false;
    
    for (int i = 0; i < size;i+=3)
    {
        if( (img.data[i] == white_pixel) and (img.data[i+1] == white_pixel) and (img.data[i+2] == white_pixel) )
        {
            if (i >= 0 and i < left_size)
            {
                left = true;
                break;
            }
            else if(i >= left_size and i < right_size_begin)
            {
                center = true;
                break;
            }
            else if( i >= right_size_begin and i < size)
            {
                right = true;
                break;
            }
            
        }          
    }
   
    if (left){
        drive_robot(0.0, 0.5); // move robot to the left
    }
        
    else if (center){
        drive_robot(0.5, 0.0); // move the robot forward.
    }    
        
    else if (right){
        drive_robot(0.0, -0.5); // move robot to the right
    }
       
}
        
    
int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

