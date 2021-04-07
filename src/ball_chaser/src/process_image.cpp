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


    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int index = 0;
    int position = -1;
    bool white_ball = false;
    int white_pixel = 255;
    int left_pos = img.width / 3;
    int center_end = 2 * img.width / 3;
    
    
    for (int i = 0; i < img.height; i++)
    {
        for (int j = 0; i < img.width; j++)
        {
           index = (i * img.width +j)*3; 
           if(img.data[index] == white_pixel)
           {
                position = j;
                white_ball = true;
                break;
           }
        }
        if(white_ball)
        {
            break;
        }
        
    }
    
   if( position >= 0 && position < left_pos)
   {
    drive_robot(0.1, 0.5); // move left
   }
   else if (position >= left_pos && position < center_end)
   {
    drive_robot(0.5, 0.0); // move forward
   }
   else if (position >= center_end)
   {
    drive_robot(0.1, -0.5); // move to the right
   }
   else if (position == -1)
   {
    drive_robot(0.0, 0.0); // stop robot
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

