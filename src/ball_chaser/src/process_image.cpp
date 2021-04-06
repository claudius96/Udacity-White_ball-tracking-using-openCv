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

 bool white_ball_side(sensor_msgs::Image img, int column_size, bool picture_side, int column_quartier)
    {
            int check_column_size = 0;
            int row = 0;
            int white_pixel = 255;
            while(i < img.height)
        {
            for( int column = column_quartier;column < column_size; column+=3)
            
            {   check_column_size = column + 2; // checking the third value of the RGB pixel if it does not depass left, right, center boundary size boundary
                if( check_column_size < column_size){
                
                    if( (img.data[column] == white_pixel) and (img.data[column+1] == white_pixel) 
                    and (img.data[column + 2] == white_pixel) )
                    {
                        picture_side = true;
                        return picture_side;
                    }
                }
                
            }
            
            row++;
            if( row < img.height){
                img.data + img.step;
            }
            
            
        }
        
    }

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{


    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int size = img.height * img.width * 3; // each pixel having 3 values.
    int left_side = img.width / 3;
    int right_side_begin = 2 * img.width / 3;
    
    int column_quartier_left = 0;
    int column_quartier_center = left_side;
    int column_quartier_right = right_side_begin;
    
    bool left = false;
    bool right = false;
    bool center  = false;
    
    // checking for white pixels
    left = white_ball_side(img, left_side, left, column_quartier_left);
    right = white_ball_side(img,img.width, right, column_quartier_right);
    center = white_ball_side(img,right_side_begin, center, column_quartier_center);
   
   
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

