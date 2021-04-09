#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> // For publishing compressed image
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp> // for image processing and GUI Modules
#include <opencv2/highgui/highgui.hpp> // for image processing and GUI modules

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
void process_image_callback(const sensor_msgs::ImageConstPtr& source)
{


    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    cv_bridge::CvImagePtr cv_ptr;
    int white_pixel = 255;
    int position = - 1;
    
    try {
        cv_ptr = cv_bridge::toCvCopy(source,  sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception %s", e.what());
        return;
    }
    
    cv::Mat& mat = cv_ptr->image;
    cv::Vec3b pixel = mat.at<cv::Vec3b>(cv_ptr->image.rows, cv_ptr->image.cols);

    
    for (int i = 0; i < (cv_ptr->image.rows); i++)
    {
        for (int j = 0; j < (cv_ptr->image.cols);i++)
        {
            if (pixel[0] == white_pixel && pixel[1] == white_pixel && pixel[3] == white_pixel)
            {
                int position = j;
                break;
            }
        }
        if (position > - 1)
        {
        break;
        }
    }
    if (position != -1)
    {
        if ( position < cv_ptr->image.cols / 3)
        {
            drive_robot(0.1, 0.5); // move left
        }
        else if ( position >= cv_ptr->image.cols / 3 && position < 2 * cv_ptr->image.cols /3)
        {
            drive_robot(0.5, 0.0); // mve forward
        }
        else if( position >= 2 * cv_ptr->image.cols /3)
        {
            drive_robot(0.1, -0.5); // move right
        }
        
    }
    else {
        drive_robot(0.0 , 0.0); // don't move
    }
   
    
  }  
int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    
    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    image_sub = it.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    //cvNamedWindow("windowName", CV_WINDOW_AUTOSIZE);
    //cvStartWindowThread();

    // Handle ROS communication events
    ros::spin();

    return 0;
}

