#include "ros/ros.h"							// Include ROS Files
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "controller_manager_msgs/SwitchController.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float64.h"
#include <opencv2/opencv.hpp>
#include "constants.h"
#include "math.h"
#include <iostream>
#include <fstream>


// Function of matrix multiplication
double** matMul(double** mat1, double** mat2, int n, int m, int o) {
    double** mat3;

    mat3 = new double*[4];
    for(int i = 0; i < n; i++) {
        mat3[i] = new double[o];
    }

    for(int i = 0; i < n; i++) {
        for(int j = 0; j < o; j++) {
            mat3[i][j] = 0;
            for(int k = 0; k < m; k++) {
                mat3[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }

    return mat3;
}

void setVelocities(double q1_vel_d, double q2_vel_d) {
    std_msgs::Float64 q1_vel;
    std_msgs::Float64 q2_vel;

    q1_vel.data = q1_vel_d;
    q2_vel.data = q2_vel_d;

    pub_q1_vel.publish(q1_vel);
    pub_q2_vel.publish(q2_vel);
}

// Funtion to handle Controller Switching from position controller to velocity controller
int switchControllers(NodeHandle n, V_string startControllers, V_string stopControllers) {
    try {
        
        switchControllerService = n.serviceClient<controller_manager_msgs::SwitchController>("/vbmbot/controller_manager/switch_controller");
        switchControllerService.waitForExistence();
        controller_manager_msgs::SwitchController req;
        req.request.start_controllers = startControllers;
        req.request.stop_controllers = stopControllers;
        req.request.strictness = 2;
        req.request.start_asap = false;
        req.request.timeout = 0;
        if(switchControllerService.call(req)) {
            if(req.response.ok == 1) {
                visualServoingFlag = true;
                string msg = "Switched from " + stopControllers[0].substr(7,19) + " to " + startControllers[0].substr(7,19);
                ROS_INFO(msg.c_str());
                return 0;
            } else {
                ROS_WARN("Switch Controller response not OK.");    
            }
        } else {
            ROS_ERROR("Error Calling the Switch Controller Service.");
        }
    } catch (ros::Exception &e ) {
        ROS_ERROR("Service Client Exception: %s", e.what());
    }
    return -1;
}

void goToPositions (double q1, double q2) {
    std_msgs::Float64 q1_pos_1;
    std_msgs::Float64 q2_pos_1;

    q1_pos_1.data = q1;
    q2_pos_1.data = q2;
    
    pub_q1_pos.publish(q1_pos_1);
    pub_q2_pos.publish(q2_pos_1);
    ROS_INFO("Published positions");
    sleep(5);

    spinOnceFlag = true;

    while(spinOnceFlag) {
        spinOnce();
    }

}

// Initialize all the Used Matrices
void initializeMatrices() {
    for(int i = 0; i < 2; i++) {
        lLe[i] = new double[8];
    }

    for(int i = 0; i < 8; i++) {
        errors[i] = new double[1];
    }

    for(int i = 0; i < 2; i++) {
        inverseJacobian[i] = new double[2];
    }

    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 8; j++) {
            if((i+j)%2 == 0) {
                lLe[i][j] = -lambda*( -f/ z);
            } else {
                lLe[i][j] = 0;
            }
        }
    }
}

// Get Joint States (Callback Function)
void getJointState(const sensor_msgs::JointState::ConstPtr& jointStatePtr) {
    q1Angle = jointStatePtr->position.at(0);
    q2Angle = jointStatePtr->position.at(1);
}


class ImageBasedVisualServo {
    private:
        image_transport::ImageTransport it;
        image_transport::Subscriber sub;
        bool a = false;

    public:
        ImageBasedVisualServo(NodeHandle n) : it(n) {

            // Subscribe to Image topic
            sub = it.subscribe("/vbmbot/camera1/image_raw", 1, &ImageBasedVisualServo::processImage, this);
        }

        ~ImageBasedVisualServo() {

        }

        void processImage(const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch(cv_bridge::Exception& e) {
                ROS_ERROR("CV Bridge Exception: %s", e.what());
                return;
            }

            // Store Image in a Mat Object
            image = cv_ptr->image;

            // Funtion to get Circle Centers
            getCircleCenters();
            
            // Start Visual Servoing if the velocity controller flag is set
            if(visualServoingFlag) {
                visualServo();
            }

            // If spin once flag is set then write image and set the flag to false
            if(spinOnceFlag) {
                imwrite(s, image);
                spinOnceFlag = false;
            }

            waitKey(3);
        }

        // Function to calculate error
        void calculateError() {

            for(int i = 0; i < 8; i = i + 2) {
                errors[i][0] = -1*(referenceCenters[i/2].x - currentCenters[i/2].x);
                errors[i+1][0] = -1*(referenceCenters[i/2].y - currentCenters[i/2].y);
            }

            cout << "Error: ";
            for(int i = 0; i < 8; i++) {
                cout << errors[i][0] << " ";
            }
            cout << endl;

        }

        void visualServo() {
            
            // If you get both angles 0 from the joint states then wait till you get non 0 angles.
            if(q1Angle == 0 && q2Angle == 0) {
                ROS_WARN("Angles are 0!");
                ROS_WARN("Will Wait to get non 0 angles before starting visual servoing");
                return;
            }
            
            // Set the current centers to the centers we got previously
            if(redCenter.x != -1) {
                currentCenters[0] = redCenter;
            }
            if(blueCenter.x != -1) {
                currentCenters[1] = blueCenter;
            }
            if(greenCenter.x != -1) {
                currentCenters[2] = greenCenter;
            }
            if(purpleCenter.x != -1) {
                currentCenters[3] = purpleCenter;
            }
            
            // Calculate the error with new centers
            calculateError();

            // Calculate Camera Velocity
            double** vc = matMul(lLe, errors, 2, 8, 1);
            cout << "Vc: ";
            for(int i = 0; i < 2; i++) {
                cout << vc[i][0] << " ";
            }
            cout << endl;

            double q1, q2;
            // get q1 and q2 angles within 0 to 2*pi
            q1 = fmod(q1Angle,M_PI*2);
            q2 = fmod(q2Angle,M_PI*2);
            cout << "Angles: " << q1 << " , " << q2 << endl;

            // Get the Inverse Jacobian Matrix
            double q1pq2 = atan2(sin(q1+q2),cos(q1+q2));
            double a = (-1 * link2Length * sin(q1)) - (link2Length * sin(q1pq2));
            double b = -1 * link3Length * sin(q1pq2);
            double c = (link2Length * cos(q1)) + (link3Length * cos(q1pq2));
            double d = link3Length * cos(q1pq2);
            double ad = a * d;
            double bc = b * c;
            double determinant = (ad - bc);

            inverseJacobian[0][0] = d / determinant;
            inverseJacobian[1][1] = a / determinant;
            inverseJacobian[0][1] = -b / determinant;
            inverseJacobian[1][0] = -c / determinant;

            cout << "Inverse Jacobian: " << endl;
            for(int i = 0; i < 2; i++) {
                for(int j = 0; j < 2; j++) {
                    cout << inverseJacobian[i][j] << " ";
                }
                cout << endl;
            }

            // Get Joint Velocities by Multiplying Inverse Jacobian with the Camera Velocities
            double** jointVelocities = matMul(inverseJacobian, vc, 2, 2, 1);

            cout << "jointVelocities: ";
            for(int i = 0; i < 2; i++) {
                cout << jointVelocities[i][0] << " ";
            }
            cout << endl;


            // Limit the velocities to avoid high velocities when reaching near singularities
            jointVelocities[0][0] = jointVelocities[0][0] < -1? -1 : jointVelocities[0][0];
            jointVelocities[1][0] = jointVelocities[1][0] < -1? -1 : jointVelocities[1][0];
            jointVelocities[0][0] = jointVelocities[0][0] > 1? 1 : jointVelocities[0][0];
            jointVelocities[1][0] = jointVelocities[1][0] > 1? 1 : jointVelocities[1][0];

            setVelocities(jointVelocities[0][0], jointVelocities[1][0]);

            double avg_error = 0;

            for(int i = 0; i < 8; i++) {
                avg_error += abs(errors[i][0]);
            }
            avg_error = avg_error/8;

            cout << "Average Error: " << avg_error << endl;
            
            if(avg_error < 1) {
                visualServoingFlag = false;
                setVelocities(0,0);
                ROS_INFO("Average error is less than 1, Stopping Visual Servoing");
                shutdown();
            }
            // Log the data so that we can later plot centers
            logData();

        }

        // Get Circle Centers (Same function from HW 3)
        void getCircleCenters() {
            Mat hsvImage;
            Mat redThresholded, greenThresholded, blueThresholded, purpleThresholded;
            imageWithCenters = image.clone();

            // Convert the image into Hsv color space
            cvtColor(image, hsvImage, COLOR_BGR2HSV);

            // Threshold the images for respective circle colors
            inRange(hsvImage, redLow, redHigh, redThresholded);
            inRange(hsvImage, blueLow, blueHigh, blueThresholded);
            inRange(hsvImage, greenLow, greenHigh, greenThresholded);
            inRange(hsvImage, purpleLow, purpleHigh, purpleThresholded);

            // Get their centers using averaging the pixel values
            redCenter = getCenter(redThresholded);
            blueCenter = getCenter(blueThresholded);
            greenCenter = getCenter(greenThresholded);
            purpleCenter = getCenter(purpleThresholded);

            if(redCenter.x == -1 || redCenter.y == -1 || blueCenter.x == -1 || blueCenter.y == -1 ||
             greenCenter.x == -1 || greenCenter.y == -1 || purpleCenter.x == -1 || purpleCenter.y == -1) {
                 ROS_WARN("Object went out of frame");
             }

            // Draw the centers on the image (if the circle is in the camera view)
            if(redCenter.x != -1) {
                circle(imageWithCenters, redCenter, 1, blackColor, FILLED);
            }
            if(blueCenter.x != -1) {
                circle(imageWithCenters, blueCenter, 1, blackColor, FILLED);
            }
            if(greenCenter.x != -1) {
                circle(imageWithCenters, greenCenter, 1, blackColor, FILLED);
            }
            if(purpleCenter.x != -1) {
                circle(imageWithCenters, purpleCenter, 1, blackColor, FILLED);
            }
            imshow("Image View", imageWithCenters);
        }

        // Log data into a csv file
        void logData() {
            ofstream csvFile;
            csvFile.open("src/vision_based_manipulation/log.csv", ios::app);
            csvFile << currentCenters[0].x << "," << currentCenters[0].y << ",";
            csvFile << currentCenters[1].x << "," << currentCenters[1].y << ",";
            csvFile << currentCenters[2].x << "," << currentCenters[2].y << ",";
            csvFile << currentCenters[3].x << "," << currentCenters[3].y << endl;
            csvFile.close();
        }


        Point getCenter(Mat image) {

            long xAvg = 0, yAvg = 0, counter = 0;

            // Average all the white pixels
            for(int i = 0; i < image.rows; i++) {
                for(int j = 0; j < image.cols; j++) {
                        int val = (int)(image.at<uchar>(i, j));
                        if(val != 0) {
                            xAvg += i;
                            yAvg += j;
                            counter += 1;
                        }
                }
            }

            // If Counter is 0 return (-1, -1) as it is a complete blank image (i.e. no pixel belongs to thresholded colors)
            if(counter == 0) {
                Point center = Point(-1, -1);
                return center;
            }
            Point center = Point(yAvg / counter, xAvg / counter);
            return center;
        }
};

// Publisher function to publish position data to get reference Centers
int positionControl(NodeHandle n, ImageBasedVisualServo ic) {
    pub_q1_pos = n.advertise<std_msgs::Float64>("/vbmbot/joint1_position_controller/command", 10);
    pub_q2_pos = n.advertise<std_msgs::Float64>("/vbmbot/joint2_position_controller/command", 10);
    sleep(1);

    goToPositions(0.1, -0.5);

    referenceCenters[0] = redCenter;
    referenceCenters[1] = blueCenter;
    referenceCenters[2] = greenCenter;
    referenceCenters[3] = purpleCenter;
    
    cout << "Reference Frame : ";
    for(int i = 0; i < 4; i++) {
        cout << referenceCenters[i] << " ";
    }
    cout << endl;

    s = "src/vision_based_manipulation/start.jpg";
    goToPositions(0.7, -1);

    currentCenters[0] = redCenter;
    currentCenters[1] = blueCenter;
    currentCenters[2] = greenCenter;
    currentCenters[3] = purpleCenter;
    
    cout << "Starting Frame : ";
    for(int i = 0; i < 4; i++) {
        cout << currentCenters[i] << " ";
    }
    cout << endl;

    if(switchControllers(n, 
            V_string({"joint1_velocity_controller","joint2_velocity_controller"}),
            V_string({"joint1_position_controller","joint2_position_controller"}) ) == 0) {
        return 0;
    }
        
    return -1;
}


int main(int argc, char** argv) {
    init(argc, argv, "image_processor_node");
    NodeHandle n;
    initializeMatrices();

    // Switch Controller from velocity to position (added so that I don't have to restart gazebo every time I change the code)
    switchControllers(n, 
            V_string({"joint1_position_controller","joint2_position_controller"}),
            V_string({"joint1_velocity_controller","joint2_velocity_controller"}));

    visualServoingFlag = false;

    ImageBasedVisualServo ic(n);
    int res = positionControl(n, ic);
    if(res == 0) {
        jointStateSubscriber = n.subscribe("/vbmbot/joint_states",1, getJointState);
        pub_q1_vel = n.advertise<std_msgs::Float64>("/vbmbot/joint1_velocity_controller/command", 10);
        pub_q2_vel = n.advertise<std_msgs::Float64>("/vbmbot/joint2_velocity_controller/command", 10);        
        sleep(1);

        setVelocities(0, 0);
        sleep(0.5);

        spin();
    } else {
        ROS_ERROR("Publisher Returned '-1'.");
    }
    shutdown();
    return 0;
}