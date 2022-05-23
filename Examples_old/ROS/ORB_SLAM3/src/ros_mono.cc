/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include<geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;


class ImageGrabber
{

public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}
    geometry_msgs::Pose pm;
    geometry_msgs::PoseStamped ps;
    //void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);
    // Write a publisher
    ros::Publisher pub =  nodeHandler.advertise<geometry_msgs::PoseStamped>("out_odom", 100);
    geometry_msgs::PoseStamped  out_odom;
    
    // Change rate => equals rate of subscriber
    ros::Rate ros_rate(10);
    
    while (ros::ok())
    {
        out_odom = igb.ps;
        pub.publish(out_odom);
        ros::spinOnce();
        ros_rate.sleep();
    }
    
    // pub.publish(igb.pm);
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveTrajectoryEuRoC("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}

//Convert Sophus:Se3f to geometry_msgs::Pose
geometry_msgs::Pose sophusToPoseMsg(const Sophus::SE3f& s) {
   geometry_msgs::Pose pose;
   Eigen::Vector3f translation = s.translation();
//    geometry_msgs::Point p;
   float scale_factor = 1.0;
   pose.position.x = translation.x();
   pose.position.y = translation.y();
   pose.position.z = translation.z();

   
   Eigen::Quaternionf quarternion = s.unit_quaternion();
   pose.orientation.w = quarternion.w();
   pose.orientation.x = quarternion.x();
   pose.orientation.y = quarternion.y();
   pose.orientation.z = quarternion.z();
   
   return pose;
 }



void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Eigen::Matrix3d R;
    // R << 0.9999522, -0.0013325,  0.0096853, 0.0015675,  0.9997035, -0.0242990, -0.0096501,  0.0243131,  0.9996578;
    //Eigen::Quaternion<Sophus::SE3f::Scalar> Rc0w(0.012155108061, 0.004834683620, 0.000725122336, 0.999914173007);
    // Eigen::Quaternion<Sophus::SE3f::Scalar> Rc0w(1,0, 0, 0);
    Eigen::Quaternion<Sophus::SE3f::Scalar> Rbc(0.5,-0.5, 0.5, -0.5); // YAW -90 and roll -90
    // Sophus::Vector3f tbc(0,0,0);
    Sophus::Vector3f tbc(0,0.05,0);
    // Sophus::SE3f Tbc = Sophus::SE3f(Rbc,tbc).inverse();
    Sophus::SE3f Tbc(Rbc,tbc);
    
    cout<<"Rbc"<< endl;
    cout<< Tbc.rotationMatrix()<<endl;
    cout<<"Rcb"<< endl;
    cout<< Tbc.inverse().rotationMatrix()<<endl;
    // Tc0w.translation() = tc0w ;
    // Tc0w.rotationMatrix() = R; 
    // Sophus::SE3f Tcb = Tbc.inverse();

    Sophus::SE3f Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    Sophus::SE3f Twc = Tcw.inverse();
    cout<<"tcw: " << Tcw.translation()   <<endl;
    cout<<"twc: " << Twc.translation()   <<endl;

    //Working
    // Sophus::SE3f Tbw = Tbc*Tcw;
    // Sophus::Vector3f tbw = Tbc.rotationMatrix()*Tcw.translation();
    // Sophus::SE3f Tbw2(Tbw.unit_quaternion(), tbw);
    // cout<<"tbw: " << tbw  <<endl;
    // cout<<"tbw2: " << Tbw2.translation()   <<endl;
    // Sophus::SE3f Twb = Tbw2.inverse();
    // cout<<"twb: " << Twb.translation()   <<endl;
    //  Checking: Twb =Tbc*Twb;


    Sophus::SE3f Twb = Tbc*Twc;
    // Twb =Twb*Tbc;
    cout<<"twb NEW: " << Twb.translation()   <<endl;

    // Eigen::Quaternion<Sophus::SE3f::Scalar> R_supp(0.7071068, 0, 0, 0.7071068 ); // YAW -90
    // Sophus::Vector3f t_supp(0,0,0);
    // Sophus::SE3f T_supp(R_supp,t_supp);
    // cout<<"R_supp"<< endl;
    // cout<< T_supp.rotationMatrix()<<endl;
    // Twb = T_supp*Twb;
    


    // geometry_msgs::Pose pm = sophusToPoseMsg(Twc);
    geometry_msgs::Pose pm = sophusToPoseMsg(Twb);
    this->pm = pm;
    this->ps.header.stamp = cv_ptr->header.stamp;
    this->ps.header.frame_id = cv_ptr->header.frame_id;
    this->ps.header.seq = cv_ptr->header.seq;

    this->ps.pose = pm;
    
}


