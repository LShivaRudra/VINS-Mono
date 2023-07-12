#include "keyframe.h"
#include "pose_graph.h"
#include <opencv2/opencv.hpp>


#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>


#include <stdio.h>

class TransmitKeyFrame{
public:
	TransmitKeyFrame(){}

    TransmitKeyFrame(KeyFrame& keyframe){
        time_stamp = keyframe.time_stamp;
        index = keyframe.index;
        local_index = keyframe.local_index;
        sequence = keyframe.sequence;

        imgarray[keyframe.image.rows][keyframe.image.cols][keyframe.image.channels()];
        convertCvMatToArray(keyframe.image, imgarray);
        
        convertPoint3fToArray(keyframe.point_3d, point3darr);
        convertPoint2fToArray(keyframe.point_2d_uv, point2DuvArr);
        convertPoint2fToArray(keyframe.point_2d_norm, point2DnormArr);

        point_id = keyframe.point_id;

        convertCvKeypointsToArray(keyframe.keypoints, KeypointArray);
        convertCvKeypointsToArray(keyframe.keypoints_norm, KeypointNormArray);
        convertCvKeypointsToArray(keyframe.window_keypoints, KeypointWindowArray);
        
        //brief desciptor conversion

        has_fast_point = keyframe.has_fast_point;
        has_loop = keyframe.has_loop;
        loop_index = keyframe.loop_index;
        loop_info = keyframe.loop_info;
    }

    double time_stamp; 
	int index;
	int local_index;
    int sequence;
	
    int*** imgarray; //replaces cv::Mat
	
    Eigen::Vector3d vio_T_w_i; 
	Eigen::Matrix3d vio_R_w_i; 
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d origin_vio_R;

    int* point3darr;  //replaces vector<cv::Point3f> point_3d
	int* point2DuvArr; //replaces vector<cv::Point2f> point_2d_uv;
	int* point2DnormArr; //vector<cv::Point2f> point_2d_norm;

	vector<double> point_id;

    
	int** KeypointArray; // replaces vector<cv::KeyPoint> keypoints;
	int** KeypointNormArray; // replaces vector<cv::KeyPoint> keypoints_norm;
	int** KeypointWindowArray; // replaces vector<cv::KeyPoint> window_keypoints;

	vector<BRIEF::bitset> brief_descriptors;
	vector<BRIEF::bitset> window_brief_descriptors;
    
	bool has_fast_point;
	bool has_loop;
	int loop_index;
	Eigen::Matrix<double, 8, 1> loop_info;


    /* =================================POSE GRAPH INFORMATION===========================*/

    // nav_msgs::Path path[10];
    // nav_msgs::Path base_path;
    // CameraPoseVisualization *posegraph_visualization;
    Vector3d t_drift;
    double yaw_drift;
    Matrix3d r_drift;
    Vector3d w_t_vio;
    Matrix3d w_r_vio;


private :
    void convertPoint3fToArray(const std::vector<cv::Point3f>& points, int* arr) {
        for (size_t i = 0; i < points.size(); i++) {
            arr[i * 3] = static_cast<int>(points[i].x);
            arr[i * 3 + 1] = static_cast<int>(points[i].y);
            arr[i * 3 + 2] = static_cast<int>(points[i].z);
        }
    }

    void convertPoint2fToArray(const std::vector<cv::Point2f>& points, int* arr) {
        for (size_t i = 0; i < points.size(); i++) {
            arr[i * 2] = static_cast<int>(points[i].x);
            arr[i * 2 + 1] = static_cast<int>(points[i].y);
        }
    }

    void convertCvMatToArray(const cv::Mat& image, int*** imgarray){
        for (int i = 0; i < image.rows; i++){
            for (int j = 0; j < image.cols; j++){
                cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
                for (int k = 0; k < image.channels(); k++){
                    imgarray[i][j][k] = pixel[k];
                }
            }
        }
    }

    void convertCvKeypointsToArray(const std::vector<cv::KeyPoint>& keypoints, int** keypointArray){
        for (size_t i = 0; i < keypoints.size(); ++i) {
            keypointArray[i][0] = static_cast<int>(keypoints[i].pt.x);
            keypointArray[i][1] = static_cast<int>(keypoints[i].pt.y);
        }
    }
};