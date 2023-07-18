#include "keyframe.h"
#include "pose_graph.h"
#include <opencv2/opencv.hpp>


#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "ThirdParty/DVision/BRIEF.h"

#include <stdio.h>

class TransmitKeyFrame{
public:
	TransmitKeyFrame(){}
    
    ~TransmitKeyFrame(){
        if (imgarray != nullptr) {
            for (int i = 0; i < image_rows; ++i) {
                for (int j = 0; j < image_cols; ++j) {
                    delete[] imgarray[i][j];
                }
                delete[] imgarray[i];
            }
            delete[] imgarray;
        }
        delete[] point3darr;
        delete[] point2DuvArr;
        delete[] point2DnormArr;
        delete[] KeypointArray;
        delete[] KeypointNormArray;
        delete[] KeypointWindowArray;
    }

    TransmitKeyFrame(KeyFrame& keyframe){
        time_stamp = keyframe.time_stamp;
        index = keyframe.index;
        local_index = keyframe.local_index;
        sequence = keyframe.sequence;
        
        image_rows = keyframe.image.rows;
        image_cols = keyframe.image.cols;
        image_chans = keyframe.image.channels();
        int*** imgarray = new int**[image_rows];
        for (int i = 0; i < image_rows; ++i) {
            imgarray[i] = new int*[image_cols];
            for (int j = 0; j < image_cols; ++j) {
                imgarray[i][j] = new int[image_chans];
            }
        }
        convertCvMatToArray(keyframe.image, imgarray);
        
        //float** KeypointArray = new float*[];

        vio_T_w_i = keyframe.vio_T_w_i;
        vio_R_w_i = keyframe.vio_R_w_i;
        T_w_i = keyframe.T_w_i;
        R_w_i = keyframe.R_w_i;
        origin_vio_T = keyframe.origin_vio_T;
        origin_vio_R = keyframe.origin_vio_R;

        point3darr = new float[keyframe.point_3d.size() * 3];
        point2DuvArr = new float[keyframe.point_2d_uv.size() * 2];
        point2DnormArr = new float[keyframe.point_2d_norm.size() * 2];


        convertPoint3fToArray(keyframe.point_3d, point3darr);
        convertPoint2fToArray(keyframe.point_2d_uv, point2DuvArr);
        convertPoint2fToArray(keyframe.point_2d_norm, point2DnormArr);

        point_id = keyframe.point_id;

        kp_vec_size = keyframe.keypoints.size();
        kp_norm_vec_size = keyframe.keypoints.size();
        win_kp_vec_size = keyframe.keypoints.size();

        KeypointArray = new KeypointArrayGeneral[kp_vec_size];
        KeypointNormArray = new KeypointArrayGeneral[kp_norm_vec_size];
        KeypointWindowArray = new KeypointArrayGeneral[win_kp_vec_size]; 

        convertCvKeypointsToArray(keyframe.keypoints, KeypointArray);
        convertCvKeypointsToArray(keyframe.keypoints_norm, KeypointNormArray);
        convertCvKeypointsToArray(keyframe.window_keypoints, KeypointWindowArray);
        
        //brief desciptor conversion
        brief_descriptors = keyframe.brief_descriptors;
        window_brief_descriptors = keyframe.window_brief_descriptors;

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
    //int*** thumbnail;
	
    Eigen::Vector3d vio_T_w_i; 
	Eigen::Matrix3d vio_R_w_i; 
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d origin_vio_R;

    float* point3darr;  //replaces vector<cv::Point3f> point_3d
	float* point2DuvArr; //replaces vector<cv::Point2f> point_2d_uv;
	float* point2DnormArr; //vector<cv::Point2f> point_2d_norm;

	vector<double> point_id;

    
	// int** KeypointArray; // replaces vector<cv::KeyPoint> keypoints;
	// int** KeypointNormArray; // replaces vector<cv::KeyPoint> keypoints_norm;
	// int** KeypointWindowArray; // replaces vector<cv::KeyPoint> window_keypoints;
    struct KeypointArrayGeneral{
        float x;
        float y;
        float size;
        float angle;
        float response;
        int octave;
        int class_id;
    };

    int kp_vec_size;
    int kp_norm_vec_size;
    int win_kp_vec_size;

    KeypointArrayGeneral* KeypointArray; //to replace vector<cv::KeyPoint> keypoints;
    KeypointArrayGeneral* KeypointNormArray; //to replace vector<cv::KeyPoint> keypoints_norm;
    KeypointArrayGeneral* KeypointWindowArray; //to replace vector<cv::KeyPoint> window_keypoints;

	vector<BRIEF::bitset> brief_descriptors;
	vector<BRIEF::bitset> window_brief_descriptors;
    
	bool has_fast_point;
	bool has_loop;
	int loop_index;
	Eigen::Matrix<double, 8, 1> loop_info;

private :
    int image_rows;
    int image_cols;
    int image_chans;

    void convertPoint3fToArray(const std::vector<cv::Point3f>& points, float*& arr) {
        for (size_t i = 0; i < points.size(); i++) {
            arr[i * 3] = static_cast<int>(points[i].x);
            arr[i * 3 + 1] = static_cast<int>(points[i].y);
            arr[i * 3 + 2] = static_cast<int>(points[i].z);
        }
    }

    void convertPoint2fToArray(const std::vector<cv::Point2f>& points, float*& arr) {
        for (size_t i = 0; i < points.size(); i++) {
            arr[i * 2] = static_cast<int>(points[i].x);
            arr[i * 2 + 1] = static_cast<int>(points[i].y);
        }
    }

    void convertCvMatToArray(const cv::Mat& image, int***& imgarray){
        for (int i = 0; i < image.rows; i++){
            for (int j = 0; j < image.cols; j++){
                cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
                for (int k = 0; k < image.channels(); k++){
                    imgarray[i][j][k] = pixel[k];
                }
            }
        }
    }

    void convertCvKeypointsToArray(const std::vector<cv::KeyPoint>& keypoints, KeypointArrayGeneral*& keypointarr){
        // for (size_t i = 0; i < keypoints.size(); ++i) {
        //     keypointArray[i][0] = static_cast<int>(keypoints[i].pt.x);
        //     keypointArray[i][1] = static_cast<int>(keypoints[i].pt.y);
        // }
        for(size_t i = 0; i < keypoints.size(); ++i){
            keypointarr[i].x = keypoints[i].pt.x;
            keypointarr[i].y = keypoints[i].pt.y;
            keypointarr[i].size = keypoints[i].size;
            keypointarr[i].angle = keypoints[i].angle;
            keypointarr[i].response = keypoints[i].response;
            keypointarr[i].octave = keypoints[i].octave;
            keypointarr[i].class_id = keypoints[i].class_id;
        }

    }

    /*
    Converting TransmitKeyFrame back to KeyFrame object
    */

    cv::Mat convertArrayToCvMat(int*** imgarray, int rows, int cols, int chans) {
        cv::Mat image(rows, cols, CV_8UC(chans));
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                cv::Vec3b pixel;
                for (int k = 0; k < chans; k++) {
                    pixel[k] = imgarray[i][j][k];
                }
                image.at<cv::Vec3b>(i, j) = pixel;
            }
        }

        return image;
    }
    
    std::vector<cv::Point3f> convertArrayToPoint3f(float* arr, int size) {
        std::vector<cv::Point3f> points;

        for (int i = 0; i < size; i += 3) {
            cv::Point3f point(arr[i], arr[i + 1], arr[i + 2]);
            points.push_back(point);
        }

        return points;
    }

    // Convert float array to vector<cv::Point2f>
    std::vector<cv::Point2f> convertArrayToPoint2f(float* arr, int size) {
        std::vector<cv::Point2f> points;

        for (int i = 0; i < size; i += 2) {
            cv::Point2f point(arr[i], arr[i + 1]);
            points.push_back(point);
        }

        return points;
    }

    // Convert KeypointArrayGeneral array to vector<cv::KeyPoint>
    std::vector<cv::KeyPoint> convertArrayToCvKeypoints(TransmitKeyFrame::KeypointArrayGeneral* keypointarr, int size) {
        std::vector<cv::KeyPoint> keypoints;

        for (int i = 0; i < size; i++) {
            cv::KeyPoint keypoint(keypointarr[i].x, keypointarr[i].y, keypointarr[i].size, keypointarr[i].angle, keypointarr[i].response, keypointarr[i].octave, keypointarr[i].class_id);
            keypoints.push_back(keypoint);
        }

        return keypoints;
    }
};