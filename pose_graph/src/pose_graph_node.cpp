#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "transmission.cpp"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10


// Socket Programming
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#define PORT 12345
#define BUFFER_SIZE 4096

using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
// queue<TransmitKeyFrame> extkf_buf;

queue<KeyFrame> keyframe_buf;


std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_key_odometrys;
ros::Publisher pub_vio_path;

ros::Publisher keyframePublisher;
nav_msgs::Path no_loop_path;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

TransmitKeyFrame *exttkf;
bool entered = false;

void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    //ROS_INFO("image_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void imu_forward_callback(const nav_msgs::Odometry::ConstPtr &forward_msg)
{
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio *  vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;        

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
    }
}

void relo_relative_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    //printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    posegraph.updateKeyFrameLoop(index, loop_info);

}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = ros::Duration();

    //static int key_odometrys_id = 0;
    key_odometrys.id = 0; //key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys.publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path.publish(no_loop_path);
    }
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}


KeyFrame ConvertToKeyFrame(TransmitKeyFrame& transmitKeyFrame) {
    // Convert imgarray back to cv::Mat
    // cv::Mat image;
    // int*** imgarray = transmitKeyFrame.imgarray;
    // int rows = transmitKeyFrame.img_dim1;
    // int cols = transmitKeyFrame.img_dim2;
    // int channels = transmitKeyFrame.img_dim3;
    // transmitKeyFrame.convertArrayToCvMat(imgarray, rows, cols, channels, image);

    // // Convert point3darr back to std::vector<cv::Point3f>
    // float* point3darr = transmitKeyFrame.point3darr;
    // size_t point3darr_num_elements = transmitKeyFrame.point3darr_num_elements;
    // std::vector<cv::Point3f> point_3d;
    // transmitKeyFrame.convertArrayToPoint3f(point3darr, point3darr_num_elements, point_3d);

    // // Convert point2DuvArr back to std::vector<cv::Point2f>
    // float* point2DuvArr = transmitKeyFrame.point2DuvArr;
    // size_t point2DuvArr_num_elements = transmitKeyFrame.point2DuvArr_num_elements;
    // std::vector<cv::Point2f> point_2d_uv;
    // transmitKeyFrame.convertArrayToPoint2f(point2DuvArr, point2DuvArr_num_elements, point_2d_uv);

    // // Convert KeypointArray back to std::vector<cv::KeyPoint>
    // TransmitKeyFrame::KeypointArrayGeneral* keypointArray = transmitKeyFrame.KeypointArray;
    // size_t KeypointArray_num_elements = transmitKeyFrame.KeypointArray_num_elements;
    // std::vector<cv::KeyPoint> keypoints;
    // transmitKeyFrame.convertArrayToCvKeypoints(keypointArray, KeypointArray_num_elements, keypoints);

    // Now create a KeyFrame object and populate it with the converted data
    KeyFrame keyframe;
    keyframe.time_stamp = transmitKeyFrame.time_stamp;
    keyframe.index = transmitKeyFrame.index;
    keyframe.local_index = transmitKeyFrame.local_index;
    keyframe.sequence = transmitKeyFrame.sequence;
    // keyframe.image = image;
    // keyframe.point_3d = point_3d;
    // keyframe.point_2d_uv = point_2d_uv;
    // keyframe.keypoints = keypoints;
    // ... populate the rest of the KeyFrame object with the remaining data

    return keyframe;
}

void extkf_callback(const std_msgs::String::ConstPtr& msg){
    entered = true;
    m_process.lock();
    TransmitKeyFrame *exttkf_ptr = new TransmitKeyFrame();
    std::memcpy(exttkf_ptr, msg->data.c_str(), sizeof(TransmitKeyFrame));
    // KeyFrame *external_kf_ptr = new KeyFrame();

    try{
        KeyFrame *external_kf_ptr = new KeyFrame();
        // *external_kf_ptr = exttkf->ToKeyFrame(*exttkf);
        *external_kf_ptr = ConvertToKeyFrame(*exttkf_ptr);
        std::cout << "converted tfk to kf with index: " << external_kf_ptr->index << std::endl;
        // delete external_kf;
        // delete exttkf;
    } catch(const std::exception& e){
        std::cerr << "Error processing external KeyFrame: " << e.what() << std::endl;
    }


    /* Uncommenting this kills the pose_graph_node for no discernable reason
    delete exttkf_ptr;     
    */  
    m_process.unlock();
    return;
}

void process()
{
    if (!LOOP_CLOSURE)
        return;
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                

                // KeyFrame keyframe_copy(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                //                    point_3d, point_2d_uv, point_2d_normal, point_id, sequence);

                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                
                // keyframe_buffer.push(*keyframe);
                // TransmitKeyFrame(keyframe_buffer.front());
                // keyframe_buffer.pop();
                frame_index++;
                last_t = T;
                
                // std::cout << "Going to convert Keyframe into custom object" << std::endl;
                // KeyFrame keyframe_copy = *keyframe;
                TransmitKeyFrame keyframe_dummy(*keyframe);
                // std::cout << "Going to serialize Keyframe" << std::endl;
                std::string KFString(reinterpret_cast<const char*>(&keyframe_dummy), sizeof(keyframe_dummy));
                std_msgs::String serializedKF;
                serializedKF.data = KFString;
                // std::cout << "Going to publish keyframe" << std::endl;
                keyframePublisher.publish(serializedKF);
                // std::cout << "sucessfully published keyframe with time_stamp: " << keyframe->time_stamp << std::endl;
                std::cout << "sucessfully published keyframe with index: " << keyframe->index << std::endl;
                // std::cout << "sucessfully published keyframe with local_index: " << keyframe->local_index << std::endl;
                // std::cout << "sucessfully published keyframe with sequence: " << keyframe->sequence << std::endl;

                // std::cout<<"Img dims1 in modkf: "<<keyframe->image.rows<<std::endl;
                // std::cout<<"Img dims2 in modkf: "<<keyframe->image.cols<<std::endl;
                // std::cout<<"Img dims3 in modkf: "<<keyframe->image.channels()<<std::endl;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    if (!LOOP_CLOSURE)
        return;
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            // printf("program shutting down...\n");
            // ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void externalkfprocess(){
    // KeyFrame *externalkf = new KeyFrame();
    while(true){
        // std::cout << "in new thread" << std::endl;
        if (entered == true){
            
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_graph_node");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);

    // read param
    n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
    n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    n.getParam("skip_cnt", SKIP_CNT);
    n.getParam("skip_dis", SKIP_DIS);
    std::string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);


    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    if (LOOP_CLOSURE)
    {
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];
        std::string pkg_path = ros::package::getPath("pose_graph");
        string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph.loadVocabulary(vocabulary_file);

        BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["image_topic"] >> IMAGE_TOPIC;        
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
        fsSettings.release();

        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();

    keyframePublisher = n.advertise<std_msgs::String>("/serialized_keyframe", 10);

    ros::Subscriber sub_imu_forward = n.subscribe("/vins_estimator/imu_propagate", 2000, imu_forward_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
    ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    ros::Subscriber sub_relo_relative_pose = n.subscribe("/vins_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);

    ros::Subscriber sub_external_kf = n.subscribe("/mod_kf_string_topic", 10, extkf_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys", 1000);
    pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
    pub_match_points = n.advertise<sensor_msgs::PointCloud>("match_points", 100);

    std::thread measurement_process;
    std::thread keyboard_command_process;
    // std::thread extKeyFrame_process;

    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);
    // extKeyFrame_process = std::thread(externalkfprocess);

    ros::spin();

    return 0;
}
