#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Bool.h>

// === 新增：PCL 滤波库 ===
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

// === 新增：OpenMP 多线程库 ===
#include <omp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class DepthToCloud {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_depth_;
    ros::Subscriber sub_info_;
    ros::Publisher pub_cloud_;

    double fx_, fy_, cx_, cy_;
    bool has_intrinsics_;
    double depth_scale_; 

    // === 新增：滤波参数 ===
    bool use_voxel_filter_;
    double voxel_leaf_size_;
    bool use_sor_filter_;
    int sor_mean_k_;
    double sor_stddev_mul_;
    
    // === 新增：PassThrough 空间裁减 ===
    bool use_pass_filter_;
    double pass_z_min_, pass_z_max_;
    double pass_x_min_, pass_x_max_;
    double pass_y_min_, pass_y_max_; // Added Y parameters

    // === 控制开关 ===
    bool processing_enabled_;
    ros::Subscriber sub_enable_;
    ros::Subscriber sub_enable_global_;

public:
    DepthToCloud() : it_(nh_), has_intrinsics_(false), depth_scale_(0.001), processing_enabled_(true) {
        ros::NodeHandle pnh("~");
        pnh.param("depth_scale", depth_scale_, 0.001);

        // === 1. 加载手动内参 ===
        double fx, fy, cx, cy;
        if (pnh.getParam("fx", fx) && pnh.getParam("fy", fy) && 
            pnh.getParam("cx", cx) && pnh.getParam("cy", cy)) {
            fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
            has_intrinsics_ = true;
        }

        // === 2. 加载滤波参数 (可在 launch 文件修改) ===
        // 降采样：对于避障，2cm~3cm 的精度足够了，能把数据量减少 90%
        pnh.param("use_voxel_filter", use_voxel_filter_, true);
        pnh.param("voxel_leaf_size", voxel_leaf_size_, 0.04); // 3cm

        // 统计滤波：去噪
        pnh.param("use_sor_filter", use_sor_filter_, true);
        pnh.param("sor_mean_k", sor_mean_k_, 35);
        pnh.param("sor_stddev_mul", sor_stddev_mul_, 0.75);

        // 空间裁减：只保留感兴趣区域的点，大幅减少后续负担
        pnh.param("use_pass_filter", use_pass_filter_, true);
        pnh.param("pass_z_min", pass_z_min_, 0.1); 
        pnh.param("pass_z_max", pass_z_max_, 3.0);
        
        pnh.param("pass_x_min", pass_x_min_, -10.0);
        pnh.param("pass_x_max", pass_x_max_, 10.0);
        pnh.param("pass_y_min", pass_y_min_, -10.0);
        pnh.param("pass_y_max", pass_y_max_, 10.0);

        sub_depth_ = it_.subscribe("depth/image_raw", 1, &DepthToCloud::depthCallback, this);
        sub_info_ = nh_.subscribe("depth/camera_info", 1, &DepthToCloud::infoCallback, this);
        
        // 订阅使能控制 (全局与局部都支持)
        sub_enable_global_ = nh_.subscribe("/enable_point_process", 1, &DepthToCloud::enableCallback, this);
        sub_enable_ = pnh.subscribe("enable", 1, &DepthToCloud::enableCallback, this);

        pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("depth/points_processed", 1);
        
        ROS_INFO("Depth processor started. Voxel: %s (%.2f m), SOR: %s, Pass: %s (%.2f ~ %.2f m)",
            use_voxel_filter_?"ON":"OFF", voxel_leaf_size_, use_sor_filter_?"ON":"OFF",
            use_pass_filter_?"ON":"OFF", pass_z_min_, pass_z_max_);
    }

    void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
        processing_enabled_ = msg->data;
        if (processing_enabled_) {
            ROS_INFO("Point processing ENABLED");
        } else {
            ROS_INFO("Point processing DISABLED");
        }
    }

    void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg) {
        if (has_intrinsics_) return;
        fx_ = info_msg->K[0]; fy_ = info_msg->K[4];
        cx_ = info_msg->K[2]; cy_ = info_msg->K[5];
        has_intrinsics_ = true;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
        if (!processing_enabled_) return;
        if (!has_intrinsics_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        PointCloud::Ptr cloud(new PointCloud);
        cloud->header.frame_id = depth_msg->header.frame_id;
        pcl_conversions::toPCL(depth_msg->header.stamp, cloud->header.stamp);
        
        // 预分配内存
        cloud->width = depth_msg->width;
        cloud->height = depth_msg->height;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        const float bad_point = std::numeric_limits<float>::quiet_NaN();
        const unsigned short* depth_data = (const unsigned short*)cv_ptr->image.data;

        // === 3. 使用 OpenMP 并行加速循环 (CPU 占用率优化) ===
        // 这能让循环速度快 4-8 倍
        #pragma omp parallel for
        for (int i = 0; i < cloud->points.size(); ++i) {
            // 通过索引计算 u, v
            int u = i % depth_msg->width;
            int v = i / depth_msg->width;

            unsigned short depth_raw = depth_data[i];
            PointT& pt = cloud->points[i];

            // 基础阈值过滤：太近(<0.2m)或太远(>2.5m)的点不要
            // D435 超过 2.5m 精度很差，容易产生噪音墙。
            // 这里的硬编码限制可以去掉，改用下面的 PassThrough Filter，或者保留作为第一道防线。
            if (depth_raw == 0) { 
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }

            float z = (float)depth_raw * depth_scale_;
            float x = (u - cx_) * z / fx_;
            float y = (v - cy_) * z / fy_;

            // 如果启用了 PassThrough，这里可以做提前判断
            if (use_pass_filter_) {
               if (z < pass_z_min_ || z > pass_z_max_) {
                    pt.x = pt.y = pt.z = bad_point;
                    continue;
               }
               if (x < pass_x_min_ || x > pass_x_max_) {
                    pt.x = pt.y = pt.z = bad_point;
                    continue;
               }
               if (y < pass_y_min_ || y > pass_y_max_) {
                    pt.x = pt.y = pt.z = bad_point;
                    continue;
               }
            }

            pt.x = x;
            pt.y = y;
            pt.z = z;
        }

        // === 4. PCL 后处理 (核心优化) ===
        PointCloud::Ptr cloud_filtered(new PointCloud);
        
        // A. 降采样 (VoxelGrid)
        if (use_voxel_filter_) {
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            sor.filter(*cloud_filtered);
            // 将过滤后的云设为下一步的输入
            cloud = cloud_filtered; // 智能指针交换，低开销
            // 注意：VoxelGrid 后点云变成无序的 (Unorganized)，width变小，height=1
        }

        // B. 统计去噪 (SOR)
        if (use_sor_filter_ && !cloud->empty()) {
            PointCloud::Ptr cloud_sor(new PointCloud);
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_mul_);
            sor.filter(*cloud_sor);
            cloud = cloud_sor;
        }

        // C. 空间裁减 (PassThrough)
        if (use_pass_filter_ && !cloud->empty()) {
            PointCloud::Ptr cloud_pass(new PointCloud);
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(pass_z_min_, pass_z_max_);
            pass.filter(*cloud_pass);

            // 仅保留通过裁减的点
            cloud = cloud_pass;
        }

        // 发布
        if (!cloud->empty()) {
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud, output_msg);
            pub_cloud_.publish(output_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_to_cloud_node");
    DepthToCloud dtc;
    ros::spin(); // 建议使用 ros::AsyncSpinner spinner(4); spinner.start(); 配合 nodelet 更好，但这里简单起见用 spin
    return 0;
}