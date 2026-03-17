#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>

class TabletopSegmentation {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_collision_;
    
    // Parameters
    std::string frame_id_;
    double z_min_, z_max_;
    double plane_distance_thresh_;
    double cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;
    bool publish_moveit_collsion_;
    std::string bt_file_path_;

public:
    TabletopSegmentation() : nh_("~") {
        nh_.param<std::string>("frame_id", frame_id_, "base_link_underpan");
        nh_.param<std::string>("bt_file_path", bt_file_path_, "");
        nh_.param("z_min", z_min_, 0.5);
        nh_.param("z_max", z_max_, 1.5);
        nh_.param("plane_distance_thresh", plane_distance_thresh_, 0.02);
        nh_.param("cluster_tolerance", cluster_tolerance_, 0.05); // 5cm
        nh_.param("min_cluster_size", min_cluster_size_, 50);
        nh_.param("max_cluster_size", max_cluster_size_, 25000);
        nh_.param("publish_moveit_collision", publish_moveit_collsion_, true);

        // LATCHED publishers because we might process once and stay alive
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 1, true);
        pub_collision_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10, true);

        if (!bt_file_path_.empty()) {
            ROS_INFO("Loading Octomap from .bt file: %s", bt_file_path_.c_str());
            processBtFile();
        } else {
            ROS_INFO("No .bt file provided. Will subscribe to /combined_cloud_in (live mode).");
            sub_cloud_ = nh_.subscribe("/combined_cloud_in", 1, &TabletopSegmentation::cloudCallback, this);
        }
    }

    void processBtFile() {
        octomap::OcTree* tree = new octomap::OcTree(0.02);
        if (!tree->readBinary(bt_file_path_)) {
            ROS_ERROR("Failed to read .bt file!");
            return;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Iterate through leaf nodes and copy valid points
        for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(),
                                           end=tree->end_leafs(); it!=end; ++it) {
            if (tree->isNodeOccupied(*it)) {
                cloud->points.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
            }
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        
        ROS_INFO("Loaded %lu occupied voxels from octomap.", cloud->points.size());
        delete tree;
        
        processCloud(cloud);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        processCloud(cloud);
    }

    void processCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        if (cloud->empty()) return;

        // 1. Voxel Grid (for faster processing)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_filtered);

        // 2. PassThrough (Z axis) to isolate the table region
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*cloud_passthrough);
        
        if (cloud_passthrough->empty()) {
            ROS_WARN("No points left after PassThrough filter (Z: %.2f~%.2f).", z_min_, z_max_);
            return;
        }

        // 3. RANSAC to find Table Plane
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(plane_distance_thresh_);
        seg.setInputCloud(cloud_passthrough);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            ROS_WARN("RANSAC could not find a table plane in the point cloud.");
            return;
        }

        // 4. Extract Objects (Everything Above the Table)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_passthrough);
        extract.setIndices(inliers);
        extract.setNegative(true); // true = Get objects NOT on the plane
        extract.filter(*cloud_objects);

        if (cloud_objects->empty()) {
            ROS_WARN("Found table, but no objects on it.");
            return;
        }

        // 5. Euclidean Clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_objects);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_objects);
        ec.extract(cluster_indices);

        ROS_INFO("Found %zu object clusters on the table.", cluster_indices.size());

        // 6. Create Bounding Boxes and Publish
        visualization_msgs::MarkerArray marker_array;
        
        // Clean out previous markers first: we can send a deleteAll marker or let RViz handle it
        visualization_msgs::Marker clear_marker;
        clear_marker.action = 3; // DELETEALL action for ROS Kinetic+
        clear_marker.header.frame_id = frame_id_;
        clear_marker.ns = "tabletop_obstacles";
        marker_array.markers.push_back(clear_marker);

        int id = 0;
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int index : indices.indices) {
                cluster->points.push_back(cloud_objects->points[index]);
            }

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            
            double dx = max_pt.x - min_pt.x;
            double dy = max_pt.y - min_pt.y;
            double dz = max_pt.z - min_pt.z;
            
            // Inflate minimally for safety (2cm on each side)
            dx += 0.04; dy += 0.04; dz += 0.04;

            double cx = min_pt.x + dx/2.0;
            double cy = min_pt.y + dy/2.0;
            double cz = min_pt.z + dz/2.0;

            // Generate RViz Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "tabletop_obstacles";
            marker.id = id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = dx;
            marker.scale.y = dy;
            marker.scale.z = dz;
            marker.color.r = 1.0;
            marker.color.g = 0.4;
            marker.color.b = 0.0;
            marker.color.a = 0.8; 
            // no lifetime specified means indefinitely until deleted
            marker_array.markers.push_back(marker);

            // Generate MoveIt Collision Object
            if (publish_moveit_collsion_) {
                moveit_msgs::CollisionObject col_obj;
                col_obj.header.frame_id = frame_id_;
                col_obj.id = "dynamic_obstacle_" + std::to_string(id);
                col_obj.operation = moveit_msgs::CollisionObject::ADD;

                shape_msgs::SolidPrimitive box;
                box.type = shape_msgs::SolidPrimitive::BOX;
                box.dimensions = {dx, dy, dz};
                col_obj.primitives.push_back(box);

                geometry_msgs::Pose box_pose;
                box_pose.position.x = cx;
                box_pose.position.y = cy;
                box_pose.position.z = cz;
                box_pose.orientation.w = 1.0;
                col_obj.primitive_poses.push_back(box_pose);

                pub_collision_.publish(col_obj);
            }
            id++;
        }

        if(!marker_array.markers.empty()) {
            pub_markers_.publish(marker_array);
            ROS_INFO("Successfully published %d obstacle markers and CollisionObjects.", id);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tabletop_segmentation_node");
    TabletopSegmentation node;
    ros::spin();
    return 0;
}
