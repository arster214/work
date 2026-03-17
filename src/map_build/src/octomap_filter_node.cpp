#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_set>
#include <queue>
#include <map>
#include <octomap_msgs/GetOctomap.h>

using namespace std;
using namespace octomap;

// 哈希函数，用于将 Key 放入 unordered_set
struct KeyHash {
    size_t operator()(const OcTreeKey& key) const {
        return key.k[0] + 1447 * key.k[1] + 345637 * key.k[2];
    }
};

class OctomapFilterNode {
public:
    OctomapFilterNode() {
        ros::NodeHandle nh;
        ros::NodeHandle nh_private("~");

        // Load parameters
        nh_private.param("min_z", min_z_, 0.05); // Default to a reasonable height
        nh_private.param("max_z", max_z_, 2.0);
        nh_private.param("min_x", min_x_, -2.0);
        nh_private.param("max_x", max_x_, 2.0);
        nh_private.param("min_y", min_y_, -2.0);
        nh_private.param("max_y", max_y_, 2.0);
        nh_private.param("min_cluster_size", min_cluster_size_, 50);

        sub_ = nh.subscribe("octomap_binary", 1, &OctomapFilterNode::octomapCallback, this);
        pub_ = nh.advertise<octomap_msgs::Octomap>("octomap_filtered", 1, true);
        pub_vis_ = nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, true);
        
        // Advertise service for octomap_saver to consume
        srv_ = nh.advertiseService("octomap_binary_service", &OctomapFilterNode::octomapServiceCallback, this);

        ROS_INFO("Octomap Filter Node Started.");
        ROS_INFO("Params: min_z: %.2f, max_z: %.2f, min_x: %.2f, max_x: %.2f, min_y: %.2f, max_y: %.2f, min_cluster: %d", 
                 min_z_, max_z_, min_x_, max_x_, min_y_, max_y_, min_cluster_size_);
    }

    // Cache the latest filtered map
    octomap_msgs::Octomap latest_msg_;
    bool has_map_ = false;

    bool octomapServiceCallback(octomap_msgs::GetOctomap::Request& req,
                                octomap_msgs::GetOctomap::Response& res) {
        if (!has_map_) {
            ROS_WARN("Service called but no map received yet.");
            return false;
        }
        res.map = latest_msg_;
        return true;
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(*msg);
        if (!tree) {
            ROS_WARN("Failed to convert octomap message to tree");
            return;
        }

        OcTree* ocTree = dynamic_cast<OcTree*>(tree);
        if (!ocTree) {
            ROS_WARN("The tree is not an OcTree");
            delete tree;
            return;
        }

        // Apply filters
        filterByHeight(ocTree);
        filterByX(ocTree);
        filterByY(ocTree);
        
        // 恢复碎块剔除，可以消除空中的离散团块
        removeSmallClusters(ocTree, min_cluster_size_);

        ocTree->updateInnerOccupancy();
        ocTree->prune();

        // Publish filtered map
        octomap_msgs::Octomap out_msg;
        out_msg.header = msg->header;
        out_msg.id = ocTree->getTreeType();
        out_msg.resolution = ocTree->getResolution();
        out_msg.binary = true;

        if (octomap_msgs::binaryMapToMsg(*ocTree, out_msg)) {
            pub_.publish(out_msg);
            // Cache for service
            latest_msg_ = out_msg;
            has_map_ = true;
        } else {
            ROS_ERROR("Failed to serialize filtered octomap");
        }

        publishMarkers(ocTree, msg->header);

        delete tree;
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub_vis_;
    ros::ServiceServer srv_;
    
    double min_z_, max_z_;
    double min_x_, max_x_;
    double min_y_, max_y_;
    int min_cluster_size_;

    void publishMarkers(OcTree* tree, const std_msgs::Header& header) {
        visualization_msgs::MarkerArray occupiedNodesVis;
        
        // 1. 发送 DELETEALL marker 清除上一帧的残留
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "filtered_map";
        clear_marker.id = 0; 
        clear_marker.action = 3; // visualization_msgs::Marker::DELETEALL
        occupiedNodesVis.markers.push_back(clear_marker);

        // Group points by size (depth) for efficient rendering using CUBE_LIST
        std::map<double, std::vector<geometry_msgs::Point>> size_to_points;
        std::map<double, std::vector<std_msgs::ColorRGBA>> size_to_colors;

        for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
            if (tree->isNodeOccupied(*it)) {
                double size = it.getSize();
                geometry_msgs::Point p;
                p.x = it.getX();
                p.y = it.getY();
                p.z = it.getZ();
                
                size_to_points[size].push_back(p);
                
                // Color by height (Z) for better visualization
                double min_h_vis = min_z_;
                double max_h_vis = max_z_;
                double h = (p.z - min_h_vis) / (max_h_vis - min_h_vis);
                h = std::max(0.0, std::min(1.0, h));
                
                std_msgs::ColorRGBA color;
                color.r = 1.0 - h; 
                color.g = h;       
                color.b = 0.0;
                color.a = 1.0;
                
                size_to_colors[size].push_back(color);
            }
        }

        int id = 0;
        for (auto const& [size, points] : size_to_points) {
            visualization_msgs::Marker m;
            m.header = header;
            m.ns = "filtered_map";
            m.id = ++id; // 从 1 开始，因为 0 号可能被用于其它用途（虽然 DELETEALL 不占 ID）
            m.type = visualization_msgs::Marker::CUBE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = size;
            m.scale.y = size;
            m.scale.z = size;
            
            m.points = points;
            m.colors = size_to_colors[size];

            occupiedNodesVis.markers.push_back(m);
        }
        
        pub_vis_.publish(occupiedNodesVis);
    }

    void filterByHeight(OcTree* tree) {
        vector<OcTreeKey> keys_to_delete;
        int count = 0;
        for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
            if (tree->isNodeOccupied(*it)) {
                if (it.getZ() < min_z_ || it.getZ() > max_z_) {
                    keys_to_delete.push_back(it.getKey());
                }
            }
        }
        for (const auto& key : keys_to_delete) {
            tree->deleteNode(key);
            count++;
        }
        //if (count > 0) ROS_INFO_THROTTLE(1.0, "Height Filter: Removed %d nodes.", count);
    }

    void filterByX(OcTree* tree) {
        vector<OcTreeKey> keys_to_delete;
        int count = 0;
        for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
            if (tree->isNodeOccupied(*it)) {
                if (it.getX() < min_x_ || it.getX() > max_x_) {
                    keys_to_delete.push_back(it.getKey());
                }
            }
        }
        for (const auto& key : keys_to_delete) {
            tree->deleteNode(key);
            count++;
        }
        //if (count > 0) ROS_INFO_THROTTLE(1.0, "X Filter: Removed %d nodes.", count);
    }

    void filterByY(OcTree* tree) {
        vector<OcTreeKey> keys_to_delete;
        int count = 0;
        for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
            if (tree->isNodeOccupied(*it)) {
                if (it.getY() < min_y_ || it.getY() > max_y_) {
                    keys_to_delete.push_back(it.getKey());
                }
            }
        }
        for (const auto& key : keys_to_delete) {
            tree->deleteNode(key);
            count++;
        }
        //if (count > 0) ROS_INFO_THROTTLE(1.0, "Y Filter: Removed %d nodes.", count);
    }

    void removeSmallClusters(OcTree* tree, int min_cluster_size) {
        unordered_set<OcTreeKey, KeyHash> visited;
        vector<OcTreeKey> keys_to_delete;
        
        tree->expand(); 

        int total_removed = 0;

        for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
            if (!tree->isNodeOccupied(*it)) continue;
            
            OcTreeKey current_key = it.getKey();
            if (visited.count(current_key)) continue;

            vector<OcTreeKey> current_cluster;
            queue<OcTreeKey> q;
            
            q.push(current_key);
            visited.insert(current_key);
            current_cluster.push_back(current_key);

            while (!q.empty()) {
                OcTreeKey k = q.front();
                q.pop();

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            if (dx == 0 && dy == 0 && dz == 0) continue;

                            OcTreeKey n_key = k;
                            n_key[0] += dx; n_key[1] += dy; n_key[2] += dz;

                            if (visited.count(n_key)) continue; 

                            OcTreeNode* node = tree->search(n_key);
                            if (node && tree->isNodeOccupied(node)) {
                                visited.insert(n_key);
                                current_cluster.push_back(n_key);
                                q.push(n_key);
                            }
                        }
                    }
                }
                // Safety break for huge clusters to improve performance
                if (current_cluster.size() >= min_cluster_size) break;
            }

            if (current_cluster.size() < min_cluster_size) {
                keys_to_delete.insert(keys_to_delete.end(), current_cluster.begin(), current_cluster.end());
            }
        }

        for (const auto& key : keys_to_delete) {
            tree->deleteNode(key);
            total_removed++;
        }
        //if (total_removed > 0) ROS_INFO_THROTTLE(1.0, "Cluster Filter: Removed %d nodes.", total_removed);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_filter_node");
    OctomapFilterNode node;
    ros::spin();
    return 0;
}
