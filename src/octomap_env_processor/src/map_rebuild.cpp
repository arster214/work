#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_set>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

// 哈希函数，用于将 Key 放入 unordered_set
struct KeyHash {
    size_t operator()(const OcTreeKey& key) const {
        return key.k[0] + 1447 * key.k[1] + 345637 * key.k[2];
    }
};

// ==========================================
// 连通域聚类去噪 (Cluster Filtering)
// ==========================================
// 原理：找出所有粘连在一起的体素块。
// 如果这一坨体素的总数量小于 min_cluster_size，就认为是悬浮噪点删除。
// 这样可以保留薄墙（虽然薄，但连成一片，数量多），删除大块噪点（虽然厚，但孤立，数量少）。
void removeSmallClusters(OcTree* tree, int min_cluster_size) {
    unordered_set<OcTreeKey, KeyHash> visited;
    vector<OcTreeKey> keys_to_delete;
    
    // 遍历所有叶子节点
    for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        if (!tree->isNodeOccupied(*it)) continue;
        
        OcTreeKey current_key = it.getKey();
        if (visited.count(current_key)) continue; // 已经处理过这个块了

        // BFS 搜索连通域
        vector<OcTreeKey> current_cluster;
        queue<OcTreeKey> q;
        
        q.push(current_key);
        visited.insert(current_key);
        current_cluster.push_back(current_key);

        while (!q.empty()) {
            OcTreeKey k = q.front();
            q.pop();

            // 检查 26 邻域
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;

                        OcTreeKey n_key = k;
                        n_key[0] += dx; n_key[1] += dy; n_key[2] += dz;

                        // 搜索邻居
                        OcTreeNode* node = tree->search(n_key);
                        if (node && tree->isNodeOccupied(node)) {
                            if (visited.find(n_key) == visited.end()) {
                                visited.insert(n_key);
                                current_cluster.push_back(n_key);
                                q.push(n_key);
                            }
                        }
                    }
                }
            }
        }

        // 核心判断：如果这坨东西太小，就删掉
        if (current_cluster.size() < min_cluster_size) {
            keys_to_delete.insert(keys_to_delete.end(), current_cluster.begin(), current_cluster.end());
        }
    }

    // 执行删除
    for (const auto& key : keys_to_delete) {
        tree->deleteNode(key);
    }
    cout << "Cluster Filter: Removed " << keys_to_delete.size() << " nodes (isolated clusters)." << endl;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <input_map.bt> <output_map.bt> [min_z] [max_z]" << endl;
        return -1;
    }

    string input_file = argv[1];
    string output_file = argv[2];
    
    // 默认高度参数，如果命令行有传参则覆盖
    double min_z = 0.45; 
    double max_z = 2.2;
    
    if (argc >= 5) {
        min_z = stod(argv[3]);
        max_z = stod(argv[4]);
    }

    cout << "Loading map..." << endl;
    OcTree* tree = new OcTree(input_file);
    if (!tree) {
        cerr << "Error: Could not read file." << endl;
        return -1;
    }

    tree->expand(); // 展开树结构，这对邻域搜索很重要

    // ================= 1. 高度过滤 =================
    
    vector<OcTreeKey> height_delete_keys;
    cout << "Filtering by height (" << min_z << " < z < " << max_z << ")..." << endl;
    
    for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        if (tree->isNodeOccupied(*it)) {
            if (it.getZ() < min_z || it.getZ() > max_z) {
                height_delete_keys.push_back(it.getKey());
            }
        }
    }
    for (const auto& key : height_delete_keys) tree->deleteNode(key);
    cout << "Height Filter: Removed " << height_delete_keys.size() << " nodes." << endl;

    // ================= X 方向滤波 =================
    // 模仿高度滤波的逻辑：切掉 X 超出 [min_x, max_x] 的体素
    // 请根据你的环境调整下面的阈值
    double min_x = -1.0;
    double max_x = 1.0;

    vector<OcTreeKey> x_delete_keys;
    cout << "Filtering by X (" << min_x << " < x < " << max_x << ")..." << endl;
    for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        if (tree->isNodeOccupied(*it)) {
            if (it.getX() < min_x || it.getX() > max_x) {
                x_delete_keys.push_back(it.getKey());
            }
        }
    }
    for (const auto& key : x_delete_keys) tree->deleteNode(key);
    cout << "X Filter: Removed " << x_delete_keys.size() << " nodes." << endl;

    tree->updateInnerOccupancy();

    // ================= 2. 聚类去噪 (解决大块噪点问题) =================
    // 这里的 50 是阈值。
    // 含义：如果有少于 50 个方块连在一起，就认为它是噪点。
    // 如果你的地图分辨率很高（比如 0.02），这个数可能要设成 200 甚至 500。
    // 如果地图分辨率低（比如 0.1），50 已经很大了。
    cout << "Running Cluster Filtering..." << endl;
    removeSmallClusters(tree, 50);

    // ================= 保存 =================
    tree->updateInnerOccupancy();
    tree->prune(); 
    
    cout << "Saving to: " << output_file << endl;
    tree->writeBinary(output_file);
    delete tree;
    return 0;
}
