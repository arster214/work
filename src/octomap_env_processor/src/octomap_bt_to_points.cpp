#include <iostream>
#include <fstream>
#include <string>

#include <octomap/OcTree.h>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.bt> <output.xyz> [free|occupied|all]\n";
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];
    const std::string mode = argc > 3 ? argv[3] : "occupied";

    octomap::OcTree octree(0.1);
    if (!octree.readBinary(input_path)) {
        std::cerr << "Failed to read octomap binary tree from: " << input_path << "\n";
        return 2;
    }

    std::ofstream ofs(output_path);
    if (!ofs) {
        std::cerr << "Failed to open output file: " << output_path << "\n";
        return 4;
    }

    std::size_t count = 0;
    for (auto it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it) {
        const bool occupied = octree.isNodeOccupied(*it);
        const bool keep = mode == "all" || (mode == "occupied" && occupied) || (mode == "free" && !occupied);
        if (!keep) {
            continue;
        }

        ofs << it.getX() << ' ' << it.getY() << ' ' << it.getZ() << ' ' << it.getSize() << '\n';
        ++count;
    }

    std::cout << "Wrote " << count << " points to " << output_path << "\n";
    return 0;
}
