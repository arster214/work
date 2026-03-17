#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

int main(int argc, char** argv) {
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " <input_points.xyz> <output_map.bt> <resolution>" << endl;
        return -1;
    }

    string input_file = argv[1];
    string output_file = argv[2];
    double resolution = stod(argv[3]);

    cout << "Creating OctoMap with resolution: " << resolution << endl;
    OcTree tree(resolution);

    ifstream infile(input_file);
    if (!infile.is_open()) {
        cerr << "Error: Could not open input file " << input_file << endl;
        return -1;
    }

    double x, y, z;
    string line;
    long count = 0;
    
    while (infile >> x >> y >> z) {
        getline(infile, line); 
        point3d endpoint((float)x, (float)y, (float)z);
        tree.updateNode(endpoint, true); 
        count++;
    }

    tree.updateInnerOccupancy();
    tree.prune();
    
    cout << "Inserted " << count << " points. Saving to " << output_file << endl;
    tree.writeBinary(output_file);

    return 0;
}
