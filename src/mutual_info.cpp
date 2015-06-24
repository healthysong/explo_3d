#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <tf/transform_broadcaster.h>



using namespace std;
using namespace std::chrono;

typedef octomap::point3d point3d;
const double PI = 3.1415926;
const double free_prob = 0.3;
// const string OCTOMAP_BINARY_TOPIC = "/octomap_binary";
octomap::OcTree *tree;
bool octomap_flag = 0; // 0 : msg not received

struct Kinect {

    double horizontal_fov;
    double angle_inc;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;

    Kinect(double _width, double _height, double _horizontal_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), max_range(_max_range) {
        angle_inc = horizontal_fov / width;
        for(double i = -width / 2; i < width / 2; ++i) {
            for(double j = -height / 2; j < height / 2; ++j) {
                pitch_yaws.push_back(make_pair(j * angle_inc, i * angle_inc));
            }
        }
    }
}; 
Kinect InitialScan(1000, 1000, 6.0, 15.0);
Kinect kinect(640, 480, 1.047198, 15.0);

double get_free_volume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
            // cout << "node : " << n.getCoordinate() << endl;
    }
    return volume;
}

vector<point3d> cast_init_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    vector<point3d> hits;
    octomap::OcTreeNode *n;

    for(auto p_y : InitialScan.pitch_yaws) {
        double pitch = p_y.first;
        double yaw = p_y.second;
        point3d direction_copy(direction.normalized());
        point3d end;
        direction_copy.rotate_IP(0.0, pitch, yaw);
        if(octree->castRay(position, direction_copy, end, true, InitialScan.max_range)) {
            hits.push_back(end);
        } else {
            direction_copy *= InitialScan.max_range;
            direction_copy += position;
            n = octree->search(direction_copy);
            if (!n)
                continue;
            if (n->getOccupancy() < free_prob )
                continue;
            hits.push_back(direction_copy);
        }
    }        
    return hits;
}

vector<pair<point3d, point3d>> generate_candidates() {
    double R = 3;   // Robot step, in meters.
    double n = 10;

    vector<pair<point3d, point3d>> candidates;
    double z = position.z();                // fixed 
    double pitch = orientation.pitch();     // fixed
    // double yaw = orientation.yaw();         // divided by n
    double x, y;
    // for(double x = position.x() - position_bound * 0.5;
    //     x < position.x() + position_bound * 0.5; x += position_bound / n)
    //     for(double y = position.y() - position_bound * 0.5;
    //         y < position.y() + position_bound * 0.5; y += position_bound / n)
    for(z = position.z() - 1; z <= position.z() + 1; z += 1)
        for(double yaw = 0; yaw < 2 * PI; yaw += PI / n) {
            x = position.x() + R * cos(yaw);
            y = position.y() + R * sin(yaw);
            candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0, pitch, yaw)));
        }
    return candidates;
}


void octomap_callback(const octomap_msgs::Octomap::ConstPtr &octomap_msg) {
        octomap::OcTree *octomap_load = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*octomap_msg));
        octomap_load->setResolution(0.2);
        // octomap::OcTree tree(0.2);
        // octomap::OcTree *octomap_curr = &tree;
        // auto octree_copy = new octomap::OcTree(*octomap_load);
        tree = octomap_load;
        octomap_flag = true;
        cout << "msg got !" << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Info_Exploration_Octomap");
    ros::NodeHandle nh;
    // Initial sub topic
    ros::Subscriber octomap_sub;
    octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, octomap_callback);

    ros::Rate r(1); // 10 hz


    // Initial Scan
    cout << "calculate free volume" << endl;
    double mapEntropy = get_free_volume(tree);
    cout << "Map Entropy : " << mapEntropy << endl;
    point3d eu2dr(1, 0, 0);
    point3d orign(0, 0, 5);
    point3d orient(0, 0, 1);
    cout << "Initial hits at: " << orign  << endl;
    eu2dr.rotate_IP(orient.roll(), orient.pitch(), orient.yaw());
    vector<point3d> Init_hits = cast_init_rays(tree, orign, eu2dr);
    cout << "finished casting initial rays" << endl;
    double before = get_free_volume(tree);


    while (ros::ok())
    {
        if( octomap_flag )
        {
            // Generate Candidates
            vector<pair<point3d, point3d>> candidates = generate_candidates();

            // Calculate Mutual Information
            for(int i = 0; i < candidates.size(); ++i) 
            {
                c = candidates[i];
                n = octomap_curr->search(c.first);

                if (!n)     continue;
                if(n->getOccupancy() > free_prob)       continue;

                high_resolution_clock::time_point t1 = high_resolution_clock::now();
                point3d eu2dr(1, 0, 0);
                eu2dr.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );

                vector<point3d> hits = cast_kinect_rays(octomap_curr, c.first, eu2dr);
                high_resolution_clock::time_point t2 = high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
                cout << "Time on ray cast: " << duration << endl;

                t1 = high_resolution_clock::now();
                MIs[i] = calc_MI(octomap_curr, c.first, hits, before);
                t2 = high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
                cout << "Candidate : " << c.first << "  **  MI: " << MIs[i] << endl;

                logfile << c.first.x() << "\t" << c.first.y() << "\t" << c.first.z() << "\t" << c.second.pitch() << "\t" <<
                c.second.yaw() << endl;
                logfile << MIs[i] << endl;

                // Pick the Best Candidate
                if (MIs[i] > MIs[max_idx])
                {
                    max_idx = i;
                }
            }

            // Send the Robot 


            // Disable flag and wait for next message.
            octomap_flag = false;
        }
        ros::spinOnce();
        r.sleep();
    }
    nh.shutdown();
    // ros::spin();

    return 0;
}