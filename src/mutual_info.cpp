#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

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
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double free_prob = 0.3;
octomap::OcTree *tree;
octomap::OcTree *tree_kinect;
bool octomap_flag = 0; // 0 : msg not received
   
point3d position;
point3d orientation;



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
    }
    return volume;
}

void get_free_points(const octomap::OcTree *octree, PointCloud::Ptr pclPtr) {
    
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
        {
            pclPtr->points.push_back(pcl::PointXYZ(n.getX()/5.0, n.getY()/5.0, n.getZ()/5.0));
            pclPtr->width++;
        }

    }
    return;
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


vector<point3d> cast_kinect_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    vector<point3d> hits;
    octomap::OcTreeNode *n;
    // cout << "d_x : " << direction.normalized().x() << " d_y : " << direction.normalized().y() << " d_z : " 
    //   << direction.normalized().z() << endl;
    for(auto p_y : kinect.pitch_yaws) {
        double pitch = p_y.first;
        double yaw = p_y.second;
        point3d direction_copy(direction.normalized());
        point3d end;
        direction_copy.rotate_IP(0.0, pitch, yaw);
        if(octree->castRay(position, direction_copy, end, true, kinect.max_range)) {
            hits.push_back(end);
        } else {
            direction_copy *= kinect.max_range;
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

vector<pair<point3d, point3d>> generate_candidates(point3d position) {
    double R = 3;   // Robot step, in meters.
    double n = 10;

    vector<pair<point3d, point3d>> candidates;
    double z = position.z();                // fixed 
    double pitch = 0;     // fixed
    double x, y;

    for(z = position.z() - 1; z <= position.z() + 1; z += 1)
        for(double yaw = 0; yaw < 2 * PI; yaw += PI / n) {
            x = position.x() + R * cos(yaw);
            y = position.y() + R * sin(yaw);
            candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0, pitch, yaw)));
        }
    return candidates;
}

double calc_MI(const octomap::OcTree *octree, const point3d &position, const vector<point3d> &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);
    // octomap::OcTree *octree_copy = dynamic_cast<octomap::OcTree *>(octree);
    // cout << "th iter " << endl;
    for(const auto h : hits) {
        octree_copy->insertRay(position, h, kinect.max_range);
    }
    octree_copy->updateInnerOccupancy();
    double after = get_free_volume(octree_copy);
    delete octree_copy;
    return after - before;
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

void kinect_callback( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    cout << "kinect callback" << endl;
    // Convert to PCL data type
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud2_msg, pcl_pc2);
    PointCloud::Ptr cloud (new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    auto octree_copy = new octomap::OcTree(0.2);
    // point3d origin(0, 0, 5);
    
    cout << "height : " << cloud->height << "  , width :" << cloud->width << endl;

    // for (int i = 1; i < cloud->height; i++)
        for (int j = 1; j< cloud->width; j++)
        {
            // cout << cloud->at(j) << endl;
            octree_copy->insertRay(position, 
                point3d(cloud->at(j).x*5, cloud->at(j).y*5, cloud->at(j).z*5), kinect.max_range);
        }

    
    cout << "fake kinect point cloud, resulting a entropy : " << get_free_volume(octree_copy) << endl;
    delete octree_copy;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Info_Exploration_Octomap");
    ros::NodeHandle nh;
    // Initial sub topic
    ros::Subscriber octomap_sub;
    ros::Subscriber kinect_sub;
    octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, octomap_callback);
    kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/virtual_Scans", 1, kinect_callback);
    
    ros::Publisher Map_pcl_pub;
    ros::Publisher Free_pcl_pub;
    ros::Publisher VScan_pcl_pub;
    Map_pcl_pub = nh.advertise<PointCloud>("Current_Map", 1);
    VScan_pcl_pub = nh.advertise<PointCloud>("virtual_Scans", 1);
    Free_pcl_pub = nh.advertise<PointCloud>("Free_points", 1);

    // PointCloud::Ptr map_pcl;
    PointCloud::Ptr map_pcl (new PointCloud);
    PointCloud::Ptr free_pcl (new PointCloud);
    PointCloud::Ptr vsn_pcl (new PointCloud);

    map_pcl->header.frame_id = "/map";
    map_pcl->height = 1;
    map_pcl->width = 0;

    free_pcl->header.frame_id = "/map";
    free_pcl->height = 1;
    free_pcl->width = 0;

    vsn_pcl->header.frame_id = "/map";
    vsn_pcl->height = 1;
    vsn_pcl->width = 0;
    

    ros::Rate r(1); // 10 hz

    // wait until the prior map comes in
    while(!octomap_flag) 
    {
        ros::spinOnce();
    }

    // Initialize parameters
    int max_idx = 0;

    position = point3d(0, 0, 5);
    orientation = point3d(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree tree1(0.2);
    octomap::OcTree* cur_tree;
    cur_tree = &tree1;
    // tree_kinect = 

    // Initial Scan
    cout << "calculate free volume" << endl;
    double mapEntropy = get_free_volume(tree);
    cout << "Original Map Entropy : " << mapEntropy << endl;
    point3d eu2dr(1, 0, 0);
    point3d orign(0, 0, 5);
    point3d orient(0, 0, 1);
    cout << "Initial hits at: " << orign  << endl;
    eu2dr.rotate_IP(orient.roll(), orient.pitch(), orient.yaw());
    vector<point3d> Init_hits = cast_init_rays(tree, orign, eu2dr);
    cout << "finished casting initial rays" << endl;
    double before = get_free_volume(tree);

    cout << "Free Initial : " << before << endl;

    // publish the point cloud : current map
    for (auto h : Init_hits) {
        map_pcl->points.push_back(pcl::PointXYZ(h.x()/5.0, h.y()/5.0, h.z()/5.0));
        map_pcl->width++;
        cur_tree->insertRay(orign, h, InitialScan.max_range);
    }

    cout << "CurMap  Entropy : " << get_free_volume(cur_tree) << endl;

    map_pcl->header.stamp = ros::Time::now().toNSec() / 1e3;
    cout << "pcl size : " << Init_hits.size() << endl;
    Map_pcl_pub.publish(map_pcl);

    get_free_points(cur_tree, free_pcl);
    // CurrentPcl_pub.publish();
    // map_pcl->points.push_back (PointType(1.0, 2.0, 3.0));

    while (ros::ok())
    {
        // update the current map
        map_pcl->header.stamp = ros::Time::now().toNSec() / 1e3;
        Map_pcl_pub.publish(map_pcl);
        free_pcl->header.stamp = ros::Time::now().toNSec() / 1e3;
        Free_pcl_pub.publish(free_pcl);


        if( 1 )
        {
            // Generate Candidates
            vector<pair<point3d, point3d>> candidates = generate_candidates(position);
            vector<double> MIs(candidates.size());
            max_idx = 0;

            // Calculate Mutual Information
            for(int i = 0; i < candidates.size(); ++i) 
            {
                auto c = candidates[i];
                n = tree->search(c.first);

                if (!n)     continue;
                if(n->getOccupancy() > free_prob)       continue;

                high_resolution_clock::time_point t1 = high_resolution_clock::now();
                point3d eu2dr(1, 0, 0);
                eu2dr.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );

                vector<point3d> hits = cast_kinect_rays(cur_tree, c.first, eu2dr);
                high_resolution_clock::time_point t2 = high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
                cout << "Time on ray cast: " << duration << endl;

                t1 = high_resolution_clock::now();
                MIs[i] = calc_MI(cur_tree, c.first, hits, before);
                t2 = high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
                cout << "Candidate : " << c.first << "  **  MI: " << MIs[i] << endl;

                // logfile << c.first.x() << "\t" << c.first.y() << "\t" << c.first.z() << "\t" << c.second.pitch() << "\t" <<
                // c.second.yaw() << endl;
                // logfile << MIs[i] << endl;

                // Pick the Best Candidate
                if (MIs[i] > MIs[max_idx])
                {
                    max_idx = i;
                    // publish the virtual scan once
                    vsn_pcl->clear();
                    for (auto h : hits) {
                        vsn_pcl->points.push_back(pcl::PointXYZ(h.x()/5.0, h.y()/5.0, h.z()/5.0));
                        vsn_pcl->width++;
                        // cur_tree->insertRay(c.first, h, kinect.max_range);
                    }
                    VScan_pcl_pub.publish(vsn_pcl);
                }

            }

            // Send the Robot 
            position = candidates[max_idx].first;
            cout << "move the robot to : " << position << endl;

            // Simulate the ray cast at best candidate
            cout << "Update the octomap..." << endl;
            eu2dr.rotate_IP(0, 0, candidates[max_idx].second.yaw());
            // hits.clear();
            vector<point3d> hits = cast_kinect_rays(tree, position, eu2dr);
            for (auto h : hits) {
                cur_tree->insertRay(position, h, kinect.max_range);
            }
            cout << "CurMap  Entropy : " << get_free_volume(cur_tree) << endl;
            // cout << "finished casting initial rays" << endl;




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