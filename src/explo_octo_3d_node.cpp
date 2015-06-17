#include <iostream>
#include <fstream>
#include <chrono>


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

class PointCloudPub {
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloud;
public:
    PointCloudPub(ros::NodeHandle _nh) : nh(_nh), msg(new PointCloud) {
        msg->header.frame_id = pointcloud_frame_id;
        msg->height = 1;
        msg->width = 0;
        pub = nh.advertise<PointCloud>("virtualScans", 1);
    }

    void insert_point3d(double x, double y, double z) {
        msg->points.push_back(PointType(x, y, z));
        msg->width++;
    }

    void clear() {
        msg->points.clear();
        msg->width = 0;
    }

    void publish() const {
        msg->header.stamp = ros::Time::now().toNSec() / 1e3;
        pub.publish(msg);
        cout << "published : " << msg->width << " points" << endl;
        ros::spinOnce();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    const string pointcloud_frame_id = "/map";
    PointCloud::Ptr msg;
};

class OctomapExploration {
public:
    OctomapExploration(ros::NodeHandle _nh) : nh(_nh), pointcloud_pub(_nh), logfile("log.txt") {
        position = point3d(0, 0, 1);
        orientation = point3d(1, 0, 0);
        octomap_sub = nh.subscribe<octomap_msgs::Octomap>(OCTOMAP_BINARY_TOPIC, 1,
                                                          &OctomapExploration::octomap_callback, this);
    }

    ~OctomapExploration() {
        logfile.close();
    }

    vector<pair<point3d, point3d>> generate_candidates() const {
        double position_bound = 0.5;
        double orientation_bound = kinect.horizontal_fov;
        double n = 5;

        vector<pair<point3d, point3d>> candidates;
        double z = position.z();
        double pitch = orientation.pitch();
        double yaw = orientation.yaw();
        for(double x = position.x() - position_bound * 0.5;
            x < position.x() + position_bound * 0.5; x += position_bound / n)
            for(double y = position.y() - position_bound * 0.5;
                y < position.y() + position_bound * 0.5; y += position_bound / n)
                    for(double yaw = 0; yaw < 2 * PI; yaw += PI / n) {
                        candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0, pitch, yaw)));
                    }

        return candidates;
    }

    vector<point3d> cast_kinect_rays(const octomap::OcTree *octree, const point3d &position,
                                     const point3d &direction) const {
        vector<point3d> hits;
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
                hits.push_back(direction_copy);
            }
        }
        return hits;
    }

    vector<point3d> cast_init_rays(const octomap::OcTree *octree, const point3d &position,
                                     const point3d &direction) const {
        vector<point3d> hits;
        // cout << "d_x : " << direction.x() << " d_y : " << direction.y() << " d_z : " 
        //   << direction.z() << endl;
        for(auto p_y : InitialScan.pitch_yaws) {
            double pitch = p_y.first;
            double yaw = p_y.second;
            point3d direction_copy(direction.normalized());
            point3d end;
            direction_copy.rotate_IP(0.0, pitch, yaw);
            // hits.push_back(direction_copy);
            if(octree->castRay(position, direction_copy, end, true, InitialScan.max_range)) {
                hits.push_back(end);
            } else {
                direction_copy *= InitialScan.max_range;
                direction_copy += position;
                hits.push_back(direction_copy);
            }
        }
        return hits;
    }

    double get_free_volume(const octomap::OcTree *octree) const {
        double volume = 0;
        for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
            // if(n->getOccupancy() < free_prob)
            if(octree->isNodeOccupied(*n))
                volume += pow(n.getSize(), 3);
        }
        return volume;
    }

    double calc_MI(const octomap::OcTree *octree, const point3d &position, const vector<point3d> &hits, const double before) const {
        auto octree_copy = new octomap::OcTree(*octree);
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
        // octomap::OcTree *octomap_curr;
        auto octomap_curr = new octomap::OcTree(*octomap_load);

        vector<pair<point3d, point3d>> candidates = generate_candidates();
        vector<double> MIs(candidates.size());
        auto c = candidates[0];
        octomap::OcTreeNode *n;

        float cr2, cp2, cy2, sr2, sp2, sy2;
        
        // Prepare a arrow for sensor representation
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.ns = "sensor_kinect";
        marker.id = 0;
        marker.type = 0;
        tf::Quaternion sensor_qua;

        // Initial Scan

        cout << "Initial hits at: " << c.first  << endl;
        point3d eu2dr(1, 0, 0);
        point3d orign(0, 0, 1);
        eu2dr.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
        vector<point3d> Init_hits = cast_init_rays(octomap_load, orign, eu2dr);
        // cout << "here" << endl;
        cout << "finished casting initial rays" << endl;
        for(auto h : Init_hits) {
            // cout << "inserting ray .." << h << endl;
            octomap_curr->insertRay(orign, h, InitialScan.max_range);
        }
        cout << "finished inserting rays" << endl;
        double before = get_free_volume(octomap_curr);

        pointcloud_pub.clear();
        for (auto h : Init_hits) {
            pointcloud_pub.insert_point3d(h.x()/6.0, h.y()/6.0, h.z()/6.0);
        }
        pointcloud_pub.publish();

#pragma omp parallel for
        for(int i = 0; i < candidates.size(); ++i) {
            c = candidates[i];
            n = octomap_curr->search(c.first, octomap_curr->getTreeDepth());

            if(n->getOccupancy() > free_prob)
            {
                continue;
            }

            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            point3d eu2dr(1, 0, 0);
            eu2dr.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            vector<point3d> hits = cast_kinect_rays(octomap_curr, c.first, eu2dr);
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            // cout << "Time on ray cast: " << duration << endl;

            t1 = high_resolution_clock::now();
            MIs[i] = calc_MI(octomap_curr, c.first, hits, before);
            t2 = high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            // cout << "Time on counting volume: " << duration << endl;

            cout << "Candidate: " << c.first.x() << " " << c.first.y() << " " << c.first.z() << " " <<
            c.second.pitch() << " " << c.second.yaw() << " MI:   " << MIs[i] << endl;

            logfile << c.first.x() << "\t" << c.first.y() << "\t" << c.first.z() << "\t" << c.second.pitch() << "\t" <<
            c.second.yaw() << endl;
            logfile << MIs[i] << endl;

            pointcloud_pub.clear();
            for (auto h : hits) {
                pointcloud_pub.insert_point3d(h.x()/6.0, h.y()/6.0, h.z()/6.0);
            }
            pointcloud_pub.publish();


            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = c.first.x();
            marker.pose.position.y = c.first.y();
            marker.pose.position.z = c.first.z();


            cr2 = cos(c.second.roll()*0.5);
            cp2 = cos(c.second.pitch()*0.5);
            cy2 = cos(c.second.yaw()*0.5);

            sr2 = -sin(c.second.roll()*0.5);
            sp2 = -sin(c.second.pitch()*0.5);
            sy2 = sin(c.second.yaw()*0.5);

            marker.pose.orientation.w = cr2*cp2*cy2 + sr2*sp2*sy2;
            marker.pose.orientation.x = sr2*cp2*cy2 - cr2*sp2*sy2;
            marker.pose.orientation.y = cr2*sp2*cy2 + sr2*cp2*sy2;
            marker.pose.orientation.z = cr2*cp2*sy2 - sr2*sp2*cy2;
   
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 1.0;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
// 
            marker.lifetime = ros::Duration();
            marker_pub.publish(marker);

        }
        nh.shutdown();
    }

private:
    ros::NodeHandle nh;
    const string OCTOMAP_BINARY_TOPIC = "/octomap_binary";
    ros::Subscriber octomap_sub;
    PointCloudPub pointcloud_pub;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("CenterOfSensor", 1);


    point3d position;
    point3d orientation;

    const double free_prob = 0.3;

    ofstream logfile;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "octomap_MI");
    ros::NodeHandle nh;
    OctomapExploration octomap_exploration(nh);
    ros::spin();

    return 0;
}