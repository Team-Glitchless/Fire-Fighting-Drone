#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <cmath>


using namespace octomap;
void print_query_info(point3d query, OcTreeNode* node) {
    if (node != NULL) 
    {
        std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
    }
    else 
        std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    
}
ros::Publisher goal_pub;

class rrt {
    public:
    /*rrt(int total_nodes, double thres) { 
        total = total_nodes;
        threshold = thres;
        std::cout << "Hemlo" << std::endl;
    }*/
    /*rrt() {
        this -> getGoal();
        
    }*/
    void getviews(point3d origin, std::vector<point3d> rays[10]){
        octre->computeRay(origin, point3d(max_x, max_y, origin.z()), rays[0]);
        octre->computeRay(origin, point3d(0, max_y, origin.z()), rays[1]);
        octre->computeRay(origin, point3d(max_x, 0, origin.z()), rays[2]);
        octre->computeRay(origin, point3d(-max_x, 0, origin.z()), rays[3]);
        octre->computeRay(origin, point3d(0, -max_y, origin.z()), rays[4]);
        octre->computeRay(origin, point3d(-max_x, max_y, origin.z()), rays[5]);
        octre->computeRay(origin, point3d(max_x, -max_y, origin.z()), rays[6]);
        octre->computeRay(origin, point3d(-max_x, -max_y, origin.z()), rays[7]);
        octre->computeRay(origin, point3d(0, max_y, max_z), rays[8]);
        octre->computeRay(origin, point3d(0, max_y, -max_z), rays[9]);      
    }
    void getDist(){
        for (int i = 0; i < 10; i++) {
            int dit = 1;
            for (int j = 0; j < ray[i].size(); j++){
                OcTreeNode* nd = octre->search(ray[i][j]);
                dit += 1;
                if (nd != NULL)
                    if (nd->getOccupancy() > threshold)
                        dist[i] = dit;
                        //std::cout << "Obstacle :" << ray[i][j] << std::endl;
                        //break; 
            }
        }
    }
    double vectorDist(point3d a, point3d b) {
        return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2) + pow(a.z() - b.z(), 2));
    }
    double infoGain(point3d g) {
        std::vector<point3d> rys[10];
        getviews(g, rys);
        double unknown = 0;
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < rys[i].size(); j++){
                OcTreeNode* nd = octre->search(rys[i][j]);
                //print_query_info(rys[i][j], nd);
                if (nd == NULL)
                     unknown += 1;
            }
        }
        return unknown;
    }
    //std::vector<double> getGoal() {
    void getGoal(){
        this -> getviews(currentPos, ray);
        this -> getDist();
        //std::vector<double> entropy(10, 0);
        std::vector<double> entropy;
        std::cout << "Entropies :";
        for (int i = 0; i < 10 ; i++) {
            int temp = infoGain(ray[i][dist[i] - 1]) / vectorDist(currentPos, ray[i][dist[i] - 1]);
            std::cout << temp << " ";
            entropy.push_back(temp);
        }
        std::cout << std::endl;
        //int maxEntropy = std::max_element(entropy.begin(),entropy.end()) - entropy.begin();
        auto maxEntropy = std::max_element(entropy.begin(),entropy.end()) - entropy.begin();
        //if (this -> checkGoal()) {
            //std::vector <double> res = getGoal();
            geometry_msgs::Point ress;
            ress.x = currentPos.x() - ray[maxEntropy][dist[maxEntropy] - 1].x();
            ress.y = currentPos.y() - ray[maxEntropy][dist[maxEntropy] - 1].y();
            ress.z = currentPos.z() - ray[maxEntropy][dist[maxEntropy] - 1].z();
            goal_pub.publish(ress);
            std::cout << "Published Goal :" << ress << std::endl;
            //std::cout<<"publishes\n";
        /*}
        else {
            std::cout << "Goal Achieved";
        }*/
        /*return {currentPos.x() - ray[maxEntropy][dist[maxEntropy] - 1].x(),
        currentPos.y() - ray[maxEntropy][dist[maxEntropy] - 1].y(),
        currentPos.z() - ray[maxEntropy][dist[maxEntropy] - 1].z()};*/
    }
    void getUnknown() {
        point3d_list ct;
        octre->getUnknownLeafCenters(ct, point3d(min_x, min_y, min_z), point3d(max_x, max_y, max_z));
        unknown = ct.size();
    }
    bool checkGoal(){
        getUnknown();
        double searched = total - unknown;
        double percent = searched / total;
        std::cout << "Percent known: " << percent << std::endl;
        if (percent < 80) return false;
        return true; 
    }
    void currPos(double x, double y, double z) {
        currentPos = point3d(x, y, z);
        std::cout << "Current pos: " << currentPos << std::endl;
    }
    void setBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max) {
        min_x = -5; min_y = -15; min_z = 0; max_x = 5; max_y = 15; max_z = 5;
    }
    void getOctree(octomap::OcTree* otre) {
        octre = otre;
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octre->getMetricMax(max_x, max_y, max_z);
        octre->getMetricMin(min_x, min_y, min_z);
        this -> setBounds(min_x, min_y, min_z, max_x, max_y, max_z);
        threshold = octre->getOccupancyThres();
        total = octre->calcNumNodes();
        //std::cout<<total<<std::endl;
        treeCenter = octre->begin_leafs().getCoordinate();
        std::cout << "Tree Center" << treeCenter << std::endl;
        std::cout << "Max bounds" << max_x << " " << max_y << " " << max_z << std::endl;
        std::cout << "Min bounds" << min_x << " " << min_y << " " << min_z << std::endl;
    }
    private:
        octomap::OcTree* octre;
        double min_x, min_y, min_z, max_x, max_y, max_z;
        point3d currentPos, treeCenter;
        double threshold;
        int total, unknown;
        const int step_size = 4;
        std::vector<point3d> ray[10];
        int dist[10];

};


void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, rrt* planner) {
    octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    //std::cout << tree_oct->calcNumNodes() << std::end;
    //std::vector <point3d> ray;
    /*point3d start(0,0,0), end (1,1,1);
    if (tree_oct->computeRay(start, end, ray)) {
        for (std::size_t i = 0; i < ray.size(); ++i) {
            print_query_info(ray[i], tree_oct->search(ray[i]));
            
        }  
        std::cout << "hemlo";      
    }*/
    planner -> getOctree(tree_oct);
    planner -> getGoal();
    
    //octomap::OcTree::leaf_iterator root = tree_oct->begin_leafs();
    //std::cout << "Center coordinate :" <<root.getCoordinate() << std::endl;
    //std::cout << "Max Bounds: x = " << max_x << " y = " << max_y << " z = " << max_z << std::endl;
    //std::cout << "Min Bounds: x = " << min_x << " y = " << min_y << " z = " << min_z << std::endl;
    //std::cout << "Probability Threshold " << tree_oct->getOccupancyThres();
}

void get_pose(const geometry_msgs::PoseStamped::ConstPtr& msg, rrt* planner) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    planner -> currPos(x,y,z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octree_maker");
	ros::NodeHandle n;
	//ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
    rrt planner;
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, boost::bind(&get_pose, _1, &planner));
    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner));    
    
    goal_pub = n.advertise<geometry_msgs::Point>( "next_goal", 10);
    ros::spin();
    return 0;
}