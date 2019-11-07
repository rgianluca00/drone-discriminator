// TODO includes
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "vision_msgs/Detection2DArray.h"

using namespace Eigen;

void objCallback(vision_msgs::Detection2DArrayConstPtr &msg)
{
    ROS_INFO("Test objCallback");
}

/* Funzioni per la sincronizzazione iniziale dei nodi. */
void publishReadyStatus()
{
    position_estimation_pkg::trackingStatus status;
    status.header.stamp = ros::Time::now();
    status.trackingSystem_id = nodeName; 
    status.isReady = true;
    trackingStatusPub.publish(status);
    ROS_INFO_STREAM("Discriminator published ready status");  
}

void waitForTeam() 
{ 
    ros::Rate waitRate(1);
    while (!teamReady)
    {
        publishReadyStatus();
        ros::spinOnce();
        waitRate.sleep();
    } 
}


int main(int argc, char** argv)
{
    //initStandardVar();
    ros::init(argc, argv, "discriminator_node");
    ros::NodeHandle node;
    ros::Rate rate(1);
    
    //initFromLaunch(node);

    // TODO topic
    ros::Subscriber detect_sub = node.subscribe("/objects", 10, &objCallback);
    
    // Pubblicazione della ROI associata all'ultimo frame
    ros::Publisher roi_pub = node.advertise<std_msgs::Float32MultiArray>("/Stampa/Roi", 1);
    ros::Publisher status_pub = node.advertise<position_estimation_pkg::trackingStatus>("/team_status", 10);
    // Sincronizzazione del nodo: il ciclo principale partirà quando tutti i nodi saranno pronti;
    publishReadyStatus();
    waitForTeam();
    
    typedef Tracker<4, 2> GNNTracker;
    GNNTracker tracker;
    
    while(ros::ok())
    {
        
    tracker.track();
    
    ros::spinOnce();
    rate.sleep();
    }

    return 0;

}
