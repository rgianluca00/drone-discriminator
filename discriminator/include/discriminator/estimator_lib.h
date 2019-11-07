#ifndef __ESTIMATOR_LIB_H_
#define __ESTIMATOR_LIB_H_

#include <iostream>
#include <stdio.h>
#include <time.h>

#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"				//Per scrivere messaggi di array di float a 32 bit
#include "std_msgs/Int32.h"					//Per scrivere messaggi di interi a 32 bit
#include <image_transport/image_transport.h>			//Per scrivere messaggi contenenti immagini
#include "std_msgs/Bool.h"					//Per scrivere messaggi di variabili booleane

#include "opencv2/opencv.hpp"					//Inlcusione delle funzioni della lib.OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"

#include "discriminator/trackingStatus.h"
#include "discriminator/imAndId.h"

using namespace std;
using namespace cv;

//---------------------------- Richiamo delle variabili globali definite in estimator -----------------//
extern int h;
extern int w;
extern string nodeName;	
extern double frequency;
extern Mat frame;
extern int nFrameOptical;
extern int nCorners;
extern int idFrameToSave;
extern vector<uchar> newstatus;
extern vector<uchar> oldstatus;
extern vector<float> err;
extern Size winSize;
extern TermCriteria termcrit;
extern vector<float> ROI;
extern vector<float> ROINN;
extern double alphaROI;
extern double alphaBar;
extern float dt;
extern float sigmaXAcc;
extern float sigmaYAcc;
extern float sigmaXMeasureVel;
extern float sigmaYMeasureVel;
extern float sigmaXMeasurePos;
extern float sigmaYMeasurePos;
extern float kBrake, kBrake_vel, kBrake_acc;
extern float xAcc;
extern float yAcc;
extern ros::Subscriber trackingStatusSub;
extern ros::Publisher trackingStatusPub;
extern map<string,bool> trackingStates;
extern int numbTrackingNodes;
extern bool teamReady;
extern bool firstMeasure;
extern bool NewMeasure;
extern int currentId;
extern bool opticalStima;
extern double thresholdCorners;
extern double distanceCorners;
extern bool saveFrameCut;
extern bool saveStatisticData;
extern int frameToReset;
extern int frameToBrake;
extern std::string nameFileToSaveTime, nameFileToSaveKalman, nameFileToSaveCorners, nameFileToSaveKalmanRecupero;
extern double 	range1, range2;
extern int	dist1, dist2, dist3;
extern std::string nameUser;
extern bool is4StateKalman;
extern bool debugDoneWindow;



//--------------------------------------------Dichiarazione delle funzioni---------------------------------------------//
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& data_frame);
void IdframeCallback (const std_msgs::Int32& Idframe);	//IdframeCallback	//Non si usa, se va si toglie
void imageCallback(const position_estimation_pkg::imAndId::ConstPtr& imAndid_msg);	//const sensor_msgs::ImageConstPtr& msg
void isFirstMeasureCallback (const std_msgs::Bool& isFirst);
void newMeasureCallback (const std_msgs::Bool& newm);
void trackingStatusCallback (const position_estimation_pkg::trackingStatus::ConstPtr& status_msg);
void startCallback(const sensor_msgs::ImageConstPtr& msg);
void publishReadyStatus();
void waitForTeam();
vector<float> barEvaluation(vector<float> ROIIN);
Rect verifyRect (Rect ROIrect);
vector<float> barToState();
Mat frameCropperFromROI (vector<float> ROIIN, Mat frame, int id);
Mat frameCropperFromBar (vector<float> ROIIN, Mat frameToCrop, vector<float> bar, int id);
vector<Point2f> newCorners ( Mat framemeasuregray, Mat framegray, std::vector<Point2f> cornersmeasure);
vector<float> newBar ( std::vector<Point2f> cornersold, std::vector<Point2f> corners, std::vector<float> oldbar);
vector<float> newVel ( std::vector<Point2f> cornersold, std::vector<Point2f> corners);
void initStandardVar();
void initFromLaunch(ros::NodeHandle nodeToInit);
float getSign(float val);
vector<float> evaluateAcceleration(vector<float> state);
void calDistanceBetweenCorners(vector<float> ROIIN);


#endif		// __ESTIMATOR_LIB_H_
