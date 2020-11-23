#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <math.h>

#include "../include/ukf.h"
#include "../include/immukf.h"
#include <visualization_msgs/Marker.h>
#include "../include/rviz_visualizer.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
using namespace Eigen;
using namespace pcl;


bool init_= false;
double timestamp_ ;
double egoVelo_;
double egoYaw_;
double egoPreYaw_;
VectorXd preMeas_;

const double gammaG_ = 8; //9.22; // 99%
const double pG_ = 0.99;
const double pD_ = 0.9;
double r_;
double g_;

//bbox association param
const double distanceThres_ = 0.25;
const int lifeTimeThres_ = 8;
//bbox update params
const double bbYawChangeThres_  = 0.2; 
// const double bbYawChangeThres_  = 0.2; 
// const double bbAreaChangeThres_ = 0.2;
const double bbAreaChangeThres_ = 0.5;
const double bbVelThres_        = 0.05;
const double bbDeltaVel_        = 0.01;
const int    nStepBack_         = 3;

int countIt = 0;
vector<UKF> targets_;
vector<int> trackNumVec_;

ifstream inFile_;
ifstream yawFile_;



//testing recover origin tf
vector<vector<double>> egoPoints_;
vector<vector<double>> egoDeltaHis_;
vector<double> egoDiffYaw_;
// 1.22191 is first ego yaw, and add M_PI/2 for cartesian visualization
double firstEgoYawOffset_ = 1.22191 - M_PI/2;


void findMaxZandS(UKF target, VectorXd& maxDetZ, MatrixXd& maxDetS){
    double cv_det   = target.lS_cv_.determinant();
    double ctrv_det = target.lS_ctrv_.determinant();
    double rm_det   = target.lS_rm_.determinant();

    if(cv_det > ctrv_det){
        if(cv_det > rm_det)  {
            maxDetZ = target.zPredCVl_;
            maxDetS = target.lS_cv_;
        }
        else                 {
            maxDetZ = target.zPredRMl_;
            maxDetS = target.lS_rm_;
        }
    }
    else{
        if(ctrv_det > rm_det) {
            maxDetZ = target.zPredCTRVl_;
            maxDetS = target.lS_ctrv_;
        }
        else                  {
            maxDetZ = target.zPredRMl_;
            maxDetS = target.lS_rm_;
        }
    }


}
void measurementValidation(vector<vector<double>> trackPoints, UKF& target, VectorXd maxDetZ, MatrixXd maxDetS,
     vector<VectorXd>& measVec, vector<int>& matchingVec , double dt, double timestamp){
    int flag=0;
    double smallestNIS = 999;
  
    for(int i = 0; i < trackPoints.size(); i++){
        double x = trackPoints[i][0];
        double y = trackPoints[i][1];


        VectorXd meas = VectorXd(2);
        meas << x, y ;
        ROS_INFO_STREAM("meas x" << x << " y : " <<y);
        
       

        VectorXd diff = meas - maxDetZ;
        double nis = diff.transpose()*maxDetS.inverse()*diff;
        double d=sqrt(nis);
         ROS_INFO_STREAM("diff" << diff);
        double euc = sqrt(diff(1)*diff(1)+diff(0)*diff(0));
        if(d < gammaG_ & matchingVec[i] == 0){ // x^2 99% range
        //if(euc<5 & matchingVec[i] == 0){
        ROS_INFO_STREAM("nis  " << nis <<" Matched measurment x:"<< trackPoints[i][0]  << " y: " << trackPoints[i][1]);    
                measVec.push_back(meas);
                matchingVec[i] = 1;
                target.lifetime_+=1;
                target.lifetimestamp=timestamp;
                flag=1;         
        }
        else
        {
         ROS_INFO_STREAM("nis  " << nis );
        }

            
    }
            if ( flag == 0 )
        {
            
            target.killtime += 1; 
        }
        
}

void filterPDA(UKF& target, vector<VectorXd> measVec, vector<double>& lambdaVec){
    // calculating association probability
    // UKF one = targets[0];
    double numMeas = measVec.size();
    
    double b = 2*numMeas*(1-pD_*pG_)/(gammaG_*pD_);
    double eCVSum   = 0;
    double eCTRVSum = 0;
    double eRMSum   = 0;



    vector<double> eCvVec;
    vector<double> eCtrvVec;
    vector<double> eRmVec;

    vector<VectorXd> diffCVVec;
    vector<VectorXd> diffCTRVVec;
    vector<VectorXd> diffRMVec;

    for(int i = 0; i < numMeas; i++){
        VectorXd diffCV   = measVec[i] - target.zPredCVl_;
        VectorXd diffCTRV = measVec[i] - target.zPredCTRVl_;
        VectorXd diffRM   = measVec[i] - target.zPredRMl_;
        ROS_INFO_STREAM("diff CTRV: " << diffCTRV);
        diffCVVec.push_back(diffCV);
        diffCTRVVec.push_back(diffCTRV);
        diffRMVec.push_back(diffRM);

        double eCV   = exp(  -0.5*diffCV.transpose()*  target.lS_cv_.inverse()  *diffCV);
        double eCTRV = exp(-0.5*diffCTRV.transpose()*  target.lS_ctrv_.inverse()*diffCTRV);
        double eRM   = exp(  -0.5*diffRM.transpose()*  target.lS_rm_.inverse()  *diffRM);
         ROS_INFO_STREAM("lS ?? : " << target.lS_ctrv_);
        ROS_INFO_STREAM("eCTRV: " << eCTRV);
        eCvVec.push_back(eCV);
        eCtrvVec.push_back(eCTRV);
        eRmVec.push_back(eRM);

        eCVSum   += eCV;
        eCTRVSum += eCTRV;
        eRMSum   += eRM;
    }
    double betaCVZero   = b/(b+eCVSum);
    double betaCTRVZero = b/(b+eCTRVSum);
    double betaRMZero   = b/(b+eRMSum);

    vector<double> betaCV;
    vector<double> betaCTRV;
    vector<double> betaRM;
    double betaCvSum =betaCVZero;
    double betaCTRVSum =betaCTRVZero;
    double betaRMSum =betaRMZero;
     ROS_INFO_STREAM("e ctrv sum: " << eCTRVSum);
    // cout << "beta cv" << endl;
    for(int i = 0; i < numMeas; i++){
        double tempCV   = eCvVec[i]/(b+eCVSum);
        double tempCTRV = eCtrvVec[i]/(b+eCTRVSum);
        double tempRM   = eRmVec[i]/(b+eRMSum);
        // cout << tempCV << endl;
        betaCV.push_back(tempCV);
        betaCTRV.push_back(tempCTRV);
        betaRM.push_back(tempRM);
        betaCvSum +=tempCV;
        betaCTRVSum +=tempCTRV;
        betaRMSum +=tempRM;

    }
    ROS_INFO_STREAM("beta ctrv sum: " << betaCTRVSum);
   for(int i = 0; i < numMeas; i++){
        betaCV[i]  = betaCV[i]/betaCvSum;
        betaCTRV[i]  = betaCTRV[i]/betaCTRVSum;
        betaRM[i]  = betaRM[i]/betaRMSum;
         ROS_INFO_STREAM("beta: " << betaCTRV[0]);
    }
   
    VectorXd sigmaXcv;
    VectorXd sigmaXctrv;
    VectorXd sigmaXrm;
    sigmaXcv.setZero(2);
    sigmaXctrv.setZero(2);
    sigmaXrm.setZero(2);

    for(int i = 0; i < numMeas; i++){
        sigmaXcv   += betaCV[i]*diffCVVec[i];
        sigmaXctrv += betaCTRV[i]*diffCTRVVec[i];
        sigmaXrm   += betaRM[i]*diffRMVec[i];
    }

    MatrixXd sigmaPcv;
    MatrixXd sigmaPctrv;
    MatrixXd sigmaPrm;
    sigmaPcv.setZero(2,2);
    sigmaPctrv.setZero(2,2);
    sigmaPrm.setZero(2,2);
    for(int i = 0; i < numMeas; i++){
        sigmaPcv   += (betaCV[i]  *diffCVVec[i]  *diffCVVec[i].transpose()     - sigmaXcv*sigmaXcv.transpose());
        sigmaPctrv += (betaCTRV[i]*diffCTRVVec[i]*diffCTRVVec[i].transpose()   - sigmaXctrv*sigmaXctrv.transpose());
        sigmaPrm   += (betaRM[i]  *diffRMVec[i]  *diffRMVec[i].transpose()     - sigmaXrm*sigmaXrm.transpose());
    }
    
    // update x and P
   // ROS_INFO_STREAM("old State x cv: " << target.x_ctrv_);
   // ROS_INFO_STREAM("Kalman gain: " << target.K_ctrv_);
    ROS_INFO_STREAM("PDA güncelleme x cv: " << sigmaXctrv);

    target.x_cv_   = target.x_cv_   + target.K_cv_*sigmaXcv;
    target.x_ctrv_ = target.x_ctrv_ + target.K_ctrv_*sigmaXctrv;
    target.x_rm_   = target.x_rm_   + target.K_rm_*sigmaXrm;
   // cout<< " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!cv!!!!!!!!!!!!!!!!!!!!!"<<target.K_cv_*sigmaXcv<< std::endl;
   //  cout<< " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!ctrv!!!!!!!!!!!!!!!!!!!!!!"<<target.K_ctrv_*sigmaXctrv<< std::endl;
   //   cout<< " !!!!!!!!!!!!!!!!!!!!!!!!!!!!rm!!!!!!!!!!!!!!!!!!!!!!"<<target.K_rm_*sigmaXrm<< std::endl;

   /* double x = sigmaXcv(0);
    double y = sigmaXcv(1);
    double diffx= x- target.x_merge_(0);
    double diffy= y- target.x_merge_(1);
    double vx= diffx/0.01;
    double vy=diffy/0.01; 
    double v = sqrt(vx*vx + vy*vy);
    double yaw= atan2(vx, vy);
    target.x_cv_(2)=target.x_ctrv_(2)=target.x_rm_(2) = v;
    target.x_cv_(3)=target.x_ctrv_(3)=target.x_rm_(3) = yaw;*/
  // cout<< " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< target.x_ctrv_(3)<< target.x_rm_(3)<<std::endl;
    while (target.x_cv_(3)> M_PI) target.x_cv_(3) -= 2.*M_PI;
    while (target.x_cv_(3)<-M_PI) target.x_cv_(3) += 2.*M_PI;
    while (target.x_ctrv_(3)> M_PI) target.x_ctrv_(3) -= 2.*M_PI;
    while (target.x_ctrv_(3)<-M_PI) target.x_ctrv_(3) += 2.*M_PI;
    while (target.x_rm_(3)> M_PI) target.x_rm_(3) -= 2.*M_PI;
    while (target.x_rm_(3)<-M_PI) target.x_rm_(3) += 2.*M_PI;
  
    if(numMeas != 0){
    	target.P_cv_   = betaCVZero*target.P_cv_ +
                  (1-betaCVZero)*(target.P_cv_ - target.K_cv_*target.lS_cv_*target.K_cv_.transpose()) +
                  target.K_cv_*sigmaPcv*target.K_cv_.transpose();
	    target.P_ctrv_ = betaCTRVZero*target.P_ctrv_ +
	                  (1-betaCTRVZero)*(target.P_ctrv_ - target.K_ctrv_*target.lS_ctrv_*target.K_ctrv_.transpose()) +
	                  target.K_ctrv_*sigmaPctrv*target.K_ctrv_.transpose();
	    target.P_rm_   = betaRMZero*target.P_rm_ +
	                  (1-betaRMZero)*(target.P_rm_ - target.K_rm_*target.lS_rm_*target.K_rm_.transpose()) +
	                  target.K_rm_*sigmaPrm*target.K_rm_.transpose();
    }
    else{
    	target.P_cv_   = target.P_cv_   - target.K_cv_  *target.lS_cv_  *target.K_cv_.transpose();
	    target.P_ctrv_ = target.P_ctrv_ - target.K_ctrv_*target.lS_ctrv_*target.K_ctrv_.transpose();
	    target.P_rm_   = target.P_rm_   - target.K_rm_  *target.lS_rm_  *target.K_rm_.transpose();
    }
    // cout << "after update p cv: "<<endl << target.P_cv_ << endl;
     
    VectorXd maxDetZ;
    MatrixXd maxDetS;
    // cout << endl<<"filterPDA" << endl;
    findMaxZandS(target, maxDetZ, maxDetS);
    double Vk =  M_PI *sqrt(gammaG_ * maxDetS.determinant());
   
    double lambdaCV, lambdaCTRV, lambdaRM;
    if(numMeas != 0){
    	lambdaCV   = (1 - pG_*pD_)/pow(Vk, numMeas) +
	                        pD_*pow(Vk, 1-numMeas)*eCVSum/(numMeas*sqrt(2*M_PI*target.lS_cv_.determinant()));
	    lambdaCTRV = (1 - pG_*pD_)/pow(Vk, numMeas) +
	                        pD_*pow(Vk, 1-numMeas)*eCTRVSum/(numMeas*sqrt(2*M_PI*target.lS_ctrv_.determinant()));
	    lambdaRM   = (1 - pG_*pD_)/pow(Vk, numMeas) +
	                        pD_*pow(Vk, 1-numMeas)*eRMSum/(numMeas*sqrt(2*M_PI*target.lS_rm_.determinant()));
    }
    else{
    	lambdaCV   = (1 - pG_*pD_)/pow(Vk, numMeas);
	    lambdaCTRV = (1 - pG_*pD_)/pow(Vk, numMeas);
	    lambdaRM   = (1 - pG_*pD_)/pow(Vk, numMeas);
    }
    // cout <<endl<< "lambda: "<<endl<<lambdaCV << " "<< lambdaCTRV<<" "<< lambdaRM << endl;
    lambdaVec.push_back(lambdaCV);
    lambdaVec.push_back(lambdaCTRV);
    lambdaVec.push_back(lambdaRM);
    //target.x_cv_   = target.x_cv_   + target.K_cv_*lambdaCV;

}





void tracking_immUkf(vector<vector<double>>  bBoxes,  double timestamp,
                     PointCloud<PointXYZ>& targetPoints, vector<vector<double>>& targetVandYaw,
                     vector<int>& trackManage, vector<bool>& isStaticVec,
                     vector<bool>& isVisVec, vector<PointCloud<PointXYZ>>& visBBs, ros::Publisher vis_pub, ros::Publisher pub,  std_msgs::Float32MultiArray msg_array1, double r, double g ,std_msgs::Float32MultiArray pp_objects,vector<PointCloud<PointXYZ>> Boxes){
printf("called \n");
r_=r;
g_=g;

if(timestamp_ !=0 && abs((timestamp_ - timestamp)/1000)>5.0)
{
targets_.clear();
init_ = false;
trackNumVec_.clear();
targetVandYaw.clear();
timestamp_=timestamp;

};
std::cout <<" Bboxes into the call back "<<bBoxes.size() << std::endl;

// initialize with first measurements
vector<vector<double>> trackPoints = bBoxes;
std::cout << " trackpoint size" << trackPoints.size() <<std::endl;

if(!init_) {
    	//for(int i = 0; i < trackPoints.size(); i++){
            if(trackPoints.size()>0){
               int i=0;
                double px = trackPoints[i][0];
                double py = trackPoints[i][1];
                double pz = trackPoints[i][2];
                PointXYZ o;
                o.x = px;
                o.y = py;
                o.z = pz;
                targetPoints.push_back(o);

                vector<double> VandYaw;
                VandYaw.push_back(0);
                VandYaw.push_back(0);
                targetVandYaw.push_back(VandYaw);
                isStaticVec.push_back(false);
                isVisVec.push_back(false);

	    		VectorXd initMeas = VectorXd(2);
	    		initMeas << px, py;

                UKF ukf;
                ukf.Initialize(initMeas, timestamp,Boxes[i]);
                targets_.push_back(ukf);
                //initialize trackNumVec
                trackNumVec_.push_back(1);

    	}
        timestamp_ = timestamp;
        trackManage = trackNumVec_;
        init_ = true;
        assert(targets_.size() == trackNumVec_.size());
        assert(targets_.size() == targetPoints.size());
        assert(targetPoints.size()== targetVandYaw.size());
        return;
    }
    vector<int> matchingVec(trackPoints.size()); 
    double dt = (timestamp - timestamp_)/1000;
   /* if (abs(dt) >10 )
    {
        targets_.clear();
        init_=false;
        trackNumVec_.clear();
        targetVandYaw.clear();
        timestamp_ = timestamp;
        return;
            
    }*/
    //double dt=0.2;     // !!!!! ELİF
    timestamp_ = timestamp;
    ROS_INFO_STREAM("time stamp: " << timestamp);
    ROS_INFO_STREAM("dt: " << dt);
for(int i = 0; i < targets_.size(); i++){
        //reset isVisBB_ to false
        targets_[i].isVisBB_ = false;

    	//todo: modify here. This skips irregular measurement and nan
    	//if(trackNumVec_[i] == 0) continue;
        // prevent ukf not to explode
        /*if(targets_[i].P_merge_.determinant() > 10 || targets_[i].P_merge_(4,4) > 1000){
            trackNumVec_[i] = 0;
            continue;
        }*/
        // cout << "target state start -----------------------------------"<<endl;
        // cout << "covariance"<<endl<<targets_[i].P_merge_<<endl;
        VectorXd maxDetZ;
        MatrixXd maxDetS;
    	vector<VectorXd> measVec;

    	vector<double> lambdaVec;
    	//cout << "ProcessIMMUKF" << endl;
        
    	ROS_INFO_STREAM("Previous State x cv: " << targets_[i].x_cv_(0)  << " x rm: " << targets_[i].x_rm_(0));
        targets_[i].ProcessIMMUKF(dt);

        ROS_INFO_STREAM("Predicted State x cv: " << targets_[i].x_cv_(0)  << " x rm: " << targets_[i].x_rm_(0));
       
        findMaxZandS(targets_[i], maxDetZ, maxDetS); 
        measurementValidation( bBoxes, targets_[i], maxDetZ, maxDetS, measVec,matchingVec,dt,timestamp_);
        ROS_INFO_STREAM("target lifetime: " << targets_[i].lifetime_);
        ROS_INFO_STREAM("target killtime: " << targets_[i].killtime);
        ROS_INFO_STREAM("meas number  " << measVec.size());
        //ROS_INFO_STREAM("meas matching vec " << matchingVec[0] <<" "<<matchingVec[1]);
        filterPDA(targets_[i], measVec, lambdaVec);
        ROS_INFO_STREAM("updated State x cv: " << targets_[i].x_cv_(0)  << " y: " << targets_[i].x_cv_(1)<< " v: "<< targets_[i].x_cv_(2));
	    // cout << "PostIMMUKF" << endl;
	    targets_[i].PostProcessIMMUKF(lambdaVec);
        ROS_INFO_STREAM("updated State x merge: " << targets_[i].x_merge_(0)  << " y: " << targets_[i].x_merge_(1)<< " v: "<< targets_[i].x_merge_(2));

        
   
}; // end of imm-ukf process
int deletedNum = 0;

  //kill process
for(int i = 0; i < targets_.size(); i++){
    if(targets_[i].killtime > 10)
        {
            cout << "kill time  "<< (timestamp_ -targets_[i].lifetimestamp)<< endl;
             if((timestamp_ -targets_[i].lifetimestamp) > 1000)
             {
            targets_.erase(targets_.begin() +i);
            deletedNum +=1;
            break;

             }

             
        }
}
cout << "deleted num "<< deletedNum<< endl;
// making new ukf target
int addedNum = 0;
int targetNum = targets_.size();
for(int i = 0; i < matchingVec.size(); i ++){
    
    if(matchingVec[i] == 0){
        cout<<"not matching " << trackPoints[i][0] <<" "<<trackPoints[i][0] << endl;
        //if(targets_.size()==2)   sleep(10);
       
        double px = trackPoints[i][0];
        double py = trackPoints[i][1];

        VectorXd initMeas = VectorXd(2);
        initMeas << px, py;

        UKF ukf;
        ukf.Initialize(initMeas, timestamp,Boxes[i]);
        targets_.push_back(ukf);
        //initialize trackNumVec
        trackNumVec_.push_back(1);
        addedNum ++;
    }
}
cout << "!!!!!!!!!!! added num "<< addedNum<< endl;
cout << "target num "<< targets_.size() << endl;
assert(targets_.size() == (addedNum + targetNum));

PointCloud<PointXYZ> vis_points;
PointXYZ vis_point;
vis_points.clear();
//msg_array1.data.clear();

for(int i = 0; i < targets_.size(); i++){
  if(targets_[i].lifetime_ > 10 && i<10){

    vis_point.x= targets_[i].x_merge_(0);
    vis_point.y= targets_[i].x_merge_(1);
    vis_points.push_back(vis_point);
    msg_array1.data[i]=targets_[i].x_merge_(0);
    msg_array1.data[10+i]=targets_[i].x_merge_(1);
    msg_array1.data[20+i]=targets_[i].x_merge_(2);
    msg_array1.data[30+i]=targets_[i].x_merge_(3);
    msg_array1.data[40+i]=targets_[i].x_merge_(4);

    pp_objects.data[8*i]=  targets_[i].dimensions[0].x;
    pp_objects.data[8*i+1]=targets_[i].dimensions[1].x;
    pp_objects.data[8*i+2]=targets_[i].dimensions[2].x;
    pp_objects.data[8*i+3]=targets_[i].dimensions[3].x;
    pp_objects.data[8*i+4]=targets_[i].dimensions[0].y;
    pp_objects.data[8*i+5]=targets_[i].dimensions[1].y;
    pp_objects.data[8*i+6]=targets_[i].dimensions[2].y;
    pp_objects.data[8*i+7]=targets_[i].dimensions[3].y;
   

  }

}
r_=0.0;
g_=1.0;
//rvizVis visualize(vis_points,  vis_pub, r_ ,g_);
//pub.publish(pp_objects); //inhouse pp
pub.publish(pp_objects); //USPathPlanning





};