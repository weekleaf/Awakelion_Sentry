#ifndef PREDICT_H
#define PREDICT_H

#include"auto_aim/kalman/armor_kalman.h"
#include <queue>
#include"auto_aim/ArmorDetector/inference_api2.hpp"
#include"auto_aim/ArmorDetector/ArmorDetector.h"
#include"auto_aim/RuneDetector/inference_api.hpp"
#include"auto_aim/AngleSolver/PnpSolver.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Eigen>

using namespace armor_detector;
using namespace buff_detector;

class ArmorPredictTool
{
public:
    ArmorPredictTool();
    ArmorPredictTool(AngleSolver angle_solver,double moto_pitch,double moto_yaw,double last_pitch,double last_yaw, double *cars_radio,ArmorObject* object_addr,
                     std::vector<ArmorObject> *cars_map,double bullet_speed,double running_time);

    bool findSameCls();
    bool solveCarRadio();
    bool predictRotated();
    bool predictMove();
    bool stateAdd();
    bool predictArmor();
    void kalmanInit();
    void inputData(AngleSolver angle_solver,double moto_pitch,double moto_yaw,double last_pitch,double last_yaw, double *cars_radio,ArmorObject* object_addr,
                   std::vector<ArmorObject> *cars_map,double bullet_speed,double running_time);

    cv::Point2f world2image(Eigen::Vector3d world_point);


    Eigen::Vector3d tvec_armor;
    Eigen::Vector3d tvec11;
    Eigen::Vector3d tvec22;






private:
    Eigen::Vector3d tvec1;
    Eigen::Vector3d tvec2;
    Eigen::Vector3d moto_tvec1;
    Eigen::Vector3d moto_tvec2;
    Eigen::Vector3d car_tvec;
    double car_angle;
    double *cars_radio;
    double car_radio;
    ArmorObject *object_addr;
    ArmorObject obj1;
    ArmorObject obj2;
    std::vector<ArmorObject> *cars_map;
    AngleSolver angle_solver;

    
    double moto_pitch;
    double moto_yaw;
    double last_pitch;
    double last_yaw;

    Armor_Kalman angle_kalman;
    Armor_Kalman car_kalman_x;
    Armor_Kalman car_kalman_y;
    Armor_Kalman car_kalman_z;
    double angle_predict;
    Eigen::Vector3d car_predict;
    


    bool switch_y=0;
    double running_time;
    double bullet_speed;




};
class RunePredictTool{

public:
    RunePredictTool();
//    RunePredictTool(AngleSolver angle_solver,BuffObject object,double moto_pitch,double moto_yaw,double bullet_speed,double running_time);
    void inputDataRune(AngleSolver angle_solver,BuffObject object,double moto_pitch,double moto_yaw,double bullet_speed,double running_time);
    void initKalman();
    bool setRuneCoordinary();






    cv::Point2f rune_center;
    std::vector<cv::Point2f> cur_pos_points;
    cv::Point2f cur_pos_center;
    double bullet_speed;
    double running_time;
    
    std::vector<cv::Point2f> pre_cur_pos_points;
    AngleSolver angle_solver;
    double moto_pitch;
    double moto_yaw;
    
    std::vector<double> angles;
    double pre_angle;
    Armor_Kalman kalman;

    Eigen::Vector3d cur_moto_tvec;


};
#endif // PREDICT_H
