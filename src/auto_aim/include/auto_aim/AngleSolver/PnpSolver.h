#ifndef PNPSOLVER_H
#define PNPSOLVER_H
//以下四个参数的大小与正负号与电机和相机的安装方向有关，尽量不要改公式里的符号，或者自己区分
const double PCBD=0;  //
const double PBMD=0; //13.7

const double YCBD=0.1;//8
const double YBMD=0.1;//8


#include "auto_aim/Settings/Settings.h"
#include<eigen3/Eigen/Dense>
#include <math.h>





class AngleSolver
{
public:
    AngleSolver(const char *camera_param_file_name, double z_scale = 1.0);
    AngleSolver();

    /**
     * @brief  设置相机安装参数
     * @param  第一个参数含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void setRelationPoseCameraPTZ(const double ptz_camera_x, const double ptz_camera_y,
                                  const double ptz_camera_z, double y_offset_barrel_ptz);

    /**
     * @brief  设置目标尺寸
     * @param  目标矩形真实宽度
     * @param  目标矩形真实高度
     * @return 返回值含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void setTargetSize(double width, double height);
    void setTargetSize(double lenth);//能量机关

    /**
     * @brief  解析角度
     * @author 梁尧森
     * @date   2018.9.21
     */

    bool getAngle(cv::Point2f *target2d,Eigen::Vector3d &tvec,Eigen::Vector3d &rvec);
    bool getAngle(std::vector<cv::Point2f> target2d,Eigen::Vector3d &tvec);

    /**
     * @brief  solvePnP
     * @author 参与开发人员
     * @date   2018-
     */
    void solvePnP4Points(const std::vector<cv::Point2f> points2d, Eigen::Vector3d &trans,Eigen::Vector3d &rvec);

    /**
     * @brief  相机坐标转换到PTZ坐标
     * @param  目标在相机坐标下的位置
     * @param  目标在PTZ坐标下的位置
     * @author 梁尧森
     * @date   2018.9.21
     */
    void tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos);

    /**
     * @brief  根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
     * @param  第一个参数含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void adjustPTZ2Barrel(const cv::Mat &pos_in_ptz,double &angle_x, double &angle_y,
                          double bullet_speed, double current_ptz_angle);

    /**
     * @brief  rectPnpSolver
     * @param  camera_param_file_name
     * @author 参与开发人员
     * @date   2018-
     */
    void rectPnpSolver(const char *camera_param_file_name);

    /**
     * @brief  获取光轴中心
     * @return 光轴中心
     * @author 梁尧森
     * @date   2018.9.21
     */
    cv::Point2f getImageCenter();

    /**
     * @brief 获取相对角度
     * @param 目标矩形
     * @param 正式长宽比
     * @param 补偿
     */
    bool getAngleWithRectify(const cv::RotatedRect &rect, double wh_ratio,
                             const cv::Point2f &offset = cv::Point2f());

    /**
     * @brief  获取矩形四个角点的坐标
     * @author 梁尧森
     * @date   2018.9.21
     */
    void getTarget2dPoinstion(const cv::RotatedRect &rect, std::vector<cv::Point2f> &target2d,
                              const cv::Point2f &offset);

    /**
     * @brief  无重力相对角解算
     * @param  目标坐标点
     * @author 参与开发人员
     * @date   2018-
     */
    bool getAngleWithoutGavity(std::vector<cv::Point2f> target2d);

    /**
     * @brief  能量机关专用PnP解算
     * @author 参与开发人员
     * @date   2020-
     */
    void solvePnP4Points_rune(const std::vector<cv::Point2f> &points2d, cv::Mat &rot, cv::Mat &trans);

    /**
     * @brief 无重力补偿
     * @author 参与开发人员
     * @date   2020-
     */
    void adjustPTZ2BarrelWithoutGavity(const cv::Mat &pos_in_ptz,
                                       double &angle_x, double &angle_y);

    /**
     * @brief
     * @param  第一个参数含义
     * @return 返回角度、距离、姿态角等信息
     * @author 梁尧森
     * @date   2018.9.21
     */
    double getPitRef();

    /**
     * @brief
     * @param  第一个参数含义
     * @return 返回角度、距离、姿态角等信息
     * @author 梁尧森
     * @date   2018.9.21
     */
    double getYawRef();

    /**
     * @brief  单目测距
     * @param  armor_rect
     * @param  dist
     * @author 陈强
     * @date   2021-
     */
    void getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist);

    /**
     * @brief  单目测距pnp算法
     * @param  2d点
     * @param  dist
     * @author 吴凯杰
     * @date   2022-
     */
    void getDistanceDanmuPnP(const std::vector<cv::Point2f> &points2d,double &dist);


    void Camera2Moto(double moto_pitch, double moto_yaw, Eigen::Vector3d tvec, Eigen::Vector3d ctvec,double &moto_move_pitch, double &moto_move_yaw, double v, double g);

    double trajectoryEstimation(double ground_dist,double height,double v,double g,double &fly_time/*顺便更正预测时间*/);

    void coordinary_transformation(double moto_pitch, double moto_yaw, Eigen::Vector3d &tvec, Eigen::Vector3d rvec, Eigen::Vector3d &moto_tvec);

public:
    cv::Mat cam_matrix;                 // 内参矩阵
    cv::Mat distortion_coeff;           // 畸变系数
    double width_target;                // 目标矩形真实宽度
    double height_target;               // 目标矩形真实高度
    std::vector<cv::Point3f> point3d;   // 目标三维坐标
    cv::Mat rot;                        // 旋转矩阵
    cv::Mat position_in_camera;         // 偏移向量
    cv::Mat position_in_ptz;            // 目标在PTZ坐标中的位置
    double pit_ref;                     // 俯仰角
    double yaw_ref;                     // 偏航角
private:
    cv::Mat rot_camera2ptz;             // 旋转矩阵（相机坐标转换到PTZ坐标）
    cv::Mat trans_camera2ptz;           // 偏移向量（相机坐标转换到PTZ坐标）
    double offset_y_barrel_ptz;         // 云台中心与枪管的偏移量
    double scale_z;                     // 放缩比例
};

#endif
