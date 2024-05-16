#ifndef SETTINGS_H
#define SETTINGS_H

#include <mutex>
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#define DEBUG_MODE      // 是否开启debug
//#define SAVE_IMG
#define ANGLE 1
#define USE_SERIAL      // 使用串口
#define USE_VIDEO 0    // 0迈德威视 1摄像头/视频
#define SHOW_WIDTH 750.0/*640*///750
#define SHOW_HEIGHT 600.0/*480*///600

//-----------------------------------【宏定义-调试模块】--------------------------------------------
// brief：主要用于定义一些窗口
//-----------------------------------------------------------------------------------------------
#define VIDEO_PATH "/home/rm/Infantry_2023.7.21_test/FOSU_AWAKENLION_INFANTRY/save_video/3.mp4"     //测试视频路径run_left
#define WIN_OTHER "主要参数"
#define WIN_CAMERA "相机位置参数"

//-----------------------------------【宏定义-键盘指令】--------------------------------------------
// brief：
// 按v  开始、结束录视频
// 按p  截图
// 按s  保存参数
//------------------------------------------------------------------------------------------------
#define KEY_SAVE_VIDEO  76  //开始、结束录视频
#define KEY_SAVE_RESULT 70  //截图
#define KEY_SAVE_PARAM  115 //保存参数

#define ARMS 1          // 当前PC所属机器人编号
#if (ARMS == 1)
//#define MODEL_PATH "/home/rm/nn_model/best_06_02.xml"
#define MODEL_PATH "/home/rm/newnewnew/src/auto_aim/tdt5.onnx"
#define MODEL_PATH_BUFF "/home/rm/newnewnew/src/auto_aim/buff-05-28-01.xml"
#define PARAM_OTHER_PATH "/home/rm/Infantry_2023.7.21_test/config_file/param_other.yml"  //全局配置文件路径
#define PARAM_CAMERA_PATH "/home/rm/Infantry_2023.7.21_test/config_file/param_camera.yml"  //装甲板配置文件路径
#define PARAM_CALIBRATION_752 "/home/rm/newnewnew/src/auto_aim/calibration/Camera752-infantry.xml"  //相机参数
#define SAVE_VIDEO_DIR   "/home/rm/Infantry_2023.7.21_test/FOSU_AWAKENLION_INFANTRY/save_video/"
#define SAVE_PIC_DIR   "/home/rm/Infantry_2023.7.21_test/FOSU_AWAKENLION_INFANTRY/save_pic/"

#endif  // ARMS

//-----------------------------------【模式切换枚举】------------------------------------------
// brief：用于模式切换
//------------------------------------------------------------------------------------------
enum MainMode
{
    armor_mode, big_rune_mode, static_rune_mode, small_rune_mode
};

//-----------------------------------【装甲板颜色枚举】----------------------------------------
// brief：用于敌方装甲板颜色切换
//------------------------------------------------------------------------------------------
enum EnemyColor
{
    red, blue
};

//-----------------------------------【ROI模式切换枚举】---------------------------------------
// brief：用于ROI模式下与普通模式切换
//------------------------------------------------------------------------------------------
enum TrackMode
{
    normal_mode, roi_mode
};

//-----------------------------------【配置-串口】--------------------------------------------
// brief：串口配置参数
//------------------------------------------------------------------------------------------
struct DeviceParam
{
    const char *dev_name;                 // 设备名称
    int baud_rate;                        // 波特率
    int databits;                         // 数据位
    int stopbits;                         // 停止位
    char parity;                          // 校验位
    DeviceParam()
    {
        dev_name = "/dev/ttyACM0";         // 设备名称
        baud_rate = 115200;                // 波特率
        databits = 8;                      // 数据位
        stopbits = 1;                      // 停止位
        parity = 'n';                      // 校验位
    }
};

//--------------7740---------------------【调式参数】---------------------------------------------
// brief：用于调试参数，创建滑动条
//------------------------------------------------------------------------------------------
struct Debug
{
    int src;
    int detect_armor;
    int armor_chosen;
    int predict_point;
    int inaccuracy_point;
    int dc_pitch;
    int dc_yaw;
    int expore_time;

    Debug()
    {
        src=1;
        detect_armor=0;
        armor_chosen=0;
        predict_point=0;
        dc_pitch=0;
        dc_yaw=0;
        expore_time=2000;

    }
};

//-----------------------------------【摄像头参数】-------------------------------------------
// brief：摄像头参数初始化，便于角度解算
//------------------------------------------------------------------------------------------
struct CameraParam
{
    double PCBD;
    double PBMD;
    double YCBD;
    double YBMD;
    CameraParam()
    {
        PCBD=10.0;
        PBMD=10.0;
        YCBD=8.0;
        YBMD=8.0;
    }
};

//-----------------------------------【矩形四个点坐标】----------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct QuadrilateralPos
{
    cv::Point2f p[4];                      // 矩形四个角点坐标
};



//-----------------------------------【重定义数据结构类型】-------------------------------------
// brief：便于与电控进行协议沟通
//------------------------------------------------------------------------------------------
typedef signed int	       s32;
typedef unsigned int	   u32;
typedef signed short	   s16;
typedef unsigned short	   u16;
typedef unsigned char	   u8;

typedef signed int	       int32_t;
typedef unsigned int	   uint32_t;
typedef signed short	   int16_t;
typedef unsigned short	   uint16_t;
typedef unsigned char	   uint8_t;

//-----------------------------------【类-设置】-------------------------------------------
// brief：
//---------------------------------------------------------------------------------------
class MainSettings
{
public:
    MainSettings(const char *param1,const char *param2);

    /**
     * @brief  设置主要参数
     * @param  param_path
     * @return void
     * @author 梁尧森
     * @date   2019.3.7
     */
    void setMainParam(const char *win_name);

    /**
     * @brief  读取其他参数
     * @param  param_path
     * @return void
     * @author 梁尧森
     * @date   2019.3.7
     */
    void readOtherParam(const char *param_path);

    /**
     * @brief  保存配置
     * @param  第一个参数
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.7
     */
    void writeOtherParam(const char *param_path);

    /**
     * @brief  设置工业相机位置
     * @param  窗口名称
     * @author 参与开发人员
     * @date   2019-
     */
    void setCameraParam(const char *win_name);

    void readCameraParam(const char *param_path);

    void writeCameraParam(const char *param_path);

//11.12
public:
    int main_mode = armor_mode;            // 主要模式
    int main_mode_flag = 0;                // 模式切换标志位
    int enemy_color = red;                 // 敌方装甲板颜色
    int track_mode = normal_mode;          // ROI模式
    Debug debug;                           // 调试参数
    CameraParam camera_param;              // 工业相机初始化参数
    DeviceParam port_param;                // 串口通信参数
};
#endif // SETTINGS_H
