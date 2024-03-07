#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "auto_aim/Settings/Settings.h"
#include "auto_aim/Preprocessing/Preprocessing.h"
#include "auto_aim/ArmorDetector/TargetDetection.h"
#include "auto_aim/AngleSolver/PnpSolver.h"
#include "auto_aim/Serial/PackData.h"
#include "auto_aim/AngleSolver/GravityCompensateResolve.h"
#include "auto_aim/ArmorDetector/NCopenvino.h"
#include "auto_aim/kalman/armor_kalman.h"
#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y-p2.y))
#define GET_DIST(a,b) std::abs(a-b)/(a>b?a:b)

struct object_rect {
  int x;
  int y;
  int width;
  int height;
};

const int color_list[7][3] = {
    {216, 82, 24},   {236, 176, 31},  {125, 46, 141},  {118, 171, 47},
    {76, 189, 237},  {238, 19, 46},   {76, 76, 76},
};

class GetNum
{
public:

    GetNum(MainSettings *main_setting)
    {
        this->main_setting = main_setting;
    }

    void initResult()
    {
        label = "";
        score = 0;
        max_score = 0;
        max_strike_value = 0;
    }
    /**
     * @brief  获取图像平均像素值
     * @param  原图
     * @param  返回的平均像素值
     * @author 张森文
     * @date   2021.10.15
     */
    void getPixelValue(cv::Mat &image,int &ave_pixel_value);

    /**
     * @brief  获取装甲板数据（角度，中心坐标，高度，宽度等）
     * @param  目标装甲板信息
     * @author 张森文
     * @date   2022.10.30
     */
    void getArmorData(ArmorInfo &armor_list);


    /**
     * @brief  gamma转换
     * @param  原图
     * @param  gamma系数
     * @return 调节gamma值后的图像
     * @author 张森文
     * @date   2021.10.15
     */
    cv::Mat gammaTransform(cv::Mat &srcImage,float kFactor);

    /**
     * @brief  调节gamma
     * @param  原图
     * @return 调节gamma值后的图像
     * @author 张森文
     * @date   2021.10.15
     */
    cv::Mat adjustGamma(cv::Mat &image);

    /**
     * @brief  仿射变换
     * @param  原图
     * @return
     * @author 张森文
     * @date   2022.10.30
     */
    cv::Mat affineTransform(cv::Mat &image);


    /**
     * @brief  透视变换
     * @param  原图
     * @return
     * @author 张森文
     * @date   2022.10.30
     */
    cv::Mat warpPerspectiveTransform(cv::Mat &image);

    /**
     * @brief  扩展灯条尺寸
     * @param  原图
     * @author 张森文
     * @date   2022.10.30
     */
    void exLightSize();

    /**
     * @brief  HOG+SVM数字分类器
     * @param  原图
     * @author 张森文
     * @date   2021.10.15
     */
    void getHogSvmPredictResult(cv::Mat &image);

    /**
     * @brief  神经网络数字识别
     * @param  输入图片
     * @author 张森文
     * @date   2022.6.29
     */
    void NNgetPredictResult(cv::Mat &image);

    /**
     * @brief  图像均一化
     * @param  输入图像
     * @param  改变输入尺寸后的图像
     * @param  输入尺寸
     * @return 预测值
     * @author 黄敏瑜
     * @date   2022.6.29
     */
    int resizeUniform(cv::Mat &src, cv::Mat &dst, cv::Size dst_size,
                       object_rect &effect_area);

    /**
     * @brief  可视化界面
     * @param  输入图像
     * @param
     * @param
     * @author 黄敏瑜
     * @date   2022.6.30
     */
    void drawBboxes(const cv::Mat &bgr, const std::vector<BoxInfo> &bboxes,
                     object_rect effect_roi);

    /**
     * @brief  神经网络数字分类器
     * @param  输入模型
     * @param  输入图像
     * @author 黄敏瑜
     * @date   2022.6.30
     */
    int numClassifier(PicoDet &detector, cv::Mat result_image);

    /**
     * @brief  打击优先级
     * @author 张森文
     * @date   2022.6.30
     */
    std::string strikedPriority();

    /**
     * @brief  获取最优装甲板
     * @param  最优装甲板下标
     * @param  最优装甲板对应标签
     * @return 是否存在
     * @author 张森文
     * @date   2022.6.30
     */
    bool getBestArmor(int &max_id,std::string &best_label,std::vector<std::string> &label_list,std::vector<float> &score_list,std::vector<int> &index_list);

    /**
     * @brief  数字识别逻辑（多目标识别）
     * @param  输入图像
     * @param  待筛选装甲板集合
     * @param  是否第一帧
     * @param  目标装甲板id
     * @param  目标装甲板对应标签
     * @author 张森文
     * @date   2022.6.30
     */
    int digitDetetor(cv::Mat &src,std::vector<ArmorInfo> &armor_all_list,int &filter_num,int &max_id,std::string &label_figure);

    /**
     * @brief  数字识别逻辑（对所有待筛选装甲板进行识别）
     * @param  输入图像
     * @param  待筛选装甲板集合
     * @param  是否第一帧
     * @param  目标装甲板id
     * @param  目标装甲板对应标签
     * @author 张森文
     * @date   2022.6.30
     */
    int allDigitDetector(cv::Mat &src,std::vector<ArmorInfo> &armor_all_list,int &filter_num,int &max_id,std::string &label_figure);

public:

    MainSettings *main_setting;

    cv::Mat warpPerspective_image;      //透视变换后图像
    cv::Mat warperspective_mat;         //透视变换矩阵
    cv::Size roi_imgSize;
    cv::Point2f roiPoints[4];
    cv::Point2f ex_armor_points[4];     //扩展后四点坐标

    cv::Point2f armor_center;    //装甲板中心坐标
    cv::Point2f armor_points[4];    //装甲板四角点坐标

    float armor_width;    //装甲板宽度
    float armor_height;   //装甲板高度
    float armor_angle;    //目标装甲板矩形角度
    ArmorInfo armor_list; //装甲板信息

    float gamma = 1.7f;   //gamma参数

    std::string label;  //标签

    float score;    //置信度

    float max_score;    //最大置信度
    float max_strike_value;     //打击分数
};






class ArmorDetector: public AdaptiveThreshold, public TargetDetection
{
public:

    ArmorDetector(ArmorSettings *armor_setting, MainSettings *main_setting, std::vector<cv::Point> armor_contours);

    /**
     * @brief  自瞄模式
     * @param  原图
     * @return void
     * @author 醒狮战队所有参与自瞄开发的算法组成员
     * @date   2021.6.29
     */
    void armorDetecProc(cv::Mat src, AngleSolver &angle_slover,
                        PackData *pack_data, UnpackData *unpack_data,
                        Serial *serial, long t1_param, GravityCompensateResolve &gc,
                        ArmorDetector &armor,int &flag, int &lost_flag/*,armor_kalman &a_kalman*/);



    /**
     * @brief  检测出所有装甲板
     * @param  原图
     * @return 装甲板四个点的列表
     * @author 梁尧森
     * @date   2019.3.31
     */
    bool detectorProc(cv::Mat src, QuadrilateralPos &pos,
                      cv::RotatedRect &rot_rect, bool &is_small,GetNum &imgProc,int &flag);

    /**
     * @brief  进一步筛选灯条
     * @param  轮廓列表
     * @param  旋转矩形列表
     * @param  矩形列表
     * @param  目标尺寸信息列表
     * @param  灯条信息列表的引用
     * @author 梁尧森
     * @date   2019.4.20
     */
    void filterEnemyLight(cv::Mat src,
                          std::vector<std::vector<cv::Point>> light_contours,
                          std::vector<cv::RotatedRect> rot_rect_list,
                          std::vector<cv::Rect> rect_list,
                          std::vector<struct TargetSize> target_size_list,
                          std::vector<LightInfo> &light_info,
                          std::vector<cv::RotatedRect> &ellipse_rect_list);

    /**
     * @brief  根据两个灯条匹配出装甲板
     * @param  灯条信息列表
     * @param  装甲板信息列表的引用
     * @author 梁尧森
     * @date   2019.4.20
     */
    void findArmorFrom2Light(std::vector<LightInfo> &light_list,
                             std::vector<ArmorInfo> &armor_list, double &min_slope,
                             std::vector<cv::RotatedRect> ellipse_rect_list);

    /**
     * @brief  根据条件筛选装甲板
     * @param  装甲板信息列表
     * @param  装甲板信息列表的引用
     * @author 梁尧森
     * @date   2019.4.20
     */
    void filterAllArmor(std::vector<ArmorInfo> armor_list,
                        std::vector<ArmorInfo> &armor_all_list, double &min_slope);


    /**
     * @brief 避免越界
     * @param rect
     * @param size
     */
    bool makeRectSafe(cv::Rect &rect, cv::Size size);


    /**
     * @brief  boundingRRect
     * @return 旋转矩形
     * @author 参与开发人员
     * @date   2019-
     */
    cv::RotatedRect boundingRRect(const cv::RotatedRect &left,
                                  const cv::RotatedRect &right);
public:
    double pit_final, yaw_final;      // 最终要发送的pitch、yaw轴点角度
    double pit_before,yaw_before;
    float yaw_predict;
    float pit_ref, yaw_ref;
    double ex_yaw_final;
    int armor_num;                    // 识别到的装甲板数目
    int fps;                          // 帧率
    double L_v;
    double angle_old;
    float last_yaw_final;
    int nflag;

private:
    ArmorSettings *armor_setting;     // 装甲板滑动条参数
    MainSettings *main_setting;       // 主要设置（用于窗口的显示）

    cv::Mat src_cl;                   // 克隆原图便于调试使用
    int ex_flag = 0;
    int run_flag = 0;
    int temp_flag = 0;
    int temp_sum = 0;
    int shoot_flag = 0;
    float last_ex_yaw = 0;
    float last_delta_yaw = 0;
    int temp = 0;
    int temp_flag_flag = 0;
    int turn_temp = 0;
    int turn_temp_flag = 0;
    float sentry_w_last;
    float sentry_ref_abs_last;
    std::vector<cv::Point2f> last_armor_points;
    cv::Point2f last_armor_center;
    int lost_flag = 0; // 丢失标志位 1代表丢失 0代表没有丢失
 //   double angle_old;
    struct timeval t1_sentry;  // 计时
    struct timeval t2_sentry;  // 计时

    std::vector<double>right_Angle_set;
    std::vector<double>left_Angle_set;
};

#endif // ARMORDETECTOR_H
