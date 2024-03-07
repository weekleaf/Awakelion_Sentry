#ifndef TARGETDETECTION_H
#define TARGETDETECTION_H

#include "auto_aim/Settings/Settings.h"

class TargetDetection
{
public:
    TargetDetection(TargetSizeParam &target_param_size, Debug &debug, std::vector<cv::Point> contours_target = std::vector<cv::Point>());
    /**
     * @brief  检测出符合面积大小，长宽比，hu特征的轮廓。返回新的列表
     * @param  源图-二值图
     * @param  目标轮廓列表引用
     * @return 是否有目标
     * @author 梁尧森
     * @date   2019.3.29
     */
    bool targetDetectionProc(cv::Mat src,
                              std::vector<std::vector<cv::Point>> &target_contours,
                              std::vector<cv::RotatedRect> &rot_rect_list,
                              std::vector<cv::Rect> &rect_list,
                              std::vector<struct TargetSize> &target_size_list);




    /**
     * @brief  返回新的列表
     * @param  轮廓
     * @param  筛选轮廓
     * @param  外接最大旋转矩形
     * @param  外接最大矩形
     * @param  轮廓属性
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.27
     */
    void filterByAttrubute(std::vector<std::vector<cv::Point>> contours,
                           std::vector<std::vector<cv::Point>> &target_contours,
                           std::vector<cv::RotatedRect> &rot_rect_list,
                           std::vector<cv::Rect> &rect_list,
                           std::vector<struct TargetSize> &target_size_list);


    /**
     * @brief  简介
     * @param  第一个参数
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.27
     */
    cv::RotatedRect adjustRect(const cv::RotatedRect &rect);

#ifdef DEBUG_MODE
public:
    cv::Mat ret;                              // src.clone()用于调试
    cv::Mat ret_2;                            // src.clone()用于调试
    cv::Mat ret_3;                            // src.clone()用于调试
    cv::Mat ret_4;
#endif // DEBUG_MODE

protected:
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i>              hierarchy;
    std::vector<cv::RotatedRect>        rot_rect_list;

private:
    TargetSizeParam *tsize;                   // 筛选使用的参数
    std::vector<cv::Point> contours_target;   // 目标轮廓
    struct Debug *debug;                      // 主要用于显示调试窗口
};

#endif  // TARGETDETECTION_H
