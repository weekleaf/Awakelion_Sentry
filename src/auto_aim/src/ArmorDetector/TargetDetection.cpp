#include "auto_aim/ArmorDetector/TargetDetection.h"

TargetDetection::TargetDetection(TargetSizeParam &target_param_size, Debug &debug, std::vector<cv::Point> contours_target)
{
    tsize = &target_param_size;
    this->debug = &debug;
    this->contours_target.assign(contours_target.begin(), contours_target.end());
}




bool TargetDetection::targetDetectionProc(cv::Mat src,
                                          std::vector<std::vector<cv::Point>> &target_contours,
                                          std::vector<cv::RotatedRect> &rot_rect_list,
                                          std::vector<cv::Rect> &rect_list,
                                          std::vector<struct TargetSize> &target_size_list)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(src, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    filterByAttrubute(contours, target_contours, rot_rect_list, rect_list, target_size_list);

#ifdef DEBUG_MODE
    if(debug->b_show_target)
    {
        cv::namedWindow("tgt_det");
        cv::imshow("tgt_det", ret);
    }
    else
    {
        if(-1 != cv::getWindowProperty("tgt_det", 1))
            cv::destroyWindow("tgt_det");
    }
#endif // DEBUG_MODE

    return target_contours.size();
}


void TargetDetection::filterByAttrubute(std::vector<std::vector<cv::Point>> contours,
                                        std::vector<std::vector<cv::Point>> &target_contours,
                                        std::vector<cv::RotatedRect> &rot_rect_list,
                                        std::vector<cv::Rect> &rect_list,
                                        std::vector<TargetSize> &target_size_list)
{
    if(target_contours.size())
        target_contours.clear();
    if(rot_rect_list.size())
        rot_rect_list.clear();
    if(rect_list.size())
        rect_list.clear();
    if(target_size_list.size())
        target_size_list.clear();

    for(int i = 0; i < contours.size(); i++)
    {
        struct TargetSize target_size;

        // 1.轮廓长度
        target_size.len = cv::arcLength(contours[i], true);
        if(target_size.len < tsize->len_min || target_size.len > tsize->len_max)
            continue;

        // 2.长宽比
        cv::RotatedRect rot_rect = adjustRect(cv::minAreaRect(contours[i]));
        target_size.ratio = rot_rect.size.height * 1.0 / rot_rect.size.width;
        if(target_size.ratio < tsize->ratio_min || target_size.ratio > tsize->ratio_max)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_ratio, s_len;
            s_len << target_size.len;
            s_ratio << target_size.ratio;
            cv::putText(ret, "len: " + s_len.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            cv::putText(ret, "ratio: " + s_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_ratio, s_len;
            s_ratio << target_size.ratio;
            s_len << target_size.len;
            cv::putText(ret, "len: " + s_len.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            cv::putText(ret, "ratio: " + s_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE

        // 3.面积
        target_size.area = cv::contourArea(contours[i]);
        if(target_size.area < tsize->area_min || target_size.area > tsize->area_max)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_area;
            s_area << target_size.area;;
            cv::putText(ret, "area: " + s_area.str(), cv::Point(rot_rect.center) + cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_area;
            s_area << target_size.area;
            cv::putText(ret, "area: " + s_area.str(), cv::Point(rot_rect.center) + cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE

        // 4.矩形度
        target_size.area_ratio = target_size.area * 1.0 / rot_rect.size.area();
        if(target_size.area_ratio < tsize->area_ratio_min * 0.01 || target_size.area_ratio > tsize->area_ratio_max * 0.01)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_area_ratio;
            s_area_ratio << target_size.area_ratio;
            cv::putText(ret, "area_ratio: " + s_area_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_area_ratio;
            s_area_ratio << target_size.area_ratio;
            cv::putText(ret, "area_ratio: " + s_area_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE

        // 5.面积比长度（似圆度）
        target_size.area_len_ratio = target_size.area * 1./ target_size.len;
        if(target_size.area_len_ratio < tsize->area_len_ratio_min * 0.1 || target_size.area_len_ratio > tsize->area_len_ratio_max * 0.1)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_area_len_ratio;
            s_area_len_ratio << target_size.area_len_ratio;
            cv::putText(ret, "area_len_ratio: " + s_area_len_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 50), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_area_len_ratio;
            s_area_len_ratio << target_size.area_len_ratio;
            cv::putText(ret, "area_len_ratio: " + s_area_len_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 50), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE

        // 6.多边形逼近
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 0.01 * target_size.len, true);
        target_size.corners = approx.size();
        if(target_size.corners < tsize->corners_size_min)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_corners;
            s_corners << target_size.corners;
            cv::putText(ret, "poly_size: " + s_corners.str(), cv::Point(rot_rect.center) + cv::Point(0, 60), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_corners;
            s_corners << target_size.corners;
            cv::putText(ret, "poly_size: " + s_corners.str(), cv::Point(rot_rect.center) + cv::Point(0, 60), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE

#ifdef DEBUG_MODE
        cv::Point2f vertex[4];
        rot_rect.points(vertex);
        for (int i = 0; i < 4; i++)
        {
            cv::line(this->ret, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0, 128, 0), 1);
        }
#endif // DEBUG_MODE

        target_contours.push_back(contours[i]);
        rot_rect_list.push_back(rot_rect);
        rect_list.push_back(cv::boundingRect(contours[i]));
        target_size_list.push_back(target_size);
    }
}

cv::RotatedRect TargetDetection::adjustRect(const cv::RotatedRect &rect)
{
    const cv::Size2f &s = rect.size;
    if(s.width < s.height)
        return rect;
    return cv::RotatedRect(rect.center, cv::Size2f(s.height, s.width), rect.angle + 90.0);
}

