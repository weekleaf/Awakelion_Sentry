#include "auto_aim/ArmorDetector/ArmorDetector.h"
#include "ros/ros.h"
#include "robot_driver/vision_rx_data.h"
#include "robot_driver/vision_tx_data.h"

extern ros::Publisher vision_pub;
extern robot_driver::vision_tx_data pc_recv_mesg;

auto detector = PicoDet("/home/rm/vision_ws/src/auto_aim/pico64_2023_5.onnx");

ArmorDetector::ArmorDetector(ArmorSettings *armor_setting, MainSettings *main_setting, std::vector<cv::Point> armor_contours)
    :AdaptiveThreshold(armor_setting->preprocess_param), TargetDetection(armor_setting->tgt_size_param, main_setting->debug, armor_contours)
{
    this->armor_setting = armor_setting;
    this->main_setting = main_setting;
}

void ArmorDetector::armorDetecProc(cv::Mat src, AngleSolver &angle_slover,
                                   PackData *pack_data, UnpackData *unpack_data,
                                   Serial *serial, long t1_param, GravityCompensateResolve &gc,
                                   ArmorDetector &armor,int &flag, int &lost_flag/*,armor_kalman &a_kalman*/)
{
    //电控调试使用time_x,time_y,num
//    static double time_x = 0;
//    static double time_y = 0;
    static int num = 0;

    bool is_small;               // 是否小装甲板
    bool is_find_armor = false;  // 是否寻找到目标装甲板
    bool is_cut = false;
    QuadrilateralPos armor_pos;  // 装甲板四个点坐标
    cv::RotatedRect rot_rect;    // 装甲板旋转矩形
//    static double  pit_before=0,yaw_before=0;
    static double pit_last, yaw_last;
    static int top = 0;
//    static bool is_left;
//    float num = 2.0;
    GetNum imgPro(main_setting);


    // 1.识别灯条; 2.获取所有装甲板四个点位置; 3.选择最优装甲板
    is_find_armor = armor.detectorProc(src, armor_pos, rot_rect, is_small,imgPro,flag);

    // 4.解算角度
    if(is_find_armor)
    {
        flag++;
        lost_flag = 0;
        std::vector<cv::Point2f> armor_rect;

        /*
        float ex_armor_x, ex_armor_y;
        ex_armor_x = std::abs(armor_pos.p[0].x - armor_pos.p[2].x);
        ex_armor_y = armor_pos.p[0].y - armor_pos.p[3].y;

        //接受电控信号使用拓展装甲板
        if(unpack_data->getStm2PcMesg()->stm32_info_data.is_left == 1)
        {
            for(int i = 0; i < 4; i++)
            {
                armor_pos.p[i].x -= ex_armor_x;
                armor_pos.p[i].y += ex_armor_y;
            }
        }
        else if(unpack_data->getStm2PcMesg()->stm32_info_data.is_left == 2)
        {
            for(int i = 0; i < 4; i++)
            {
                armor_pos.p[i].x += ex_armor_x;
                armor_pos.p[i].y -= ex_armor_y;
            }
        }
        */

        for(int i = 0; i < 4; i++)
            armor_rect.push_back(armor_pos.p[i]);

#ifdef DEBUG_MODE
        cv::Mat src_clone = src.clone();
        for(int i = 0; i < 4; i++)
            cv::line(src_clone, armor_pos.p[i], armor_pos.p[(i + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
        if(main_setting->debug.b_show_fps)
        {
            std::ostringstream s_fps;
            s_fps << armor.fps;
            cv::putText(src_clone, "FPS: " + s_fps.str(), cv::Point(20, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
        }
        if(main_setting->debug.b_show_ex_armor)
        {
            cv::namedWindow("ex_result");
            cv::imshow("ex_result", src_clone);
        }
        else
        {
            if(-1 != cv::getWindowProperty("ex_result", 1))
                cv::destroyWindow("ex_result");
        }
#endif // DEBUG_MODE

        //设置大，小装甲板在不同曝光，二值，相机下对应的“真实”尺寸
        if(is_small)
            /*small       big        exporetime   cam
             * 13.1 4.95  22.7 5.4   1500         new
             *
                                                                */
            angle_slover.setTargetSize(13.1, 4.9);   //12.4 5.4||13.0 5.6(new)
        else
            angle_slover.setTargetSize(22.7, 5.4);   //21.6 5.4||22.7 5.4(new)

        //进入角度解算
        angle_slover.getAngle(armor_rect, main_setting->camera_param.bullet_speed);



        if(1)
        {
            double dist;

            //进入测距解算
            if(main_setting->debug.b_dist_mode == 1){
                angle_slover.getDistancePnP(armor_rect, dist);
            }
            else if(main_setting->debug.b_dist_mode == 2){
                angle_slover.getDistanceDanmu(armor_rect, dist);
            }
            //pitch重力补偿
            if(/*!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd) && unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd != 0*/ 0)//电控发送弹速不为0或异常值
            {
                pit_final = -gc.solveAngleWithGravity(-angle_slover.getPitRef(), dist / 100, /*unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd*/ 12);
            }else{
                pit_final = -gc.solveAngleWithGravity(-angle_slover.getPitRef(), dist / 100, 12);
            }

#ifdef DEBUG_MODE
            std::cout << "=====================================================================" << std::endl;
            if(is_small)
                std::cout << "小装甲板" << std::endl;
            else
                std::cout << "大装甲板" << std::endl;
            std::cout << "distance:  " << dist << std::endl;
//            std::cout << "speed:     " << unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd << std::endl;
            std::cout << "=====================================================================" << std::endl;
#endif // DEBUG_MODE
        }
        else //电控发送弹速异常
        {
            pit_final = angle_slover.getPitRef();
#ifdef DEBUG_MODE
            std::cout << "can not get the bullet spd!!!" << std::endl;
#endif // DEBUG_MODE
        }
        //进入小陀螺云台补偿
//        if(unpack_data->getStm2PcMesg()->stm32_info_data.is_top == 1 && top == 0){
//            yaw_final = -angle_slover.getYawRef() - 0;
//            //top = 1;
//            std::cout << "成功补偿" << std::endl;
//            std::cout << "成功补偿" << std::endl;
//            std::cout << "成功补偿" << std::endl;
//            std::cout << "成功补偿" << std::endl;
//            std::cout << "成功补偿" << std::endl;
//            std::cout << "成功补偿" << std::endl;
//            std::cout << "成功补偿" << std::endl;
//            std::cout << "成功补偿" << std::endl;

//        }
//        else{
            yaw_final = -angle_slover.getYawRef();
//            std::cout << "没有补偿" << std::endl;
//            if(unpack_data->getStm2PcMesg()->stm32_info_data.is_top == 0)
//                top = 0;
//        }



#ifdef DEBUG_MODE
        std::cout << "=====================================================================" << std::endl;
        std::cout << "pit_final: " << pit_final << std::endl;
        std::cout << "yaw_final: " << yaw_final << std::endl;
        std::cout << "=====================================================================" << std::endl;

        //得到每轮运行时间
//        if(time_x <= 0.01){
//            time_x = (double)cv::getTickCount();
//        }
//        else{
//            time_y = ((double)cv::getTickCount() - time_x) / cv::getTickFrequency();
//            time_x = (double)cv::getTickCount();
//            time_ = u32(time_y * 1000);
//            std::cout << "time_y = " << u32(time_y * 1000) << "ms" << std::endl;
//        }
#endif // DEBUG_MODE

        // pack_data->setPc2StmMesg()->gimbal_control_data.pit_ref = pit_final;
        // pack_data->setPc2StmMesg()->gimbal_control_data.yaw_ref = yaw_final;
        // pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 1;   // 视觉有效位
        // pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;    // 时间戳     /*u32(time_y * 1000)*/ //每轮运行时间
        // pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = 0;

        pc_recv_mesg.aim_pitch = pit_final;
        pc_recv_mesg.aim_yaw = yaw_final;
        pc_recv_mesg.visual_valid = 1;

    }
    else
    {
        //电控调试代码

        //方波
//        std::cout <<"方波"<<std::endl;
//        if(num<50){
//            pit_last = 10;
//            yaw_last = 10;
//            pit_final = 10;
//            pit_final = 10;
//            num++;
//        }else if(num<100){
//            pit_last = -10;
//            yaw_last = -10;
//            pit_final = -10;
//            pit_final = -10;
//            num++;
//        }else{
//            num = 0;
//        }

        //正弦波
//        if(num <= 150 &&num >=- 150){
//            pit_last = num/10.0;
//            yaw_last = num/10.0;
//            pit_final = num/10.0;
//            yaw_final = num/10.0;
//            num++;
//        }else if(num <= 450){

//            pit_last = pit_last - 0.1;
//            yaw_last = yaw_last - 0.1;
//            pit_final = pit_final - 0.1;
//            yaw_final = yaw_final - 0.1;
//            num ++;

//        }
//        else{
//            num = -150;
//        }

        flag = 0;

        // pack_data->setPc2StmMesg()->gimbal_control_data.pit_ref = pit_last;
        // pack_data->setPc2StmMesg()->gimbal_control_data.yaw_ref = yaw_last;
        // pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;    //时间戳     /*u32(time_y * 1000)*/ //每轮运行时间
        // pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = 0;

        pc_recv_mesg.aim_pitch = pit_last;
        pc_recv_mesg.aim_yaw = yaw_last;

        //if(1)
        if(lost_flag < 25)
            //pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 1;
            pc_recv_mesg.visual_valid = 1;
        else
        {
            //pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 0;
            pc_recv_mesg.visual_valid = 0;
            pit_final = 0;
            yaw_final = 0;
        }

        lost_flag++;
    }

    // 6.串口发送
    // pack_data->process(serial);
    vision_pub.publish(pc_recv_mesg);
}

bool ArmorDetector::detectorProc(cv::Mat src, QuadrilateralPos &pos, cv::RotatedRect &rot_rect, bool &is_small,GetNum &imgProc,int &flag)
{
    cv::Mat bin;

#ifdef DEBUG_MODE
    ret = src.clone();
    ret_2 = src.clone();
    ret_3 = src.clone();
#endif // DEBUG_MODE
    struct PreprocessInfo preprocess_info;
    if(main_setting->debug.b_thre_mode == 1){
        if(this->main_setting->enemy_color == red)
            mulAndIterRedTre(src, bin, preprocess_info);
        else
            mulAndIterBlueTre(src, bin, preprocess_info);
    }
    if(main_setting->debug.b_thre_mode == 2){
        if(this->main_setting->enemy_color == red)
            channelSubRedTre(src, bin, preprocess_info);
        else
            channelSubBlueTre(src, bin, preprocess_info);
    }
#ifdef DEBUG_MODE
    src_cl = src.clone();
    if(this->main_setting->debug.b_show_bin)
    {
        cv::namedWindow("armor_bin");
        cv::imshow("armor_bin", bin);
    }
    else
    {
        if(-1 != cv::getWindowProperty("armor_bin", 1))
            cv::destroyWindow("armor_bin");
    }
#endif // DEBUG_MODE

    std::vector<std::vector<cv::Point>> light_contours;  // 灯条轮廓
    std::vector<cv::RotatedRect> rot_rect_list;          // 灯条矩形集合
    std::vector<cv::Rect> rect_list;                     // 灯条矩形集合
    std::vector<struct TargetSize> target_size_list;     // 目标尺寸集合(一一对应)
    std::vector<LightInfo> light_list;                   // 灯条集合
    std::vector<cv::RotatedRect> ellipse_rect_list;      // 椭圆拟合灯条


    // 1.识别敌方灯条
    if(!targetDetectionProc(bin, light_contours, rot_rect_list, rect_list, target_size_list))
        return false;
    filterEnemyLight(src, light_contours, rot_rect_list, rect_list, target_size_list, light_list, ellipse_rect_list);

#ifdef DEBUG_MODE
    if(main_setting->debug.b_show_target)
    {
        cv::namedWindow("enemy_det");
        cv::imshow("enemy_det",ret_2);
    }
    else
    {
        if(-1 != cv::getWindowProperty("enemy_det",1))
            cv::destroyWindow("enemy_det");
    }
#endif // DEBUG_MODE

    std::vector<ArmorInfo> armor_pre_list;
    std::vector<ArmorInfo> armor_all_list;
    static ArmorInfo armor_all_last;
    static int filter_num;
    double min_slope;

    // 2.识别敌方装甲板
    findArmorFrom2Light(light_list, armor_pre_list, min_slope, ellipse_rect_list);
    filterAllArmor(armor_pre_list, armor_all_list, min_slope);
//    findArmor(light_list,armor_all_list);


#ifdef DEBUG_MODE
    if(main_setting->debug.b_show_armor)
    {
        cv::namedWindow("armor_det");;
        cv::imshow("armor_det", ret_3);
    }
    else
    {
        if(-1 != cv::getWindowProperty("armor_det", 1))
            cv::destroyWindow("armor_det");
    }
#endif // DEBUG_MODE

    // 3.筛选最优目标
    int s_max_idx = 0;

    if(filter_num > 0)
    {
        //当前帧所有候选目标与前一帧目标比较
        for(int i = 0; i < armor_all_list.size(); i++)
        {

            float &cur_x = armor_all_list[i].rrect.center.x;
            float &cur_y = armor_all_list[i].rrect.center.y;

            float &last_x = armor_all_last.rrect.center.x;
            float &last_y = armor_all_last.rrect.center.y;
 
            float dist = std::sqrt(std::pow(cur_x-last_x, 2) + std::pow(cur_y-last_y, 2));
            armor_all_list[i].s_list.push_back(dist / 892. + 1);
        }
    }
    filter_num++;

    armor_num = armor_all_list.size();

    std::string label;
//    static double old_armor_center_x=0;
//    static double old_armor_center_y=0;
//    static int waiting_frame=1;
//    static int old_is_small=-1;


//    double compensation_value;
//    getBestDetector(armor_all_list,s_max_idx,compensation_value);

//    if(!imgProc.digitDetetor(src,armor_all_list,filter_num,s_max_idx,label))
//        return false;
    if(!imgProc.allDigitDetector(src,armor_all_list,filter_num,s_max_idx,label))
        return false;


    if(armor_all_list.size() == 0 || label == "0")
        return false;

//以前的代码（最优装甲板）
#ifdef DEBUG_MODE
    cv::Mat src_clone = src.clone();

    for(int i = 0; i < armor_all_list.size(); i++)
    {
        cv::Point2f vertex[4];
        armor_all_list[i].rrect.points(vertex);
        for(int j = 0; j < 4; j++)
            cv::line(src_clone, vertex[j], vertex[(j + 1) % 4], (label != "0" && i == s_max_idx)?cv::Scalar(0, 255, 0):cv::Scalar(0,0,255), 1);

        std::ostringstream label_filiter,probability_filiter,strike_filiter;
        label_filiter << label;
        probability_filiter << imgProc.max_score;
        strike_filiter << imgProc.max_strike_value;

        if(i == s_max_idx && label != "0")
        {
            cv::putText(src_clone, "label: " + label_filiter.str(), cv::Point(armor_all_list[i].rrect.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
            cv::putText(src_clone, "max_score: " + probability_filiter.str(), cv::Point(armor_all_list[i].rrect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
            cv::putText(src_clone, "max_strike_value: " + strike_filiter.str(), cv::Point(armor_all_list[i].rrect.center) + cv::Point(0, 60), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
        }
    }
    if(main_setting->debug.b_show_result)
    {
        cv::namedWindow("result");
        cv::imshow("result", src_clone);
    }
    else
    {
        if(-1 != cv::getWindowProperty("result", 1))
            cv::destroyWindow("result");
    }
#endif // DEBUG_MODE

    armor_all_last = armor_all_list[s_max_idx]; 
    pos = armor_all_last.pos;
    rot_rect = armor_all_last.rrect;
    is_small = armor_all_list[s_max_idx].is_small;
//    old_armor_center_x=armor_all_last.rrect.center.x;
//    old_armor_center_y=armor_all_last.rrect.center.y;
//    old_is_small=armor_all_list[s_max_idx].is_small;
    return true;
}



void ArmorDetector::filterAllArmor(std::vector<ArmorInfo> armor_list,
                                   std::vector<ArmorInfo> &armor_all_list,double &min_slope)
{
    const double small_armor_wh_threshold = 3.4;     //判断大小装甲板，长宽比系数，标定完后需修改
    //筛选所有待定的目标
    for(int i = 0; i < armor_list.size(); i++)
    {
        const cv::RotatedRect &rect = armor_list[i].rrect;

        //1.长宽比
        double w = std::max(rect.size.width, rect.size.height);
        double h = std::min(rect.size.width, rect.size.height);
        double wh_ratio = w / h;
        if (wh_ratio > armor_setting->armor_param.armor_hw_ratio_max * 0.1 || wh_ratio < armor_setting->armor_param.armor_hw_ratio_min * 0.1)
        {
#ifdef DEBUG_MODE
            std::stringstream s;
            s << wh_ratio;
            cv::putText(ret_3, "armor_wh_ratio: " + s.str(), cv::Point(rect.center) + cv::Point(0, 50), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255));
#endif
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::stringstream s;
            s << wh_ratio;
            cv::putText(ret_3, "armor_wh_ratio: " + s.str(), cv::Point(rect.center) + cv::Point(0, 50), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
        }
#endif

        bool &is_small = armor_list[i].is_small;

        std::cout<<"wh_ratio:   "<<wh_ratio<<std::endl;
        if (wh_ratio < small_armor_wh_threshold)
            is_small = true;
        else
            is_small = false;

        armor_list[i].s_list.push_back(1.4 / wh_ratio);
        armor_list[i].s_list.push_back(armor_list[i].rrect.size.area() /*/ 25000.*/);
        armor_all_list.push_back(armor_list[i]);
    }
}


void ArmorDetector::findArmorFrom2Light(std::vector<LightInfo> &light_list,
                                        std::vector<ArmorInfo> &armor_list, double &min_slope,
                                        std::vector<cv::RotatedRect> ellipse_rect_list)
{
    min_slope = 9999.f;
    for(int i = 0; i < light_list.size(); i++)
    {
        const cv::RotatedRect &rect_i = light_list[i].rrect;
        const cv::Point &center_i = rect_i.center;
        double slope_i = light_list[i].slope;
        float xi = center_i.x;
        float yi = center_i.y;
        float area_height = rect_i.size.height;

        for(int j = i + 1; j < light_list.size(); j++)
        {
            const cv::RotatedRect &rect_j = light_list[j].rrect;
            const cv::Point &center_j = rect_j.center;
            double slope_j = light_list[j].slope;
            float xj = center_j.x;
            float yj = center_j.y;
            cv::RotatedRect rot_rect = boundingRRect(rect_i, rect_j);
            //椭圆拟合角度差 注释1给出的是 灯条与y轴的夹角之和

//            std::cout << "ellipse_rect_list.angle_i = " << ellipse_rect_list[i].angle <<std::endl;
//            std::cout << "ellipse_rect_list.angle_j = " << ellipse_rect_list[j].angle <<std::endl;
            //注释1灯条角度范围为-90-90度 若灯条角度大于90度则变换为-90度内
//            if(ellipse_rect_list[i].angle > 90 ){
//                ellipse_rect_list[i].angle = (ellipse_rect_list[i].angle - 180) ;

//            }
//            if(ellipse_rect_list[j].angle > 90 ){
//                ellipse_rect_list[j].angle = (ellipse_rect_list[j].angle - 180) ;

//            }


//            float dis_ellipse_angle = abs(ellipse_rect_list[i].angle - ellipse_rect_list[j].angle);

            //对于y轴的两边,同边相减,异边角度差为两等条与y轴的夹角相加
            float dis_ellipse_angle,ellipse_angle_max,ellipse_angle_min;
            ellipse_angle_max = std::max(ellipse_rect_list[i].angle,ellipse_rect_list[j].angle);
            ellipse_angle_min = std::min(ellipse_rect_list[i].angle,ellipse_rect_list[j].angle);
            if(ellipse_angle_max-ellipse_angle_min>=90){
                dis_ellipse_angle = 180-ellipse_angle_max+ellipse_angle_min;
            }else{
                dis_ellipse_angle = ellipse_angle_max-ellipse_angle_min;
            }


#ifdef DEBUG_MODE
            QuadrilateralPos temp;

            rot_rect.points(temp.p);
            for(int k = 0; k < 4; k++)
                cv::line(ret_3, temp.p[k], temp.p[(k + 1) % 4], cv::Scalar(0, 0, 255), 1, 8);
#endif // DEBUG_MODE

            // 1.高度差
            float delta_height = GET_DIST(rect_i.size.height, rect_j.size.height);
            if(delta_height > armor_setting->armor_param.delta_height_max * 0.001)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_delta_height;
                s_delta_height << delta_height;

                cv::putText(ret_3, "delta_height: " + s_delta_height.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else{
                std::ostringstream s_delta_height;
                s_delta_height << delta_height;
                cv::putText(ret_3, "delta_height: " + s_delta_height.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
#endif // DEBUG_MODE

            // 2.角度
//            float delta_angle = abs(rect_j.angle - rect_i.angle);
//            if(delta_angle == 180)
//                delta_angle = 0;
            //float delta_angle=std::fabs(std::fabs(light_list[i].angle)-std::fabs(light_list[j].angle));
            float delta_angle = std::max(light_list[j].angle,light_list[i].angle) - std::min(light_list[j].angle,light_list[i].angle);
            if(dis_ellipse_angle > armor_setting->armor_param.delta_ellipse_angle_max * 0.1 )

            {
#ifdef DEBUG_MODE
                std::ostringstream s_j_angle, s_i_angle, s_delta_angle;
                s_j_angle << ellipse_rect_list[j].angle;
                s_i_angle << ellipse_rect_list[i].angle;
                s_delta_angle << dis_ellipse_angle;
                cv::putText(ret_3, "delta_angle: " + s_delta_angle.str(), cv::Point(rot_rect.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255) ,1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else
            {
                std::ostringstream s_j_angle, s_i_angle, s_delta_angle;
                s_j_angle << ellipse_rect_list[j].angle;
                s_i_angle << ellipse_rect_list[i].angle;
                s_delta_angle << dis_ellipse_angle;
                cv::putText(ret_3, "delta_angle: " + s_delta_angle.str(), cv::Point(rot_rect.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0) ,1);
            }
#endif // DEBUG_MODE

            // 3.宽比高
            float delta_w = abs(xj - xi);
            float delta_w_ratio = delta_w * 1. / rect_j.size.height;
            if (delta_w_ratio < armor_setting->armor_param.delta_w_ratio_min * 0.1 || delta_w_ratio > armor_setting->armor_param.delta_w_ratio_max * 0.1)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_delta_w_ratio;
                s_delta_w_ratio << delta_w_ratio;
                cv::putText(ret_3, "delta_width: " + s_delta_w_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else
            {
                std::ostringstream s_delta_w_ratio;
                s_delta_w_ratio << delta_w_ratio;
                cv::putText(ret_3, "delta_width: " + s_delta_w_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
#endif // DEBUG_MODE

            // 4.高度差比宽
            float delta_h = std::abs(yi - yj) / delta_w;
            if (delta_h > armor_setting->armor_param.delta_h_max * 0.001)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_delta_h;
                s_delta_h << delta_h;
                cv::putText(ret_3, "delta_h: " + s_delta_h.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else
            {
                std::ostringstream s_delta_h;
                s_delta_h << delta_h;
                cv::putText(ret_3, "delta_h: " + s_delta_h.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
#endif // DEBUG_MODE

            // 5.面积
            float delta_area = std::abs(xj-xi) * area_height;
//            std::cout << "armor_area: " << delta_area << std::endl;
            if (delta_area > armor_setting->armor_param.max_armor_area || delta_area < armor_setting->armor_param.min_armor_area)
            {
#ifdef  DEBUG_MODE
                std::ostringstream s_armor_area;
                s_armor_area <<  delta_area;
                cv::putText(ret_3, "delta_area: " + s_armor_area.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else
            {
                std::ostringstream s_armor_area;
                s_armor_area <<  delta_area;
                cv::putText(ret_3, "delta_area: " + s_armor_area.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
#endif // DEBUG_MODE


//            int is_error;

//            if ((slope_i > 0 && slope_j < 0) || (slope_i < 0 && slope_j > 0))
//            {
//                is_error = 1;
//                continue;
//            }
//            else
//            {
//                is_error = 0;
//            }
            //6.两个灯条的角度
            //if(dis_ellipse_angle > armor_setting->armor_param.ellipse_angle / 100)continue;

            //7.装甲板内是否有灯条
            if(armor_setting -> armor_param.contour_armor){
                std::vector<cv::Point> bounding_contours; //boundingRect所需要用到的点集；
                for(int n =0; n < light_list[i].contours.size() ;n++)
                {
                    bounding_contours.push_back(light_list[i].contours[n]);
                }
                for(int n =0; n < light_list[j].contours.size() ;n++){
                    bounding_contours.push_back(light_list[j].contours[n]);
                }
                cv::Rect bounding_armor = cv::boundingRect(bounding_contours);
                int contour_flag = 0;
                for(int n=0 ; n < light_list.size() ;n++){
                    cv::Point test_point;

                    test_point.x = light_list[n].rrect.center.x;
                    test_point.y = light_list[n].rrect.center.y;
                    if(light_list[n].rrect.center == light_list[i].rrect.center)continue;
                    if(light_list[n].rrect.center == light_list[j].rrect.center)continue;
                    if(bounding_armor.contains(test_point)){
                        contour_flag = 1;
                        break;
                    }
                }
                if(contour_flag ==1 )continue;
            }



            cv::Point2f p_i[4], p_j[4];
            rect_i.points(p_i);
            rect_j.points(p_j);

            ArmorInfo armor;
            if(rect_i.center.x < rect_j.center.x)
            {
                armor.l_light = light_list[i];
                armor.r_light = light_list[j];
            }
            else
            {
                armor.l_light = light_list[j];
                armor.r_light = light_list[i];
            }

            cv::Point2f *p = armor.pos.p;
            std::vector<cv::Point2f> p_list = {
                (p_i[1] + p_i[2]) / 2,
                (p_i[3] + p_i[0]) / 2,
                (p_j[1] + p_j[2]) / 2,
                (p_j[3] + p_j[0]) / 2
            };

            //装甲板矩形角点排序
            std::sort(p_list.begin(), p_list.end(), [](cv::Point2f &p1, cv::Point2f &p2)->bool {return p1.x < p2.x; });

            if(p_list[0].y > p_list[1].y)
            {
                cv::Point2f point_temp = p_list[0];
                p_list[0] = p_list[1];
                p_list[1] = point_temp;
            }
            if(p_list[2].y < p_list[3].y)
            {
                cv::Point2f point_temp = p_list[2];
                p_list[2] = p_list[3];
                p_list[3] = point_temp;
            }

            for(int i = 0; i < 4; i++)
                p[i] = p_list[i];

            armor.rrect = rot_rect;
            armor.s_list={1 - delta_height, (armor_setting->armor_param.delta_angle_max - delta_angle) / armor_setting->armor_param.delta_angle_max, -delta_h / 1.5 + 1};
            armor_list.push_back(armor);

#ifdef DEBUG_MODE
            for(int k = 0; k < 4; k++)
                cv::line(ret_3, armor.pos.p[k], armor.pos.p[(k+1) % 4], cv::Scalar(0, 255, 0), 3, 8);
#endif // DEBUG_MODE
        }
    }
}

cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)
{
    const cv::Point &pl = left.center, &pr = right.center;
    cv::Point2f center = (pl + pr) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
}

void ArmorDetector::filterEnemyLight(cv::Mat src,
                                     std::vector<std::vector<cv::Point>> light_contours,
                                     std::vector<cv::RotatedRect> rot_rect_list,
                                     std::vector<cv::Rect> rect_list,
                                     std::vector<struct TargetSize> target_size_list,
                                     std::vector<LightInfo> &light_info,
                                     std::vector<cv::RotatedRect> &ellipse_rect_list)
{
    for(int i = 0; i < light_contours.size(); i++)
    {
        LightInfo light_info_i;
//        std::cout<< light_contours.size()<< std::endl;
//        if (light_contours.size()<6){
//                    continue;
//        }
        cv::RotatedRect rot_rect = adjustRect(rot_rect_list[i]);

/*----------分区赛回来改过的角度，问题更多，会识别不到灯条---------        // 1.角度
        double delta_angle = rot_rect.angle;
        if(delta_angle > (armor_setting->tgt_size_param.slope_offset)*0.1)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_slope_offset;
            s_slope_offset << delta_angle;
            cv::putText(ret_2, "slope_offset: " + s_slope_offset.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_slope_offset;
            s_slope_offset << delta_angle;
            cv::putText(ret_2, "slope_offset: " + s_slope_offset.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE
*/
/*旧版角度，不会识别异常，但无法做到筛选，需要改成椭圆拟合的灯条角度，把倾斜过大的筛掉*/


        // 1.角度
        double angle = rot_rect.angle;
        angle = 90 - angle;
        angle = angle < 0.0 ? angle + 180 : angle;
        float delta_angle = abs(angle - 90);
        if(delta_angle > armor_setting->tgt_size_param.slope_offset)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_slope_offset;
            s_slope_offset << delta_angle;
            cv::putText(ret_2, "slope_offset: " + s_slope_offset.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_slope_offset;
            s_slope_offset << delta_angle;
            cv::putText(ret_2, "slope_offset: " + s_slope_offset.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE



        // 2.颜色
        cv::Rect rect = rect_list[i];
        static cv::Point ex_ratio_point = 0.5 * cv::Point(rect.width, rect.height);
        cv::Rect rect_ex(rect.tl() - ex_ratio_point, rect.br() + ex_ratio_point);
        makeRectSafe(rect_ex, cv::Size(src.cols * 1, src.rows * 1));
        cv::Mat roi_not_continue = src(rect_ex);
        cv::Mat ch[3];
        cv::Mat roi = roi_not_continue.clone();
        cv::split(roi, ch);
        int diff = 0;
        for(int i = 0; i < roi.rows; i++)
        {
            uchar *p_b = ch[0].ptr<uchar>(i);
            uchar *p_r = ch[2].ptr<uchar>(i);
            for(int j = 0; j < roi.cols; j++)
            {
                diff += *(p_b + j) - *(p_r + j);
            }
        }

        float avg_diff = diff * 1. / target_size_list[i].area;
//        if(std::abs(avg_diff) < armor_setting->tgt_size_param.color_th || ((this->main_setting->enemy_color == red && avg_diff > 0) || (this->main_setting->enemy_color == blue && avg_diff < 0)))
//        {
//#ifdef DEBUG_MODE
//            std::ostringstream s;
//            s << avg_diff;
//            cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
//            cv::rectangle(ret_2, rect_ex, cv::Scalar(0,0,255), 1);
//#endif
//            continue;
//        }
//#ifdef DEBUG_MODE
//        else
//        {
//            std::ostringstream s;
//            s << avg_diff;
//            cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
//            cv::rectangle(ret_2, rect_ex, cv::Scalar(0,255,0), 1);
//        }
//#endif
        if(this->main_setting->enemy_color == red)
        {
            if(std::abs(avg_diff) < armor_setting->tgt_size_param.color_th_r || ((this->main_setting->enemy_color == red && avg_diff > 0) || (this->main_setting->enemy_color == blue && avg_diff < 0)))
            {
    #ifdef DEBUG_MODE
                std::ostringstream s;
                s << avg_diff;
                cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
                cv::rectangle(ret_2, rect_ex, cv::Scalar(0,0,255), 1);
    #endif
                continue;
            }
    #ifdef DEBUG_MODE
            else
            {
                std::ostringstream s;
                s << avg_diff;
                cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
                cv::rectangle(ret_2, rect_ex, cv::Scalar(0,255,0), 1);
            }
    #endif
        }
        else if(this->main_setting->enemy_color == blue)
        {
            if(std::abs(avg_diff) < armor_setting->tgt_size_param.color_th_b || ((this->main_setting->enemy_color == red && avg_diff > 0) || (this->main_setting->enemy_color == blue && avg_diff < 0)))
            {
    #ifdef DEBUG_MODE
                std::ostringstream s;
                s << avg_diff;
                cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
                cv::rectangle(ret_2, rect_ex, cv::Scalar(0,0,255), 1);
    #endif
                continue;
            }
    #ifdef DEBUG_MODE
            else
            {
                std::ostringstream s;
                s << avg_diff;
                cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
                cv::rectangle(ret_2, rect_ex, cv::Scalar(0,255,0), 1);
            }
    #endif
        }

        cv::Point2f p[4];
         rot_rect.points(p);
        //椭圆角度
        cv::RotatedRect ellipse_rect = cv::fitEllipse(light_contours[i]);
        //if(ellipse_rect.angle < 180 - (armor_setting->tgt_size_param.ellipse_light_angle / 100) || ellipse_rect.angle > (armor_setting->tgt_size_param.ellipse_light_angle /100))continue;

        cv::Point2f p_up = (p[1] + p[2]) / 2;
        cv::Point2f p_down = (p[0] + p[3]) / 2;
        double slope = (p_up.x - p_down.x) / (p_up.y - p_down.y);
        light_info_i.idx = i;
        light_info_i.contours.assign(light_contours[i].begin(), light_contours[i].end());
        light_info_i.size = target_size_list[i];
        light_info_i.rrect = rot_rect;
        light_info_i.rect = rect_list[i];
        light_info_i.angle = delta_angle;
        light_info_i.slope = slope;
        light_info.push_back(light_info_i);
        ellipse_rect_list.push_back(ellipse_rect);
    }
}

bool ArmorDetector::makeRectSafe(cv::Rect & rect, cv::Size size)
{
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        return false;
    return true;
}






void GetNum::getPixelValue(cv::Mat &image,int &ave_pixel_value)
{

    int rows = image.rows;
    int cols = image.cols *image.channels();

    //遍历图像，得到图像总像素值
    for(int i = 0;i < rows;i ++){
        uchar *p_value = image.ptr<uchar>(i);
        for(int j = 0;j < cols;j ++){
            ave_pixel_value += *p_value;
        }
    }
    ave_pixel_value /= (rows * cols);     //得到平均像素值
}

void adaptiveBinaryzation(cv::Mat &gray,int ave_pixel_value)
{
    int rows = gray.rows;
    int cols = gray.cols *gray.channels();

    for(int i = 0;i < rows;i ++){
        uchar *p_value = gray.ptr<uchar>(i);
        for(int j = 0;j < cols;j ++){
            if(*p_value > ave_pixel_value){
                *p_value = 255;
            }
            else{
                *p_value = 0;
            }
        }
    }
}

void GetNum::getArmorData(ArmorInfo &armor_list)
{
    this->armor_list = armor_list;
    armor_angle = armor_list.rrect.angle;

    if(this->armor_list.is_small)
        roi_imgSize = cv::Size(armor_list.rrect.size.width,armor_list.rrect.size.width / 1.213);
    else
        roi_imgSize = cv::Size(armor_list.rrect.size.width,armor_list.rrect.size.width / 1.925);
    warperspective_mat = cv::Mat(3,3,CV_32FC1);

    roiPoints[0] = cv::Point2f(0,0);
    roiPoints[1] = cv::Point2f(roi_imgSize.width,0);
    roiPoints[2] = cv::Point2f(roi_imgSize.width,roi_imgSize.height);
    roiPoints[3] = cv::Point2f(0,roi_imgSize.height);
}



// -------------2023赛季 数字识别 获取ROI--------
void GetNum::NNgetPredictResult(cv::Mat &image)
{
    cv::Mat result_image;

    cv::Mat roi_image = warpPerspectiveTransform(image);    //透视变换

    if(armor_list.is_small){
        result_image =  roi_image(cv::Rect(roi_image.rows * 0.2100,0,roi_image.rows * 0.86,roi_image.rows)); //小装甲版
    }
    else{
        result_image =  roi_image(cv::Rect(roi_image.rows * 0.4625,0,roi_image.rows * 0.90,roi_image.rows));//大装甲板
    }

    cv::Mat classify_image = adjustGamma(result_image);    //调节gamma值

    if(main_setting->debug.b_save_digit_roi_pic)
        main_setting->grabImg(classify_image, main_setting->debug.n_key_order, main_setting->debug.f_save_pic_inter * cv::getTickFrequency() / 1000.0);


    if(main_setting->debug.b_show_digit_roi)
    {
        cv::namedWindow("digit_roi");
        cv::imshow("digit_roi", classify_image);

    }
    else
    {
        if(-1 != cv::getWindowProperty("digit_roi", 1))
            cv::destroyWindow("digit_roi");
    }


    cv::Mat classify_bin_image;
    cv::cvtColor(classify_image,classify_image,cv::COLOR_BGR2GRAY);     //转灰度图

    int ave_pixel_value = 0;

    int rows = classify_image.rows;
    int cols = classify_image.cols *classify_image.channels();

    //遍历图像，得到图像平均像素值
    this->getPixelValue(classify_image,ave_pixel_value);


    for(int i = 0;i < rows;i ++){
        uchar *p_value = classify_image.ptr<uchar>(i);
        for(int j = 0;j < cols;j ++){
            if(*p_value > ave_pixel_value){
                *p_value = 255;
            }
            else{
                *p_value = 0;
            }
        }
    }

    cv::threshold(classify_image,classify_bin_image,ave_pixel_value + 2,255,cv::THRESH_BINARY);    //二值化

    cv::medianBlur(classify_bin_image,classify_bin_image,3);    //中值滤波

    if(main_setting->debug.b_save_digit_bin)
        main_setting->grabImg(classify_bin_image, main_setting->debug.n_key_order, main_setting->debug.f_save_pic_inter * cv::getTickFrequency() / 1000.0);

    if(main_setting->debug.b_show_digit_bin)
    {
        cv::namedWindow("digit_thre");
        cv::imshow("digit_thre", classify_bin_image);

    }
    else
    {
        if(-1 != cv::getWindowProperty("digit_thre", 1))
            cv::destroyWindow("digit_thre");
    }

    cv::cvtColor(classify_bin_image,classify_bin_image,CV_GRAY2BGR);
    numClassifier(detector, classify_bin_image);       //将图片信息输入分类器

//    }
}



cv::Mat GetNum::gammaTransform(cv::Mat &srcImage,float kFactor)
{
    unsigned char LUT[256];
    for(int i = 0;i < 256;i++)
    {
        float f =(i + 0.5f) / 255;
        f = (float)(pow(f,kFactor));
        LUT[i] = cv::saturate_cast<uchar>(f * 255.0f - 0.5f);
    }
    cv::Mat resultImage = srcImage.clone();

    if(srcImage.channels() == 1)
    {
        cv::MatIterator_<uchar> iterator = resultImage.begin<uchar>();
        cv::MatIterator_<uchar> iteratorEnd = resultImage.end<uchar>();
        for(;iterator != iteratorEnd;iterator++)
        {
            *iterator = LUT[(*iterator)];
        }
    }
    else
    {
        cv::MatIterator_<cv::Vec3b> iterator = resultImage.begin<cv::Vec3b>();
        cv::MatIterator_<cv::Vec3b> iteratorEnd = resultImage.end<cv::Vec3b>();
        for(;iterator!=iteratorEnd;iterator++)
        {
            (*iterator)[0] = LUT[((*iterator)[0])];  //b
            (*iterator)[1] = LUT[((*iterator)[1])];  //g
            (*iterator)[2] = LUT[((*iterator)[2])];  //r
        }
    }

    return resultImage;
}


cv::Mat GetNum::adjustGamma(cv::Mat &image)
{
    float kFactor = 1 / this->gamma;
    cv::Mat result = gammaTransform(image,kFactor);
    return result;
}

cv::Mat GetNum::affineTransform(cv::Mat &armor_roi)
{
    cv::Mat rot_mat;
    //装甲板调整至标准位

    if(armor_angle < 90){
        rot_mat = cv::getRotationMatrix2D(cv::Point(armor_roi.cols/2,armor_roi.rows/2),-armor_angle ,1.0);   //矩形左倾
    }
    else{
        rot_mat = cv::getRotationMatrix2D(cv::Point(armor_roi.cols/2,armor_roi.rows/2),std::abs(180 - armor_angle) ,1.0);    //矩形右倾
    }
    cv::Size dst_sz(armor_roi.cols,armor_roi.rows);
    cv::warpAffine(armor_roi,armor_roi,rot_mat,dst_sz,cv::INTER_NEAREST);

    return armor_roi;

}

cv::Mat GetNum::warpPerspectiveTransform(cv::Mat &image)
{
    cv::Mat roi_armorImg;
    exLightSize();

    warperspective_mat = cv::getPerspectiveTransform(ex_armor_points,roiPoints);
    cv::warpPerspective(image,warpPerspective_image,warperspective_mat,roi_imgSize,cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(0));
    warpPerspective_image.copyTo(roi_armorImg);
    return roi_armorImg;
}

void GetNum::exLightSize()
{
    float expand_facter;        //扩展系数
    if(armor_list.is_small)
        expand_facter = 2.8;
    else
        expand_facter = 2.5;

    cv::Size exLSize(int(armor_list.l_light.rrect.size.width),int(armor_list.l_light.rrect.size.height*expand_facter));
    cv::Size exRSize(int(armor_list.r_light.rrect.size.width),int(armor_list.r_light.rrect.size.height*expand_facter));
    cv::RotatedRect exLlight(armor_list.l_light.rrect.center,exLSize,armor_list.rrect.angle);
    cv::RotatedRect exRlight(armor_list.r_light.rrect.center,exRSize,armor_list.rrect.angle);

    cv::Point2f l_points[4];
    exLlight.points(l_points);


    cv::Point2f up_l, down_l;
    if(l_points[0].x > l_points[3].x)
    {
        cv::Point2f temp = l_points[0];
        l_points[0] = l_points[3];
        l_points[3] = temp;
    }
    if(l_points[1].x > l_points[2].x)
    {
        cv::Point2f temp = l_points[1];
        l_points[1] = l_points[2];
        l_points[2] = temp;
    }

    if(l_points[2].y < l_points[3].y)
    {
        up_l = l_points[2];
        down_l = l_points[3];
    }
    else
    {
        up_l = l_points[3];
        down_l = l_points[2];
    }


    cv::Point2f r_points[4];
    exRlight.points(r_points);

    cv::Point2f up_r, down_r;
    if(r_points[0].x > r_points[3].x)
    {
        cv::Point2f temp = r_points[0];
        r_points[0] = r_points[3];
        r_points[3] = temp;
    }
    if(r_points[1].x > r_points[2].x)
    {
        cv::Point2f temp = r_points[1];
        r_points[1] = r_points[2];
        r_points[2] = temp;
    }

    if(r_points[1].y < r_points[0].y)
    {
        up_r = r_points[1];
        down_r = r_points[0];
    }
    else
    {
        up_r = r_points[0];
        down_r = r_points[1];
    }


    ex_armor_points[0] = up_l;
    ex_armor_points[1] = up_r;
    ex_armor_points[2] = down_r;
    ex_armor_points[3] = down_l;
}

int GetNum::resizeUniform(cv::Mat &src, cv::Mat &dst, cv::Size dst_size,
                   object_rect &effect_area) {
  int w = src.cols;
  int h = src.rows;
  int dst_w = dst_size.width;
  int dst_h = dst_size.height;
  dst = cv::Mat(cv::Size(dst_w, dst_h), CV_8UC3, cv::Scalar(0));

  float ratio_src = w * 1.0 / h;
  float ratio_dst = dst_w * 1.0 / dst_h;

  int tmp_w = 0;
  int tmp_h = 0;
  if (ratio_src > ratio_dst) {
    tmp_w = dst_w;
    tmp_h = floor((dst_w * 1.0 / w) * h);
  } else if (ratio_src <= ratio_dst) {
    tmp_h = dst_h;
    tmp_w = floor((dst_h * 1.0 / h) * w);
  }
  //hmy于6月30 22：：4修改此部分
      //else {
 //   cv::resize(src, dst, dst_size);
 //   effect_area.x = 0;
 //   effect_area.y = 0;
  //  effect_area.width = dst_w;
  //  effect_area.height = dst_h;
 //   return 0;

  cv::Mat tmp;
  cv::resize(src, tmp, cv::Size(tmp_w, tmp_h));

  if (tmp_w != dst_w) {
    int index_w = floor((dst_w - tmp_w) / 2.0);
    for (int i = 0; i < dst_h; i++) {
      memcpy(dst.data + i * dst_w * 3 + index_w * 3, tmp.data + i * tmp_w * 3,
             tmp_w * 3);
    }
    effect_area.x = index_w;
    effect_area.y = 0;
    effect_area.width = tmp_w;
    effect_area.height = tmp_h;
  } else if (tmp_h != dst_h) {
    int index_h = floor((dst_h - tmp_h) / 2.0);
    memcpy(dst.data + index_h * dst_w * 3, tmp.data, tmp_w * tmp_h * 3);
    effect_area.x = 0;
    effect_area.y = index_h;
    effect_area.width = tmp_w;
    effect_area.height = tmp_h;
  } else {
    printf("error\n");
  }
  return 0;
}

void GetNum::drawBboxes(const cv::Mat &bgr, const std::vector<BoxInfo> &bboxes,
                 object_rect effect_roi) {
  static const char *class_names[] = {
      "G",      "5",
      "4",    "3",     "2",
      "1",
 };

  cv::Mat image = bgr.clone();
  int src_w = image.cols;
  int src_h = image.rows;
  int dst_w = effect_roi.width;
  int dst_h = effect_roi.height;
  float width_ratio = (float)src_w / (float)dst_w;
  float height_ratio = (float)src_h / (float)dst_h;


  for (size_t i = 0; i < bboxes.size(); i++) {
    const BoxInfo &bbox = bboxes[i];
    cv::Scalar color =
        cv::Scalar(color_list[bbox.label][0], color_list[bbox.label][1],
                   color_list[bbox.label][2]);
    cv::rectangle(image,
                  cv::Rect(cv::Point((bbox.x1 - effect_roi.x) * width_ratio,
                                     (bbox.y1 - effect_roi.y) * height_ratio),
                           cv::Point((bbox.x2 - effect_roi.x) * width_ratio,
                                     (bbox.y2 - effect_roi.y) * height_ratio)),
                  color);

    score = bbox.score;
    label = class_names[bbox.label];

    char text[256];
    sprintf(text, "%s %.1f%%", class_names[bbox.label], bbox.score * 100);
    int baseLine = 0;
    cv::Size label_size =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
    int x = (bbox.x1 - effect_roi.x) * width_ratio;
    int y =
        (bbox.y1 - effect_roi.y) * height_ratio - label_size.height - baseLine;
    if (y < 0)
      y = 0;
    if (x + label_size.width > image.cols)
      x = image.cols - label_size.width;

    cv::rectangle(image, cv::Rect(cv::Point(x, y),
                                  cv::Size(label_size.width,
                                           label_size.height + baseLine)),
                  color, -1);
    cv::putText(image, text, cv::Point(x, y + label_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
  }

//#ifdef DEBUG_MODE
//  cv::imshow("image", image);
//#endif
}

int GetNum::numClassifier(PicoDet &detector, cv::Mat result_image) {

    cv::Mat image = result_image;
    if (image.empty()) {
      return -1;
    }
    object_rect effect_roi;
    cv::Mat resized_img;
    resizeUniform(image, resized_img, cv::Size(64, 64),
                   effect_roi);
    auto results = detector.detect(resized_img, 0.1 * main_setting->digit_param.min_confidence, 0.1 * main_setting->digit_param.min_param);
    drawBboxes(image, results, effect_roi);

  return 0;
}

std::string GetNum::strikedPriority()
{
    if(score > main_setting->digit_param.min_score * 0.01)
    {
        if(label == "2")
            this->label = "2";
    }
    else
    {
        label = "0";
    }
    return label;
}


int GetNum::digitDetetor(cv::Mat &src,std::vector<ArmorInfo> &armor_all_list,int &filter_num,int &max_id,std::string &label_figure)
{
    label_figure = "";

#ifndef USE_DIGIT_FIGURE

    float s_max = -9999.f;
    if(filter_num > 0)
    {
        //std::vector<float>w_list = {0,0,0,9999,0,0};  // 高度，平行度, 长宽比，面积，跟上一帧距离（量纲不统一）
        for(int i = 0; i < armor_all_list.size(); i++)
        {
            //float s = ArmorDetector::addByWeight(armor_all_list[i].s_list, w_list);
            if(std::abs(armor_all_list[i].s_list[5]) < s_max)
            {

                max_id = i;
                s_max = std::abs(armor_all_list[i].s_list[5]);
            }
        }
    }
#endif


#ifdef USE_DIGIT_FIGURE
    //数字识别优先级
        if(filter_num > 0 && armor_all_list.size() > 0)
        {
            std::vector<int> detector_num;
//            detector_num.clear();
            if(armor_all_list.size() >= 2)
            {
                for(int i = 0; i < armor_all_list.size(); i++)
                {
                    detector_num.push_back(i);
                }

                for(int i = 0; i < armor_all_list.size() - 1; i++)
                {
                    for(int j = 0; j < armor_all_list.size() - i - 1; j++)
                    {
                        if(std::abs(armor_all_list[j].s_list[5]) < std::abs(armor_all_list[j+1].s_list[5]))
                        {
                            int label_num = detector_num[j];
                            detector_num[j] = detector_num[j+1];
                            detector_num[j+1] = label_num;
                        }
                    }
                }


                int k = 0;
                while(k < armor_all_list.size())
                {
                    max_id = detector_num[k];

                    initResult();
                    getArmorData(armor_all_list[max_id]);
//                        digit_figure = imgProc.getPredictResult(src,digit_bin,max_is_small);
                    NNgetPredictResult(src);
                    label_figure = strikedPriority();

                    if(label_figure != "0")
                        break;
                    else if(k == armor_all_list.size() - 1)
                        return false;
                    else
                        k++;
                }
            }


            else
            {
                max_id = 0;

                this->initResult();
                getArmorData(armor_all_list[max_id]);

                this->NNgetPredictResult(src);
                label_figure = this->strikedPriority();

                if(label_figure == "0")
                    return false;
            }
    }
#endif
    max_score = score;
    max_strike_value = 1;

    return true;
}

bool GetNum::getBestArmor(int &max_id,std::string &best_label,std::vector<std::string> &label_list,std::vector<float> &score_list,std::vector<int> &index_list)
{
    this->max_score = 0;
    this->max_strike_value = 0;
    for(int i = 0;i < index_list.size();i++)
    {
        float weight , current_score;
        if(label_list[i] == "1")
            weight = 1.25;
        else if(label_list[i] == "2" || label_list[i] == "G")
            weight = 0.5;
        else
            weight = 1;

        current_score = weight * score_list[i];

        if(current_score > this->max_strike_value)
        {
            this->max_score = score_list[i];
            this->max_strike_value = current_score;
            best_label = label_list[i];
            max_id = index_list[i];
        }
    }

    if(this->max_strike_value > main_setting->digit_param.min_strike_value * 0.01)
        return true;
    return false;
}

int GetNum::allDigitDetector(cv::Mat &src,std::vector<ArmorInfo> &armor_all_list,int &filter_num,int &max_id,std::string &label_figure)
{
    label_figure = "";

#ifndef USE_DIGIT_FIGURE

    float s_max = -9999.f;
    if(filter_num > 0)
    {
        //std::vector<float>w_list = {0,0,0,9999,0,0};  // 高度，平行度, 长宽比，面积，跟上一帧距离（量纲不统一）
        for(int i = 0; i < armor_all_list.size(); i++)
        {
            //float s = ArmorDetector::addByWeight(armor_all_list[i].s_list, w_list);
            if(std::abs(armor_all_list[i].s_list[5]) < s_max)
            {

                max_id = i;
                s_max = std::abs(armor_all_list[i].s_list[5]);
            }
        }
    }
#endif


#ifdef USE_DIGIT_FIGURE
    //数字识别优先级
        if(filter_num > 0 && armor_all_list.size() > 0)
        {
            if(armor_all_list.size() >= 2)
            {
                std::vector<std::string> label_list;
                std::vector<float> score_list;
                std::vector<int> index_list;
                for(int i = 0;i < armor_all_list.size();i ++)
                {
                    initResult();
                    getArmorData(armor_all_list[i]);
                    NNgetPredictResult(src);       //神经网络分类函数
                    if(this->label.size()==0)
                        continue;
                    if(this->score > main_setting->digit_param.min_score*0.01)       //获取高置信度标签信息
                    {
                        label_list.push_back(label);
                        score_list.push_back(score);
                        index_list.push_back(i);
                    }
                }
                if(!this->getBestArmor(max_id,label_figure,label_list,score_list,index_list))       //获取最优装甲板
                    return false;
            }
            else
            {
                max_id = 0;

                this->initResult();
                getArmorData(armor_all_list[max_id]);
                this->NNgetPredictResult(src);      //神经网络分类函数
                label_figure = this->strikedPriority();
                this->max_score = this->score;
                this->max_strike_value = 1;

                if(label_figure == "0")
                    return false;
            }
    }
#endif

    return true;
}

