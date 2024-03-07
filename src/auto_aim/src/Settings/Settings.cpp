#include "auto_aim/Settings/Settings.h"

Settings::Settings()
{

}

ArmorSettings::ArmorSettings(const char *param)
{
    this->readArmorParam(param);
}

MainSettings::MainSettings(const char *param)
{
    this->readOtherParam(param);
}



void MainSettings::readOtherParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::READ);

    fs["main_mode"] >> this->main_mode;
    fs["enemy_color"] >> this->enemy_color;
    fs["debug-b_show_src"] >> this->debug.b_show_src;
    fs["debug-b_show_bin"] >> this->debug.b_show_bin;
    fs["debug-b_thre_mode"] >> this->debug.b_thre_mode;
    fs["debug-b_dist_mode"] >> this->debug.b_dist_mode;
    fs["debug-b_show_digit_bin"] >> this->debug.b_show_digit_bin;
    fs["debug-b_show_digit_roi"] >> this->debug.b_show_digit_roi;
    fs["debug-b_show_target"] >> this->debug.b_show_target;
    fs["debug-b_show_armor"] >> this->debug.b_show_armor;
    fs["debug-b_show_result"] >> this->debug.b_show_result;
    fs["debug-b_show_fps"] >> this->debug.b_show_fps;
    fs["debug-b_save_result"] >> this->debug.b_save_result;
    fs["debug-n_save_result"] >> this->debug.n_save_result;
    fs["debug-b_save_pic"] >> this->debug.b_save_pic;
    fs["debug-b_save_digit_bin"] >> this->debug.b_save_digit_bin;
    fs["debug-b_save_digit_roi_pic"] >> this->debug.b_save_digit_roi_pic;
    fs["debug-f_save_pic_inter"] >> this->debug.f_save_pic_inter;
    fs["debug-expore_time"] >> this->debug.expore_time;
    fs["debug-b_show_dc"] >> this->debug.b_show_dc;
    fs["camera_param-x"] >> this->camera_param.x;
    fs["camera_param-y"] >> this->camera_param.y;
    fs["camera_param-z"] >> this->camera_param.z;
    fs["camera_param-z_ptz2bul"] >> this->camera_param.z_offset_ptz2bul;
    fs["camera_param-y_offset"] >> this->camera_param.y_offset;
    fs["camera_param-bullet_speed"] >> this->camera_param.bullet_speed;

    fs["min_confidence"] >> digit_param.min_confidence;
    fs["min_param"] >> digit_param.min_param;
    fs["min_score"] >> digit_param.min_score;
    fs["min_strike_value"] >> digit_param.min_strike_value;

#ifdef DEBUG_MODE
    std::cout << "Read other param finished!" << std::endl;
#endif
    fs.release();
}

void MainSettings::setMainParam(const char *win_name)
{
    static int is_shut_down_main = 0;
    static int is_shut_down_show = 1;
    static int is_shut_down_save = 1;

    if(is_shut_down_main)
    {
        if(-1 != cv::getWindowProperty(win_name, 1))
            cv::destroyWindow(win_name);
        if(-1!=cv::getWindowProperty("展示调试窗口", 1))
            cv::destroyWindow("展示调试窗口");
        if(-1!=cv::getWindowProperty("调整曝光参数等", 1))
            cv::destroyWindow("调整曝光参数等");
        return;
    }

    cv::namedWindow(win_name);
    cv::createTrackbar("main_mode", win_name, &this->main_mode, 5);
    cv::createTrackbar("enemy_color", win_name, &this->enemy_color, 1);
    //cv::createTrackbar("is_shut_down_main", win_name, &is_shut_down_main, 1);
    cv::createTrackbar("is_shut_down_other", win_name, &is_shut_down_show, 1);
    cv::createTrackbar("is_shut_down_save", win_name, &is_shut_down_save, 1);

    if(is_shut_down_save)
    {
        if(-1 != cv::getWindowProperty("调整曝光参数等",1))
            cv::destroyWindow("调整曝光参数等");
    }
    else
    {
        cv::namedWindow("调整曝光参数等");
        cv::createTrackbar("debug-b_save_result", "调整曝光参数等", &debug.b_save_result, 1);
        cv::createTrackbar("debug-b_save_pic", "调整曝光参数等", &debug.b_save_pic, 1);
        cv::createTrackbar("debug-b_save_digit_bin", "调整曝光参数等", &debug.b_save_digit_bin, 1);
        cv::createTrackbar("debug-b_save_digit_roi_pic", "调整曝光参数等", &debug.b_save_digit_roi_pic, 1);
        cv::createTrackbar("debug-f_save_pic_inter", "调整曝光参数等", &debug.f_save_pic_inter, 10000);
        cv::createTrackbar("debug-expore_time", "调整曝光参数等", &debug.expore_time, 100000);
    }

    if(is_shut_down_show)
    {
        if(-1 != cv::getWindowProperty("展示调试窗口", 1))
            cv::destroyWindow("展示调试窗口");
    }
    else
    {
        cv::namedWindow("展示调试窗口");
        cv::createTrackbar("debug-b_show_src", "展示调试窗口", &debug.b_show_src, 1);
        cv::createTrackbar("debug-b_show_bin", "展示调试窗口", &debug.b_show_bin, 1);
        cv::createTrackbar("debug-b_show_digit_bin", "展示调试窗口", &debug.b_show_digit_bin, 1);
        cv::createTrackbar("debug-b_show_digit_roi", "展示调试窗口", &debug.b_show_digit_roi, 1);
        cv::createTrackbar("debug-b_show_target", "展示调试窗口", &debug.b_show_target, 1);
        cv::createTrackbar("debug-b_show_armor", "展示调试窗口", &debug.b_show_armor, 1);
        cv::createTrackbar("debug-b_show_result", "展示调试窗口", &debug.b_show_result, 1);
        cv::createTrackbar("debug-b_show_ex_armor", "展示调试窗口", &debug.b_show_ex_armor, 1);
        cv::createTrackbar("debug-b_show_fps", "展示调试窗口", &debug.b_show_fps, 1);
        cv::createTrackbar("debug-b_show_dc", "展示调试窗口", &debug.b_show_dc, 1);
    }
}
//自描模式预处理
//预处理模式选择
bool Settings::setTredMode(const char *win_name, MainSettings *main_setting){
    if(main_setting->debug.b_show_bin)
        cv::namedWindow(win_name);
    else
    {
        if(-1 != cv::getWindowProperty(win_name, 1))
            cv::destroyWindow(win_name);
        return false;
    }
    cv::createTrackbar("Mode_number",win_name, &main_setting->debug.b_thre_mode, 2);
    cv::createTrackbar("Dist_number",win_name, &main_setting->debug.b_dist_mode, 2);
    return true;
}

bool Settings::setPreprocessParam(const char *win_name)
{  //0
    cv::namedWindow(win_name);
    cv::createTrackbar("gray_thre_min",win_name,&preprocess_param.gray_thre_min, 255);
    cv::createTrackbar("gray_thre_max",win_name,&preprocess_param.gray_thre_max, 255);
    cv::createTrackbar("gray_max_w",win_name,&preprocess_param.gray_max_w, 10);
    cv::createTrackbar("gray_avg_w",win_name,&preprocess_param.gray_avg_w, 10);
    //1
    cv::createTrackbar("BlueR_Dvalue_max", win_name, &preprocess_param.BlueR_Dvalue_max, 255);
    cv::createTrackbar("BlueR_Dvalue_min", win_name, &preprocess_param.BlueR_Dvalue_min, 255);
    cv::createTrackbar("Blue_greenvalue", win_name, &preprocess_param.Blue_greenvalue, 255);

    cv::createTrackbar("RedB_Dvalue_max", win_name, &preprocess_param.RedB_Dvalue_max, 255);
    cv::createTrackbar("RedB_Dvalue_min", win_name, &preprocess_param.RedB_Dvalue_min, 255);
    cv::createTrackbar("Red_greenvalue", win_name, &preprocess_param. Red_greenvalue, 255);
    return true;
}




bool Settings::setPreprocessParam(const char *win_name, MainSettings *main_setting)
{
    if(main_setting->debug.b_thre_mode == 1){
        cv::namedWindow("正片叠底参数");
        cv::createTrackbar("r_gray_thre_min","正片叠底参数",&preprocess_param.r_gray_thre_min, 255);
        cv::createTrackbar("b_gray_thre_min","正片叠底参数",&preprocess_param.b_gray_thre_min, 255);
        cv::createTrackbar("gray_thre_max", "正片叠底参数", &preprocess_param.gray_thre_max, 255);
        cv::createTrackbar("r_gray_max_w","正片叠底参数",&preprocess_param.r_gray_max_w, 10);
        cv::createTrackbar("b_gray_max_w","正片叠底参数",&preprocess_param.b_gray_max_w, 10);
        cv::createTrackbar("gray_avg_w", "正片叠底参数", &preprocess_param.gray_avg_w, 10);
    }
    else
    {
        if(-1 != cv::getWindowProperty("正片叠底参数", 1))
            cv::destroyWindow("正片叠底参数");
    }
    if(main_setting->debug.b_thre_mode == 2){

        cv::namedWindow("通道相减参数");
        cv::createTrackbar("BlueR_Dvalue_max", "通道相减参数", &preprocess_param.BlueR_Dvalue_max, 255);
        cv::createTrackbar("BlueR_Dvalue_min", "通道相减参数", &preprocess_param.BlueR_Dvalue_min, 255);
        cv::createTrackbar("Blue_greenvalue", "通道相减参数", &preprocess_param.Blue_greenvalue, 255);
        cv::createTrackbar("RedB_Dvalue_max", "通道相减参数", &preprocess_param.RedB_Dvalue_max, 255);
        cv::createTrackbar("RedB_Dvalue_min", "通道相减参数", &preprocess_param.RedB_Dvalue_min, 255);
        cv::createTrackbar("Red_greenvalue", "通道相减参数", &preprocess_param.Red_greenvalue, 255);
    }
    else
    {
        if(-1 != cv::getWindowProperty("通道相减参数", 1))
            cv::destroyWindow("通道相减参数");
    }
    return true;
}


bool Settings::setTgtSizeParam(const char *win_name, MainSettings *main_setting)
{
    if(main_setting->debug.b_show_target)
        cv::namedWindow(win_name);
    else
    {
        if(-1 != cv::getWindowProperty(win_name, 1))
            cv::destroyWindow(win_name);
        return false;
    }
    cv::createTrackbar("slope_offset*0.1", win_name, &tgt_size_param.slope_offset, 300);
    cv::createTrackbar("len_min", win_name,&tgt_size_param.len_min, 200);
    cv::createTrackbar("len_max", win_name, &tgt_size_param.len_max, 1000);
    cv::createTrackbar("ratio_min", win_name, &tgt_size_param.ratio_min, 30);
    cv::createTrackbar("ratio_max", win_name, &tgt_size_param.ratio_max, 30);
    cv::createTrackbar("area_min", win_name, &tgt_size_param.area_min, 1000);
    cv::createTrackbar("area_max", win_name, &tgt_size_param.area_max, 10000);
    cv::createTrackbar("area_ratio_min", win_name, &tgt_size_param.area_ratio_min, 100);
    cv::createTrackbar("area_ratio_max", win_name, &tgt_size_param.area_ratio_max, 100);
    cv::createTrackbar("area_len_ratio_min", win_name, &tgt_size_param.area_len_ratio_min, 100);
    cv::createTrackbar("area_len_ratio_max", win_name, &tgt_size_param.area_len_ratio_max, 100);
    cv::createTrackbar("corners_size_min", win_name, &tgt_size_param.corners_size_min, 10);
    //cv::createTrackbar("color_th", win_name, &tgt_size_param.color_th, 500);
    cv::createTrackbar("color_th_r", win_name, &tgt_size_param.color_th_r, 800);
    cv::createTrackbar("color_th_b", win_name, &tgt_size_param.color_th_b, 800);
    //cv::createTrackbar("ellipse_light_angle", win_name, &tgt_size_param.ellipse_light_angle, 9000);
    return true;
}


bool ArmorSettings::setArmorParam(MainSettings *main_setting)
{
    setTredMode("Thre_Mode", main_setting);
    setPreprocessParam(WIN_NAME_ARMORP_PREPROCESS, main_setting);
    setTgtSizeParam(WIN_NAME_ARMORP_TARGET, main_setting);

    if(main_setting->debug.b_show_armor)
        cv::namedWindow(WIN_NAME_ARMORP);
    else
    {
        if(-1 != cv::getWindowProperty(WIN_NAME_ARMORP, 1))
            cv::destroyWindow(WIN_NAME_ARMORP);
        return false;
    }
    cv::createTrackbar("delta_height_max", WIN_NAME_ARMORP, &armor_param.delta_height_max, 600);
    cv::createTrackbar("delta_angle_max", WIN_NAME_ARMORP, &armor_param.delta_angle_max, 500);
    cv::createTrackbar("delta_ellipse_angle_max", WIN_NAME_ARMORP, &armor_param.delta_ellipse_angle_max, 900);
    cv::createTrackbar("delta_w_ratio_min", WIN_NAME_ARMORP, &armor_param.delta_w_ratio_min, 200);
    cv::createTrackbar("delta_w_ratio_max", WIN_NAME_ARMORP, &armor_param.delta_w_ratio_max, 200);
    cv::createTrackbar("delta_h_max", WIN_NAME_ARMORP, &armor_param.delta_h_max, 1500);
    cv::createTrackbar("armor_hw_ratio_min", WIN_NAME_ARMORP, &armor_param.armor_hw_ratio_min, 100);
    cv::createTrackbar("armor_hw_ratio_max", WIN_NAME_ARMORP, &armor_param.armor_hw_ratio_max, 100);
    cv::createTrackbar("armor_ex_num", WIN_NAME_ARMORP, &armor_param.armor_ex_num, 100);
    cv::createTrackbar("min_armor_area", WIN_NAME_ARMORP, &armor_param.min_armor_area, 1000);
    cv::createTrackbar("max_armor_area", WIN_NAME_ARMORP, &armor_param.max_armor_area, 60000);
    //cv::createTrackbar("ellipse_angle", WIN_NAME_ARMORP, &armor_param.ellipse_angle, 18000);
    cv::createTrackbar("contour_armor", WIN_NAME_ARMORP, &armor_param.contour_armor, 1);
    return true;
}

void ArmorSettings::readArmorParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::READ);

    fs["armor_height"] >> armor_height;
    fs["armor_width"] >> armor_width;
    fs["small_armor_height"] >> small_armor_height;
    fs["small_armor_width"] >> small_armor_width;

    fs["gray_thre_min"] >> preprocess_param.gray_thre_min;
    fs["r_gray_thre_min"] >> preprocess_param.r_gray_thre_min;
    fs["b_gray_thre_min"] >> preprocess_param.b_gray_thre_min;
    fs["gray_thre_max"] >> preprocess_param.gray_thre_max;
    fs["gray_max_w"] >> preprocess_param.gray_max_w;
    fs["r_gray_max_w"] >> preprocess_param.r_gray_max_w;
    fs["b_gray_max_w"] >> preprocess_param.b_gray_max_w;
    fs["gray_avg_w"] >> preprocess_param.gray_avg_w;

    fs["BlueR_Dvalue_max"] >> preprocess_param.BlueR_Dvalue_max;
    fs["BlueR_Dvalue_min"] >> preprocess_param.BlueR_Dvalue_min;
    fs["Blue_greenvalue"] >> preprocess_param.Blue_greenvalue;
    fs["Red_greenvalue"] >> preprocess_param.Red_greenvalue;
    fs["RedB_Dvalue_max"] >> preprocess_param.RedB_Dvalue_max;
    fs["RedB_Dvalue_min"] >> preprocess_param.RedB_Dvalue_min;


    fs["len_min"] >> tgt_size_param.len_min;
    fs["len_max"] >> tgt_size_param.len_max;
    fs["light_ratio_min"] >> tgt_size_param.ratio_min;
    fs["light_ratio_max"] >> tgt_size_param.ratio_max;
    fs["light_area_min"] >> tgt_size_param.area_min;
    fs["light_area_max"] >> tgt_size_param.area_max;
    fs["area_ratio_min"] >> tgt_size_param.area_ratio_min;
    fs["area_ratio_max"] >> tgt_size_param.area_ratio_max;
    fs["area_len_ratio_min"] >> tgt_size_param.area_len_ratio_min;
    fs["area_len_ratio_max"] >> tgt_size_param.area_len_ratio_max;
    fs["corners_size_min"] >> tgt_size_param.corners_size_min;
    fs["slope_offset"] >> tgt_size_param.slope_offset;
    //fs["color_th"] >> tgt_size_param.color_th;
    fs["color_th_r"] >> tgt_size_param.color_th_r;
    fs["color_th_b"] >> tgt_size_param.color_th_b;
    //fs["ellipse_light_angle"] >> tgt_size_param.ellipse_light_angle;

    fs["delta_height_max"] >> armor_param.delta_height_max;
    fs["delta_angle_max"] >> armor_param.delta_angle_max;
    fs["delta_ellipse_angle_max"] >> armor_param.delta_ellipse_angle_max;

    fs["delta_w_ratio_min"] >> armor_param.delta_w_ratio_min;
    fs["delta_w_ratio_max"] >> armor_param.delta_w_ratio_max;
    fs["delta_h_max"] >> armor_param.delta_h_max;

    fs["armor_hw_ratio_min"] >> armor_param.armor_hw_ratio_min;
    fs["armor_hw_ratio_max"] >> armor_param.armor_hw_ratio_max;

    fs["armor_ex_num"] >> armor_param.armor_ex_num;
    fs["min_armor_area"] >> armor_param.min_armor_area;
    fs["max_armor_area"] >> armor_param.max_armor_area;
    //fs["ellipse_angle"] >> armor_param.ellipse_angle;
    fs["contour_in_armor"] >> armor_param.contour_armor;

#ifdef DEBUG_MODE
    std::cout << "Read armor param finished!" << std::endl;
#endif
    fs.release();
}

void MainSettings::writeOtherParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::WRITE);

    fs << "main_mode" << this->main_mode;
    fs << "enemy_color" << this->enemy_color;
    fs << "debug-b_show_src" << this->debug.b_show_src;
    fs << "debug-b_show_bin" << this->debug.b_show_bin;
    fs << "debug-b_thre_mode" << this->debug.b_thre_mode;
    fs << "debug-b_dist_mode" << this->debug.b_dist_mode;
    fs << "debug-b_show_digit_bin" << this->debug.b_show_digit_bin;
    fs << "debug-b_show_digit_roi" << this->debug.b_show_digit_roi;
    fs << "debug-b_show_target" << this->debug.b_show_target;
    fs << "debug-b_show_armor" << this->debug.b_show_armor;
    fs << "debug-b_show_result" << this->debug.b_show_result;
    fs << "debug-b_show_fps" << this->debug.b_show_fps;
    fs << "debug-b_save_result" << this->debug.b_save_result;
    fs << "debug-n_save_result" << this->debug.n_save_result;
    fs << "debug-b_save_pic" << this->debug.b_save_pic;
    fs << "debug-b_save_digit_bin" << this->debug.b_save_digit_bin;
    fs << "debug-b_save_digit_roi_pic" << this->debug.b_save_digit_roi_pic;
    fs << "debug-f_save_pic_inter" << this->debug.f_save_pic_inter;
    fs << "debug-expore_time" << this->debug.expore_time;
    fs << "debug-b_show_dc" << this->debug.b_show_dc;

    fs << "camera_param-x" << this->camera_param.x;
    fs << "camera_param-y" << this->camera_param.y;
    fs << "camera_param-z" << this->camera_param.z;
    fs << "camera_param-y_offset" << this->camera_param.y_offset;
    fs << "camera_param-z_ptz2bul" << this->camera_param.z_offset_ptz2bul;
    fs << "camera_param-bullet_speed" << this->camera_param.bullet_speed;

    fs << "port_param-dev_name" << this->port_param.dev_name;
    fs << "port_param-baud_rate" << this->port_param.baud_rate;
    fs << "port_param-databits" << this->port_param.databits;
    fs << "port_param-stopbits" << this->port_param.stopbits;
    fs << "port_param-parity" << this->port_param.parity;

    fs << "min_confidence" << digit_param.min_confidence;
    fs << "min_param" << digit_param.min_param;
    fs << "min_score" << digit_param.min_score;
    fs << "min_strike_value" << digit_param.min_strike_value;

    std::cout << "Sava other param finished!" << std::endl;
    fs.release();
}

void ArmorSettings::writeArmorParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::WRITE);

    fs << "armor_height" << armor_height;
    fs << "armor_width" << armor_width;
    fs << "small_armor_height" << small_armor_height;
    fs << "small_armor_width" << small_armor_width;

    fs << "gray_thre_min" << preprocess_param.gray_thre_min;
    fs << "r_gray_thre_min" << preprocess_param.r_gray_thre_min;
    fs << "b_gray_thre_min" << preprocess_param.b_gray_thre_min;
    fs << "gray_thre_max" << preprocess_param.gray_thre_max;
    fs << "gray_max_w" << preprocess_param.gray_max_w;
    fs << "r_gray_max_w" << preprocess_param.r_gray_max_w;
    fs << "b_gray_max_w" << preprocess_param.b_gray_max_w;
    fs << "gray_avg_w" << preprocess_param.gray_avg_w;

    fs << "BlueR_Dvalue_max" << preprocess_param.BlueR_Dvalue_max;
    fs << "BlueR_Dvalue_min" << preprocess_param.BlueR_Dvalue_min;
    fs << "Blue_greenvalue" << preprocess_param.Blue_greenvalue;
    fs << "Red_greenvalue" << preprocess_param.Red_greenvalue;
    fs << "RedB_Dvalue_max" << preprocess_param.RedB_Dvalue_max;
    fs << "RedB_Dvalue_min" << preprocess_param.RedB_Dvalue_min;

    fs << "len_min" << tgt_size_param.len_min;
    fs << "len_max" << tgt_size_param.len_max;
    fs << "light_ratio_min" << tgt_size_param.ratio_min;
    fs << "light_ratio_max" << tgt_size_param.ratio_max;
    fs << "light_area_min" << tgt_size_param.area_min;
    fs << "light_area_max" << tgt_size_param.area_max;
    fs << "area_ratio_min" << tgt_size_param.area_ratio_min;
    fs << "area_ratio_max" << tgt_size_param.area_ratio_max;
    fs << "area_len_ratio_min" << tgt_size_param.area_len_ratio_min;
    fs << "area_len_ratio_max" << tgt_size_param.area_len_ratio_max;
    fs << "corners_size_min" << tgt_size_param.corners_size_min;
    fs << "slope_offset" << tgt_size_param.slope_offset;
    //fs << "color_th" << tgt_size_param.color_th;
    fs << "color_th_r" << tgt_size_param.color_th_r;
    fs << "color_th_b" << tgt_size_param.color_th_b;
    //fs << "ellipse_light_angle" << tgt_size_param.ellipse_light_angle;

    fs << "delta_height_max" << armor_param.delta_height_max;
    fs << "delta_angle_max" << armor_param.delta_angle_max;
    fs << "delta_ellipse_angle_max" << armor_param.delta_ellipse_angle_max;

    fs << "delta_w_ratio_min" << armor_param.delta_w_ratio_min;
    fs << "delta_w_ratio_max" << armor_param.delta_w_ratio_max;
    fs << "delta_h_max" << armor_param.delta_h_max;

    fs << "armor_hw_ratio_min" << armor_param.armor_hw_ratio_min;
    fs << "armor_hw_ratio_max" << armor_param.armor_hw_ratio_max;

    fs << "armor_ex_num" << armor_param.armor_ex_num;
    fs << "min_armor_area" << armor_param.min_armor_area;
    fs << "max_armor_area" << armor_param.max_armor_area;
    //fs << "ellipse_angle" << armor_param.ellipse_angle;
    fs << "contour_in_armor" << armor_param.contour_armor;

    std::cout << "Save armor param finished!" << std::endl;
    fs.release();
}

void MainSettings::setCameraParam(const char *win_name)
{
    cv::namedWindow(win_name);
    static int x = (camera_param.x * 100.);
    cv::createTrackbar("cp-x", win_name, &x, 1000);
    camera_param.x = (x / 100.);

    static int y = camera_param.y * 100.;
    cv::createTrackbar("cp-y", win_name, &y, 1000);
    camera_param.y = y / 100.;

    static int z = camera_param.z * 100.;
    cv::createTrackbar("cp-z", win_name, &z, 3000);
    camera_param.z = z / 100.;

    static int z_offset_ptz2bul = camera_param.z_offset_ptz2bul * 100.;
    cv::createTrackbar("cp-y_offset", win_name, &z_offset_ptz2bul, 3000);
    camera_param.z_offset_ptz2bul = z_offset_ptz2bul / 100.;

    static int y_offset = camera_param.y_offset * 100.;
    cv::createTrackbar("cp-y_offset", win_name, &y_offset, 1000);
    camera_param.y_offset = y_offset / 100.;

    static int overlap_dist = camera_param.overlap_dist * 1.;
    cv::createTrackbar("cp-overlap_dist", win_name, &overlap_dist, 1000000);
    camera_param.overlap_dist = overlap_dist / 1.;

    static int bullet_speed = camera_param.bullet_speed * 100.;
    cv::createTrackbar("cp-bullet_speed", win_name, &bullet_speed, 3000);
    camera_param.bullet_speed = bullet_speed / 100.;
}

bool MainSettings::digitClassiferParam()
{
//    if(debug.b_show_result)
//    {
//        cv::namedWindow("数字分类器参数设置");
//    }
//    else
//    {
//        if(-1!=cv::getWindowProperty("数字分类器参数设置",1))
//            cv::destroyWindow("数字分类器参数设置");
//        return false;
//    }
    cv::namedWindow("数字分类器参数设置");
    cv::createTrackbar("min_confidence","数字分类器参数设置",&digit_param.min_confidence,10);
    cv::createTrackbar("min_param","数字分类器参数设置",&digit_param.min_param,10);
    cv::createTrackbar("min_score","数字分类器参数设置",&digit_param.min_score,100);
    cv::createTrackbar("min_strike_value","数字分类器参数设置",&digit_param.min_strike_value,100);

    return true;
}

bool MainSettings::grabImg(cv::Mat &img, char order, long interval)
{
    cv::Mat img_show = img.clone();
//    imshow("img_show",img_show);
    //1 s
    //=1000       ms = 10e3 ms
    //=1000000    us = 10e6 us
    //=1000000000 ns = 10e9 ns
    static long start;
    static int img_filename;
    if(cv::getTickCount() - start >= interval)
    {
        start=cv::getTickCount();

        std::ostringstream s;
        s << SAVE_PIC_DIR << ++img_filename << ".png";
        cv::imwrite(s.str(), img);
        img_show = -img_show;
    }
    return true;
}
