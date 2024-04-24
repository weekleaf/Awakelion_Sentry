#include "auto_aim/Settings/Settings.h"

MainSettings::MainSettings(const char *param1,const char *param2)
{
    this->readOtherParam(param1);
    this->readCameraParam(param2);
}

void MainSettings::setMainParam(const char *win_name)
{
    cv::namedWindow(win_name);
    cv::createTrackbar("main_mode",win_name,&this->main_mode,5);
    cv::createTrackbar("enemy_color",win_name,&this->enemy_color,1);
    cv::createTrackbar("src",win_name,&this->debug.src,1);
    cv::createTrackbar("detect_armor",win_name,&this->debug.detect_armor,1);
    cv::createTrackbar("armor_chosen",win_name,&this->debug.armor_chosen,1);
    cv::createTrackbar("predict_point",win_name,&this->debug.predict_point,1);
    cv::createTrackbar("inaccuracy_point",win_name,&this->debug.inaccuracy_point,1);
    cv::createTrackbar("dc_pitch",win_name,&this->debug.dc_pitch,1);
    cv::createTrackbar("dc_yaw",win_name,&this->debug.dc_yaw,1);

}

void MainSettings::readOtherParam(const char *param_path)
{
//    cv::FileStorage fs(param_path, cv::FileStorage::READ);

//    fs["main_mode"] >> this->main_mode;
//    fs["enemy_color"] >> this->enemy_color;
//    fs["src"] >>this->debug.src;
//    fs["detect_armor"]>>this->debug.detect_armor;
//    fs["armor_chosen"]>>this->debug.armor_chosen;
//    fs["predict_point"]>>this->debug.predict_point;
//    fs["inaccuracy_point"]>>this->debug.inaccuracy_point;
//    fs["dc_pitch"]>>this->debug.dc_pitch;
//    fs["dc_yaw"]>>this->debug.dc_yaw;

//#ifdef DEBUG_MODE
//    std::cout << "Read other param finished!" << std::endl;
//#endif
//    fs.release();
}

void MainSettings::writeOtherParam(const char *param_path)
{
//    cv::FileStorage fs(param_path, cv::FileStorage::WRITE);

//    fs<<"main_mode" << this->main_mode;
//    fs<<"enemy_color" << this->enemy_color;
//    fs<<"src" << this->debug.src;
//    fs<<"detect_armor"<<this->debug.detect_armor;
//    fs<<"armor_chosen"<<this->debug.armor_chosen;
//    fs<<"predict_point"<<this->debug.predict_point;
//    fs<<"inaccuracy_point"<<this->debug.inaccuracy_point;
//    fs<<"dc_pitch"<<this->debug.dc_pitch;
//    fs<<"dc_yaw"<<this->debug.dc_yaw;

//    std::cout << "Sava other param finished!" << std::endl;
//    fs.release();
}

void MainSettings::setCameraParam(const char *win_name)
{
//    cv::namedWindow(win_name);
//    cv::createTrackbar("PCBD",win_name,this->camera_param.PCBD,20);
//    cv::createTrackbar("PBMD",win_name,this->camera_param.PBMD,20);
//    cv::createTrackbar("YCBD",win_name,this->camera_param.YCBD,20);
//    cv::createTrackbar("YBMD",win_name,this->camera_param.YBMD,20);
}

void MainSettings::readCameraParam(const char *param_path)
{
//    cv::FileStorage fs(param_path, cv::FileStorage::READ);
//    fs["PCBD"]>>this->camera_param.PCBD;
//    fs["PBMD"]>>this->camera_param.PBMD;
//    fs["YCBD"]>>this->camera_param.YCBD;
//    fs["YBMD"]>>this->camera_param.YBMD;
//    std::cout << "Read CameraParam finished!" << std::endl;
//    fs.release();
}

void MainSettings::writeCameraParam(const char *param_path)
{
//    cv::FileStorage fs(param_path, cv::FileStorage::WRITE);

//    fs<<"PCBD" << this->camera_param.PCBD;
//    fs<<"PBMD" << this->camera_param.PBMD;
//    fs<<"YCBD" << this->camera_param.YCBD;
//    fs<<"YBMD" << this->camera_param.YBMD;

//    std::cout << "Sava CameraParam finished!" << std::endl;
//    fs.release();
}
