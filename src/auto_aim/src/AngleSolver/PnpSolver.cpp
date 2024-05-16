#include "auto_aim/AngleSolver/PnpSolver.h"

AngleSolver::AngleSolver(const char *camera_param_file_name, double z_scale)
{
    rectPnpSolver(camera_param_file_name);
    scale_z = z_scale;

    rot_camera2ptz = cv::Mat::eye(3, 3, CV_64FC1);
    trans_camera2ptz = cv::Mat::zeros(3, 1, CV_64FC1);
    offset_y_barrel_ptz = 0;
}

AngleSolver::AngleSolver()
{

}
void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect &rect,
    std::vector<cv::Point2f> &target2d,
    const cv::Point2f &offset)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}

void AngleSolver::rectPnpSolver(const char *camera_param_file_name)
{
    cv::FileStorage fs(camera_param_file_name, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "Could not open the configuration file" << std::endl;

    fs["Camera_Matrix"] >> cam_matrix;
    fs["Distortion_Coefficients"] >> distortion_coeff;
    fs["board_Width"] >> width_target;
    fs["board_Height"] >> height_target;

    if(cam_matrix.empty() || distortion_coeff.empty())
    {
        std::cout << "cam_matrix or distortion_coeff is empty!!!" << std::endl;
        return;
    }

//    //根据目标矩形的宽高设置三维坐标
//    double half_x = width_target / 2.0;
//    double half_y = height_target / 2.0;

//    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
//    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
//    point3d.push_back(cv::Point3f(half_x, half_y, 0));
//    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
}

void AngleSolver::setTargetSize(double width, double height)
{
    width_target = width;
    height_target = height;

    //根据目标矩形的宽高设置三维坐标
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.clear();
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
}
void AngleSolver::setTargetSize(double lenth)//能量机关的点输出顺序不同，需改
{
    //根据目标矩形的宽高设置三维坐标
    double half=lenth/2.0;

    point3d.clear();
    point3d.push_back(cv::Point3f(-half, -half, 0));
    point3d.push_back(cv::Point3f(-half, half, 0));
    point3d.push_back(cv::Point3f(half, half, 0));
    point3d.push_back(cv::Point3f(-half, -half, 0));
}


bool AngleSolver::getAngle(cv::Point2f *target2d,Eigen::Vector3d &tvec,Eigen::Vector3d &rvec)
{
    //根据检测出的目标在图像中的二维坐标，算出旋转矩阵与位移向量
    std::vector<cv::Point2f> target2d_temp;
    for(int i = 0; i < 4; i++){
        target2d_temp.push_back(target2d[i]);
    }
    solvePnP4Points(target2d_temp,tvec,rvec);
    return true;
}

bool AngleSolver::getAngle(std::vector<cv::Point2f> target2d,Eigen::Vector3d &tvec)
{
    //根据检测出的目标在图像中的二维坐标，算出旋转矩阵与位移向量
    Eigen::Vector3d rvec;
    solvePnP4Points(target2d,tvec,rvec);
    return true;
}
void AngleSolver::adjustPTZ2Barrel(const cv::Mat &pos_in_ptz,
    double &angle_x, double &angle_y,
    double bullet_speed,
    double current_ptz_angle)
{
    const double *_xyz = (const double *)pos_in_ptz.data;
    angle_x = atan2(_xyz[0], _xyz[2]);

    double xyz[3] = { _xyz[0], _xyz[1], _xyz[2] };

    double alpha = 0.0, theta = 0.0;

    alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        angle_y = - alpha - theta;
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = -alpha + theta;
    }
    angle_x = angle_x * 180.0 / M_PI;
    angle_y = angle_y * 180.0 / M_PI;
}

void AngleSolver::solvePnP4Points(const std::vector<cv::Point2f> points2d, Eigen::Vector3d &trans,Eigen::Vector3d &rvec)
{
    cv::Mat tvec_temp;
    cv::Mat rvec_temp;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec_temp, tvec_temp,cv::SOLVEPNP_ITERATIVE); //自瞄
    trans<<tvec_temp.at<double>(0,0), tvec_temp.at<double>(1,0), tvec_temp.at<double>(2,0);
    rvec<<rvec_temp.at<double>(0,0), rvec_temp.at<double>(1,0), rvec_temp.at<double>(2,0);
}
/*2023赛季新单目测距*/
/*
    解决的问题：
    1.4m内测距误差5cm内
    2.5m内测距误差10cm内
    尚未解决的问题：
    1.左右倾斜不准
    2.超过6m测距不准
    3.装甲板偏移屏幕中心点不准
    4.像素点抖动导致测距不准
*/
/*pnp测距*/
void AngleSolver::getDistanceDanmuPnP(const std::vector<cv::Point2f> &points2d,
                                    double &dist){
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
//    std::cout << "2d点 = " << std::endl << points2d <<std::endl;

    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, position_in_camera, cv::SOLVEPNP_ITERATIVE);
    double x_pos =  position_in_camera.at<double>(0, 0);
    double y_pos =  position_in_camera.at<double>(1, 0);
    double z_pos =  position_in_camera.at<double>(2, 0);
//    dist = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
    dist = z_pos * 1.0;

    std::cout << "当前使用pnp测距" << std::endl;
    //std::cout << "旋转向量 = " << std::endl << rvec << std::endl;
    std::cout << "x_pos = " << x_pos << std::endl;
    std::cout << "y_pos = " << y_pos << std::endl;
    std::cout << "z_pos = " << z_pos << std::endl;
    //std::cout << "dist = " << dist << std::endl;

//    double angle_x = std::atan(x_pos/z_pos);
//    angle_x = angle_x * 180.0 / PI;
//    std::cout << "pnp: angle_x = " << angle_x << std::endl;
}

/*小孔成像测距new*/
void AngleSolver::getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist)
{
    cv::RotatedRect armor = cv::minAreaRect(armor_rect);
//    std::cout << "armor_rect = " << std::endl << armor_rect <<std::endl;

    float p_w = std::max(armor.size.width, armor.size.height);
    float p_h = std::min(armor.size.width, armor.size.height);

    float fx_w = cam_matrix.at<double>(0, 0) * width_target; //* SHOW_WIDTH / 750;    //750是因为标定时大小为750（600同理）
    float fx_h = cam_matrix.at<double>(1, 1) * height_target; //* SHOW_HEIGHT / 600;

    float dist_w = fx_w / p_w;
    float dist_h = fx_h / p_h;

    //装甲板倾斜的时候dist_w偏大,此处为30度时dist_w大于dist_h的距离
    if(dist_w - dist_h > 15  )
    {
        dist = dist_h;
    }
    else
    {
        //正对装甲板时，削弱高比例（高波动大，不准）
        dist = (4*dist_w + dist_h) / 5;
    }


    std::cout << "当前使用小孔成像测距"<<std::endl;
//    std::cout << "灯条像素宽 = " << p_w << std::endl;
//    std::cout << "灯条像素高 = " << p_h << std::endl;
//    std::cout << "fx_w = " << fx_w << std::endl;
//    std::cout << "fx_h = " << fx_h << std::endl;
    std::cout << "dist_w = " << dist_w << std::endl;
    std::cout << "dist_h = " << dist_h << std::endl;
    std::cout << "dist = " << dist << std::endl;

}


void AngleSolver::Camera2Moto(double moto_pitch, double moto_yaw , Eigen::Vector3d tvec,Eigen::Vector3d ctvec, double &moto_move_pitch, double &moto_move_yaw,double v,double g)
{
    //输出为相对角
#if (ANGLE==0)
    v*=100.0;
    g*=100.0;//输入用国际单位制，但是运算建议用量纲长度cm，时间用s即可
    moto_pitch/=(180.0/M_PI);//电控收发都是角度制，需要转成弧度制处理

    double z0=sqrt(tvec(1,0)*tvec(1,0)+tvec(0,0)*tvec(0,0));//水平距离
    double h=tvec(2,0)/*-PBMD*cos(temp_pitch)*/;
    double dist=z0/*+PBMD*sin(temp_pitch)*/;
    dist=cos(moto_pitch)*dist-sin(moto_pitch)*h;
    h=cos(moto_pitch)*h+sin(moto_pitch)*dist;
    double z=h;//y
    double y=dist;//这里的y是水平距离，z是竖直距离，不要搞混,因为公式太长不想改了...
     double k;//zy枪管
     //std::cout<<"z"<<z<<" "<<"y"<<y<<std::endl;
     double vz1,vy1,fly_time;
     fly_time = sqrt(1.0 / (g * g) * (g * z + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) - v * v) * -2.0);

     if (std::isnan(fly_time)) {
         k = z / y;
     }
     else {
         vz1 = +(g * (y * y) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - g * (z * z) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - (g * g) * z * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) + (v * v) * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         vy1 = -((g * g) * y * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) - (v * v) * y * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0 + g * y * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         k = vz1 / vy1;
     }//未加空阻
//     k=z/y;
      double angle=atan(k);
      double moto_to_pitch=angle;
    moto_move_pitch=moto_to_pitch-moto_pitch/*pitch*//*moto_to_pitch-moto_pitch*//*-abs(tvec(1,0))/tvec(1,0)*M_PI/2.0*/;//电控收角度制，发给他们前需要转成角度制
    moto_move_pitch*=(180.0/M_PI);


//    //以下都是yaw的与pitch无关
    moto_yaw+=180.0;
    moto_yaw/=(180.0/M_PI);//电控收发都是角度制，需要转成弧度制处理
//    std::cout<<"moto_yaw"<<moto_yaw<<std::endl;
    double _x2=tvec(0,0);
    double _z2=/*abs(tvec(2,0))/tvec(2,0)*sqrt(pow(tvec(2,0),2)+pow(tvec(1,0),2))*/tvec(1,0);//?????

//    //std::cout<<_x2<<" "<<_z2<<" "<<atan(_x2/_z2)<<std::endl;
    double moto_to_yaw=atan(_x2/_z2);
    moto_move_yaw=(moto_to_yaw)/*-moto_yaw*/;
    moto_move_yaw*=-(180.0/M_PI);//已转成角度制
#else
    v*=100.0;
    g*=100.0;//输入用国际单位制，但是运算建议用量纲长度cm，时间用s即可
    moto_pitch/=(180.0/M_PI);//电控收发都是角度制，需要转成弧度制处理

    double z0=sqrt(tvec(1,0)*tvec(1,0)+tvec(0,0)*tvec(0,0));//水平距离
    double h=tvec(2,0)/*-PBMD*cos(temp_pitch)*/;
    double dist=z0/*+PBMD*sin(temp_pitch)*/;
/////////////////////////////////
    double z=h;//y
    double y=dist;//这里的y是水平距离，z是竖直距离，不要搞混,因为公式太长不想改了...
    //y=sqrt(pow(x,2)+pow(y,2));
    //上述已转完坐标系

     double k;//zy枪管
     //std::cout<<"z"<<z<<" "<<"y"<<y<<std::endl;
     double vz1,vy1,fly_time;
     fly_time = sqrt(1.0 / (g * g) * (g * z + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) - v * v) * -2.0);

     if (std::isnan(fly_time)) {
         k = z / y;
     }
     else {
         vz1 = +(g * (y * y) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - g * (z * z) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - (g * g) * z * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) + (v * v) * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         vy1 = -((g * g) * y * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) - (v * v) * y * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0 + g * y * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         k = vz1 / vy1;

     }//未加空阻
//     k=z/y;
//////////////////////////////////////////////////////
      double angle=atan(k);
      double moto_to_pitch=angle;
      //std::cout<<"moto_to_pitch:"<<moto_to_pitch*(180.0/M_PI)<<std::endl;

    moto_move_pitch=moto_to_pitch/*pitch*//*moto_to_pitch-moto_pitch*//*-abs(tvec(1,0))/tvec(1,0)*M_PI/2.0*/;//电控收角度制，发给他们前需要转成角度制
    moto_move_pitch*=(180.0/M_PI);

//以下都是yaw的与pitch无关
    moto_yaw+=180.0;
    if(moto_yaw>=360) moto_yaw-=360.0;
    moto_yaw/=(180.0/M_PI);//电控收发都是角度制，需要转成弧度制处理
//    std::cout<<"moto_yaw"<<moto_yaw<<std::endl;

    double _x2=tvec(0,0);
    double _z2=/*abs(tvec(2,0))/tvec(2,0)*sqrt(pow(tvec(2,0),2)+pow(tvec(1,0),2))*/tvec(1,0);//?????

//    //std::cout<<_x2<<" "<<_z2<<" "<<atan(_x2/_z2)<<std::endl;

    double moto_to_yaw=atan(_z2/_x2);

   if(_z2<0&&_x2>0){
       moto_to_yaw+=M_PI/2.0*3.0;
       std::cout<<4<<std::endl;
   }
   else if(_z2<0&&_x2<0){
       moto_to_yaw+=M_PI/2.0 ;
       std::cout<<3<<std::endl;
   }

  else if(_z2>0&&_x2<0){
      moto_to_yaw+=M_PI/2.0;
     std::cout<<2<<std::endl;
  }
   else{
       moto_to_yaw+=M_PI/2.0*3.0;
       std::cout<<1<<std::endl;
   }

//   std::cout<<"moto_to_yaw!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<moto_to_yaw*(180.0/M_PI)<<std::endl;


      moto_move_yaw=(moto_to_yaw)-M_PI/*-moto_yaw*/;
      moto_move_yaw*=(180.0/M_PI);//已转成角度制//符号协商
#endif
}
void AngleSolver::Camera2Moto_rune(double moto_pitch, double moto_yaw , Eigen::Vector3d tvec,Eigen::Vector3d ctvec, double &moto_move_pitch, double &moto_move_yaw,double v,double g)
{
    //输出为相对角

    v*=100.0;
    g*=100.0;//输入用国际单位制，但是运算建议用量纲长度cm，时间用s即可
    moto_pitch/=(180.0/M_PI);//电控收发都是角度制，需要转成弧度制处理

    double z0=tvec(1,0);//水平距离
    double h=tvec(2,0)/*-PBMD*cos(temp_pitch)*/;
    double dist=z0/*+PBMD*sin(temp_pitch)*/;
    double z=h;//y
    double y=dist;//这里的y是水平距离，z是竖直距离，不要搞混,因为公式太长不想改了...
     double k;//zy枪管
     //std::cout<<"z"<<z<<" "<<"y"<<y<<std::endl;
     double vz1,vy1,fly_time;
     fly_time = sqrt(1.0 / (g * g) * (g * z + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) - v * v) * -2.0);

     if (std::isnan(fly_time)) {
         k = z / y;
     }
     else {
         vz1 = +(g * (y * y) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - g * (z * z) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - (g * g) * z * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) + (v * v) * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         vy1 = -((g * g) * y * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) - (v * v) * y * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0 + g * y * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         k = vz1 / vy1;
     }//未加空阻
//     k=z/y;
      double angle=atan(k);
      double moto_to_pitch=angle;
    moto_move_pitch=moto_to_pitch-moto_pitch/*pitch*//*moto_to_pitch-moto_pitch*//*-abs(tvec(1,0))/tvec(1,0)*M_PI/2.0*/;//电控收角度制，发给他们前需要转成角度制
    moto_move_pitch*=(180.0/M_PI);


//    //以下都是yaw的与pitch无关
    moto_yaw+=180.0;
    moto_yaw/=(180.0/M_PI);//电控收发都是角度制，需要转成弧度制处理
//    std::cout<<"moto_yaw"<<moto_yaw<<std::endl;
    double _x2=tvec(0,0);
    double _z2=/*abs(tvec(2,0))/tvec(2,0)*sqrt(pow(tvec(2,0),2)+pow(tvec(1,0),2))*/tvec(1,0);//?????

//    //std::cout<<_x2<<" "<<_z2<<" "<<atan(_x2/_z2)<<std::endl;
    double moto_to_yaw=atan(_x2/_z2);
      moto_move_yaw=(moto_to_yaw)/*-moto_yaw*/;
      moto_move_yaw*=-(180.0/M_PI);//已转成角度制
}

void AngleSolver::coordinary_transformation(double moto_pitch, double moto_yaw, Eigen::Vector3d &tvec, Eigen::Vector3d rvec,Eigen::Vector3d &moto_tvec)
{    

    Eigen::Vector3d C2IT;

    C2IT << 0.554498,
            13.101168,
           -7.744782;
    Eigen::Vector3d oc_tvec;
    oc_tvec<<tvec(0,0),tvec(2,0),-tvec(1,0);
    //std::cout<<"oc_tvec"<<oc_tvec<<std::endl;


#if (ANGLE==0)
    moto_tvec=oc_tvec + C2IT;
#else
    double a=moto_pitch/(180.0/M_PI);
    double b=moto_yaw/(180.0/M_PI)+M_PI;
    //std::cout<<b*(180.0/M_PI)<<std::endl;

    Eigen::Vector3d moto_tvec_temp;
    moto_tvec_temp=oc_tvec + C2IT;

    double xt1=moto_tvec_temp(0,0);
    double yt1=moto_tvec_temp(1,0)*cos(a);
    double x=xt1*cos(b)-yt1*sin(b);
    double y=yt1*cos(b)+xt1*sin(b);
    double l_t=sqrt(pow(moto_tvec_temp(0,0),2)+pow(moto_tvec_temp(1,0),2));
    double z=moto_tvec_temp(2,0)*cos(a)+l_t*sin(a);


    moto_tvec << x,y,z;
#endif
    //std::cout<<"moto_tvec"<<moto_tvec<<std::endl

}

