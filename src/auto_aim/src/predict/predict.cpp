#include "auto_aim/predict/predict.h"
#include "auto_aim/Settings/Settings.h"

ArmorPredictTool::ArmorPredictTool(){

}


void ArmorPredictTool::inputData(AngleSolver angle_solver, double moto_pitch, double moto_yaw, double last_pitch, double last_yaw, double *cars_radio, ArmorObject* object_addr,
                                 std::vector<ArmorObject> *cars_map, double bullet_speed, double running_time){
    this->cars_radio=cars_radio;
    this->car_radio=*(cars_radio+(*object_addr).cls);
    this->cars_map=cars_map;
    this->angle_solver=angle_solver;
    this->object_addr=object_addr;
    this->moto_pitch=moto_pitch;
    this->moto_yaw=moto_yaw;
    this->last_pitch=last_pitch-moto_pitch;
    this->last_yaw=last_yaw-moto_yaw;
    this->running_time=running_time;
    this->bullet_speed=bullet_speed;
//    std::cout<<"car_radio:"<<car_radio<<std::endl;
    //    std::cout<<"(*object_addr).color:"<<(*object_addr).color<<std::endl;
}

cv::Point2f ArmorPredictTool::world2image(Vector3d world_point)
{
    Eigen::Vector3d I2CT;

    I2CT << -0.554498,
            +7.744782,
           -13.101168;

    Eigen::Vector3d temp_camera_point;
    Eigen::Vector3d camera_point;
    #if (ANGLE==0)
    temp_camera_point<<world_point(0,0),-world_point(2,0),world_point(1,0);
    camera_point=temp_camera_point+I2CT;
    #else
    double x_t=world_point(0,0);
    double y_t=world_point(1,0);
    double z_t=world_point(2,0);
    double x=x_t*cos(-moto_yaw/(180.0/M_PI)) - y_t*sin(-moto_yaw/(180.0/M_PI));
    double y=y_t*cos(-moto_yaw/(180.0/M_PI)) + x_t*sin(-moto_yaw/(180.0/M_PI));
    double xx=x_t+I2CT(0,0);
    double yy=y_t+I2CT(1,0);
    double xy=sqrt(xx*xx + yy*yy)/cos(moto_pitch/(180.0/M_PI));
    double z=z_t*cos(-moto_pitch/(180.0/M_PI))+xy*sin(-moto_pitch/(180.0/M_PI))+I2CT(2,0);
    camera_point<<x,z,y;
    #endif
    cv::Point2f point=cv::Point2f(camera_point(0,0)/camera_point(2,0)*1081.23719869997+317.449119082735,camera_point(1,0)/camera_point(2,0)*1080.68541841437+234.296263922105);

    return point;

}

ArmorPredictTool::ArmorPredictTool(AngleSolver angle_solver,double moto_pitch,double moto_yaw,double last_pitch,double last_yaw, double *cars_radio,ArmorObject* object_addr,
                                   std::vector<ArmorObject> *cars_map,double bullet_speed,double running_time)
{
    this->cars_radio=cars_radio;
    this->car_radio=*(cars_radio+(*object_addr).cls);
    this->cars_map=cars_map;
    this->angle_solver=angle_solver;
    this->object_addr=object_addr;
    this->moto_pitch=moto_pitch;
    this->moto_yaw=moto_yaw;
    this->last_pitch=last_pitch-moto_pitch;
    this->last_yaw=last_yaw-moto_yaw;
    this->running_time=running_time;
    this->bullet_speed=bullet_speed;
//#ifdef DEBUG_MODE
//    std::cout<<"car_radio:"<<car_radio<<std::endl;
//    std::cout<<"(*object_addr).color:"<<(*object_addr).color<<std::endl;
//#endif
}

bool ArmorPredictTool::solveCarRadio()
{
    //长度单位为cm
    int car_cls=(*object_addr).cls;
    //std::cout<<"car_cls's num:"<<cars_map[car_cls].size()<<std::endl;
    if(cars_map[car_cls].size()==2){
        Eigen::Vector3d moto_t_11; Eigen::Vector3d moto_t_22;
        Eigen::Vector3d rvec11;
        Eigen::Vector3d rvec22;
        if((*object_addr).area==cars_map[car_cls][0].area /*&& ((*object_addr).apex[0].y==cars_map[car_cls][0].y)*/){
            obj1=cars_map[car_cls][0];
            obj2=cars_map[car_cls][1];
        }
        else{
            obj1=cars_map[car_cls][1];
            obj2=cars_map[car_cls][0];
        }
        cv::Point2f armor_points[4]={obj1.apex[0],obj1.apex[1],obj1.apex[2],obj1.apex[3]};
        cv::Point2f armor_points2[4]={obj2.apex[0],obj2.apex[1],obj2.apex[2],obj2.apex[3]};

        angle_solver.getAngle(armor_points,tvec11,rvec11);
        angle_solver.getAngle(armor_points2,tvec22,rvec22);

        angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec11,rvec11,moto_t_11);
        angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec22,rvec22,moto_t_22);
        this->tvec1=moto_t_11;
        this->tvec2=moto_t_22;
        this->running_time+=sqrt(pow(this->tvec1(1,0),2)+pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
        this->car_tvec=tvec1;
        return 1;
    }
    else if(cars_map[car_cls].size()==1){
        Eigen::Vector3d moto_t_11;
        Eigen::Vector3d rvec11;
        obj1=cars_map[car_cls][0];
        cv::Point2f armor_points[4]={obj1.apex[0],obj1.apex[1],obj1.apex[2],obj1.apex[3]};
        angle_solver.getAngle(armor_points,tvec11,rvec11);
        angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec11,rvec11,moto_t_11);
        this->tvec1=moto_t_11;
        this->running_time+=sqrt(pow(this->tvec1(1,0),2)+pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
        this->car_tvec=tvec1;
        return 1;
    }
    else{
        std::cout<<"no armor!"<<std::endl;
        return 0;
    }
}

bool ArmorPredictTool::predictRotated()
{

    static std::vector<double> angles;
        angles.push_back(this->car_angle);

    //std::cout<<car_angle*180.0/M_PI<<std::endl;

       if(angles.size()>=4){
           angles.erase(angles.begin());
            
           Eigen::Vector2d _angle;
           Eigen::Vector2d _anglep;

           if(running_time<=0) running_time=1.0;
           double v1=(angles[2]-angles[1])/running_time;
           double v2=(angles[1]-angles[0])/running_time;
           double v=(v1+v2)/2.0;
           double a=(v1-v2)/running_time/2;

           //std::cout<<v<<" "<<a<<std::endl;
           _angle << v,a;
           _anglep=angle_kalman.update(_angle,running_time);
           //std::cout<<"_anglep"<<_anglep<<std::endl;
          this->angle_predict=angles[1] + _anglep(0,0) * running_time + _anglep(1,0) * running_time*running_time/2.0;

           if(angle_predict<M_PI/4.0){
               if(_anglep(1,0)<-1.57&&angle_predict>0.0){
                   angle_predict+=M_PI/2.0;
                   this->switch_y=1;
               }
               else if(_anglep(1,0)<-1.57&&angle_predict<0.0){
                   angle_predict+=M_PI;
                   this->switch_y=0;
               }
               else if(angle_predict<M_PI/4.3){
                   angle_predict+=M_PI/2.0;
                   this->switch_y=1;
               }
           }
           else if(angle_predict>M_PI*3.0/4.0){
               if(_anglep(1,0)>1.57&&angle_predict<M_PI){
                   angle_predict-=M_PI/2;
                   this->switch_y=1;
               }
               else if(_anglep(1,0)>1.57&&angle_predict>M_PI){
                   angle_predict-=M_PI;
                   this->switch_y=0;
               }
               else if(angle_predict>M_PI*3.6/4.0){
                   angle_predict+=M_PI/2;
                   this->switch_y=1;
               }
           }
            //std::cout<<"angle_predict"<<angle_predict*180.0/M_PI<<std::endl;
            return true;
    }
       else return false;
}

bool ArmorPredictTool::predictMove()
{

    static std::vector<double> x_store;
    static std::vector<double> y_store;
    static std::vector<double> z_store;
    //std::cout<<"car_tvec"<<car_tvec<<std::endl;
       x_store.push_back(this->car_tvec(0,0));
       y_store.push_back(this->car_tvec(1,0));
       z_store.push_back(this->car_tvec(2,0));

       if(x_store.size()>=4){
        x_store.erase(x_store.begin());
        y_store.erase(y_store.begin());
        z_store.erase(z_store.begin());
//        double x_delete=cos(moto_pitch)*sin(/*moto_yaw+*/last_yaw)*sqrt(pow(x_store[2],2)+pow(y_store[2],2)+pow(z_store[2],2))-x_store[2];
//        double y_delete=cos(moto_pitch)*cos(/*moto_yaw+*/last_yaw)*sqrt(pow(x_store[2],2)+pow(y_store[2],2)+pow(z_store[2],2))-y_store[2];
//        double z_delete=sin(last_pitch)*sqrt(pow(y_store[2],2)+pow(z_store[2],2)+pow(x_store[2],2))-z_store[2];

          Eigen::Vector2d _X;
          Eigen::Vector2d _Y;
          Eigen::Vector2d _Z;
          Eigen::Vector2d _Xp;
          Eigen::Vector2d _Yp;
          Eigen::Vector2d _Zp;

          if(this->running_time<=0) this->running_time=0.5;

          double vx1=(x_store[2]-x_store[1])/running_time;
          double vy1=(y_store[2]-y_store[1])/running_time;
          double vz1=(z_store[2]-z_store[1])/running_time;
          double vx2=(x_store[1]-x_store[0])/running_time;
          double vy2=(y_store[1]-y_store[0])/running_time;
          double vz2=(z_store[1]-z_store[0])/running_time;
          double ax=(vx1-vx2)/running_time/2;
          double ay=(vy1-vy2)/running_time/2;
          double az=(vz1-vz2)/running_time/2;
          _X << vx1,ax;
          _Y << vy1,ay;
          _Z << vz1,az;

          _Xp=car_kalman_x.update(_X,this->running_time);
          _Yp=car_kalman_y.update(_Y,this->running_time);
          _Zp=car_kalman_z.update(_Z,this->running_time);

          double x;
          double y;
          double z;
          if(obj2.area>obj1.area){
              x=tvec2(0,0) + _Xp(0,0) * running_time + 0.5 * _Xp(1,0) * running_time * running_time;
              y=tvec2(1,0) + _Yp(0,0) * running_time + 0.5 * _Yp(1,0) * running_time * running_time;
              z=tvec2(2,0) + _Zp(0,0) * running_time + 0.5 * _Zp(1,0) * running_time * running_time;
              x_store.clear();
              y_store.clear();
              z_store.clear();
          }
          else{
              x=tvec1(0,0) + _Xp(0,0) * running_time + 0.5 * _Xp(1,0) * running_time * running_time;
              y=tvec1(1,0) + _Yp(0,0) * running_time + 0.5 * _Yp(1,0) * running_time * running_time;
              z=tvec1(2,0) + _Zp(0,0) * running_time + 0.5 * _Zp(1,0) * running_time * running_time;
          }
           
          car_predict << x,y,z;
           return true;
       }
       else return false;
}

bool ArmorPredictTool::stateAdd()
{

    double armor_x=car_predict(0,0);
    double armor_y=car_predict(1,0);
    double armor_z=car_predict(2,0);
    tvec_armor<<armor_x,armor_y,armor_z;
#ifdef DEBUG_MODE
    static bool last_point_is_ex=false;
    static cv::Point2f last_point;
    cv::Mat armor_draw=cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
    if(last_point_is_ex==true){
//        std::cout<<"像素距离误差:"<<POINT_DIST(last_point,world2image(tvec1))<<std::endl;
//        std::cout<<"准星横向误差:"<<last_point.x-320.0<<std::endl;
    }
    cv::circle(armor_draw,world2image(tvec_armor),5,cv::Scalar(0,255,0),5);
    cv::circle(armor_draw,world2image(tvec1),5,cv::Scalar(255,0,0),5);
    cv::circle(armor_draw,cv::Point2f(640/2,480/2),5,cv::Scalar(255,255,255),5);
    cv::imshow("armor_draw",armor_draw);
    last_point=world2image(tvec_armor);
    last_point_is_ex=true;
#endif

}

void ArmorPredictTool::kalmanInit(){
    static bool init_flag=false;
    if(!init_flag){
        Eigen::Matrix2d A=Eigen::Matrix2d::Identity();
        Eigen::Matrix2d H;
        for(int i=0;i<2;++i)
        {
            H(i,i)=1;
        }
        Eigen::Matrix2d R;
        R<<100,0,0,100;
        Eigen::Matrix2d Q;
        Q<<0.1,0.1,0.1,0.1;
        Eigen::Vector2d init{0.0,0.0};
        angle_kalman = Armor_Kalman(A,H,R,Q,init,0);
        car_kalman_x = Armor_Kalman(A,H,R,Q,init,0);
        car_kalman_y = Armor_Kalman(A,H,R,Q,init,0);
        car_kalman_z = Armor_Kalman(A,H,R,Q,init,0);
        init_flag=true;
        std::cout<<"init autoaim kf success!"<<std::endl;
    }
}

bool ArmorPredictTool::predictArmor()
{



    if(solveCarRadio()){
           if(predictMove()){
               stateAdd();
               return true;
           }
           else return false;
    }
    else return false;

}

RunePredictTool::RunePredictTool(){
}

//RunePredictTool::RunePredictTool(AngleSolver angle_solver,BuffObject object,double moto_pitch,double moto_yaw,double bullet_speed,double running_time){
    
//    this->angle_solver=angle_solver;
//    this->rune_center=object.apex[0];
//    this->moto_pitch=moto_pitch;
//    this->moto_yaw=moto_yaw;
//    for(int i=1;i<5;i++){
//        this->cur_pos_points.push_back(object.apex[i]);
//    }
//    this->cur_pos_center=(object.apex[1]+object.apex[3])/2;
//}

void RunePredictTool::inputDataRune(AngleSolver angle_solver,BuffObject object,double moto_pitch,double moto_yaw,double bullet_speed,double running_time)
{
    this->angle_solver=angle_solver;
    this->rune_center=object.apex[0];
    this->moto_pitch=moto_pitch;
    this->moto_yaw=moto_yaw;
    this->bullet_speed=bullet_speed;
    this->running_time=running_time;
    this->cur_pos_points.clear();
    for(int i=1;i<5;i++){
        this->cur_pos_points.push_back(object.apex[i]);
    }
    this->cur_pos_center=(object.apex[1]+object.apex[3])/2;
}

void RunePredictTool::initKalman()
{
    static bool init_flag_rune=false;
    if(!init_flag_rune){
        Eigen::Matrix2d A=Eigen::Matrix2d::Identity();
        Eigen::Matrix2d H;
        for(int i=0;i<2;++i)
        {
            H(i,i)=1;
        }
        Eigen::Matrix2d R;
        R<<100,0,0,100;
        Eigen::Matrix2d Q;
        Q<<0.1,0.1,0.1,0.1;
        Eigen::Vector2d init{0.0,0.0};
        kalman = Armor_Kalman(A,H,R,Q,init,0);
        init_flag_rune=true;
        std::cout<<"init rune kf success!"<<std::endl;
    }
}
bool RunePredictTool::setRuneCoordinary(){
    Eigen::Vector3d source_tvec;
    Eigen::Vector3d rvec_temp;
    Eigen::Vector3d moto_source_tvec;
    //std::cout<<"cur_pos_points.size()"<<cur_pos_points.size()<<std::endl;
#ifdef DEBUG_MODE
    std::vector<cv::Point2f> sourse_cur_pos_points;
    for(int i=0;i<4;i++){
        sourse_cur_pos_points.push_back(cur_pos_points[i]);
    }
#endif
    angle_solver.getAngle(cur_pos_points,source_tvec);
    angle_solver.coordinary_transformation(moto_pitch,moto_yaw,source_tvec,rvec_temp,moto_source_tvec);
    double cur_dist=sqrt(pow(moto_source_tvec(1,0),2)+pow(moto_source_tvec(0,0),2)+pow(moto_source_tvec(2,0),2));//cm



                cv::Point2d rc_pos_in_rune=cv::Point2d(cur_pos_center.x-rune_center.x,rune_center.y-cur_pos_center.y);//x轴已翻转
                for(int i=0;i<4;i++){
                    cur_pos_points[i].x=rune_center.x-cur_pos_points[i].x;
                    cur_pos_points[i].y-=rune_center.y;
                }//将整个靶子转到旋转中心坐标系中

                double k=rc_pos_in_rune.y/rc_pos_in_rune.x;
                if(std::isnan(k)) return 0;

                double angle=atan(k);
                if(rc_pos_in_rune.x<0&&k<0) angle+=M_PI;
                else if(rc_pos_in_rune.x<0&&k>0) angle+=M_PI;
                else if(rc_pos_in_rune.x>0&&k<0) angle+=M_PI*2.0;

                angle*=180.0/M_PI;// 转成角度制处理


                // Eigen::Vector3d source_point;
                // source_point<< 0.7*cos(angle),0.7*sin(angle),0;

                angles.push_back(angle);

                if(angles.size()>=4){
                    angles.erase(angles.begin());
                    double predict_t;

                    if(bullet_speed > 0.0&&bullet_speed < 35.0)
                    {
                        predict_t=cur_dist/100.0/bullet_speed;
                    }
                    else
                    {
                        predict_t=cur_dist/100.0/26.0;
#ifdef DEBUG_MODE
                        std::cout << "no bulet speed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  " << std::endl;
#endif // DEBUG_MODE
                    }

                    //this->running_time+=predict_t+3.5;
                    this->running_time=0.2+predict_t;


                    double angle_diff=angles[2]-angles[1];
                    double angle_diff2=angles[1]-angles[0];

                    static int fx=0;
                    if(angle_diff>0&&angle_diff2>0&&fx==0) fx=1;
                    if(angle_diff<0&&angle_diff2<0&&fx==0) fx=-1;

                    if(fx==0) return 0;


                    if(abs(angle_diff)>=355){
                        angle_diff=angle_diff-360.0*(angle_diff/abs(angle_diff));
                        if(fx==1&&angle_diff<0){
                            while(angle_diff<0){
                                angle_diff+=72.0;
                            }
                        }
                        else if(fx==-1&&angle_diff>0){
                            while(angle_diff>0){
                                angle_diff-=72.0;
                            }
                        }
                    }
                    else if(fx==1&&angle_diff<0){
                        while(angle_diff<0){
                            angle_diff+=72.0;
                        }
                    }
                    else if(fx==1&&angle_diff>0){
                        angle_diff=(int)floor(angle_diff)%72+angle_diff-floor(angle_diff);
                    }
                    else if(fx==-1&&angle_diff>0){
                        while(angle_diff>0){
                            angle_diff-=72.0;
                        }
                    }
                    else if(fx==-1&&angle_diff<0){
                        angle_diff=(int)floor(angle_diff)%72+angle_diff-floor(angle_diff);
                    }



                    if(abs(angle_diff2)>=355){
                        angle_diff2=angle_diff2-360.0*(angle_diff2/abs(angle_diff2));
                        if(fx==1&&angle_diff2<0){
                            while(angle_diff2<0){
                                angle_diff2+=72.0;
                            }
                        }
                        else if(fx==-1&&angle_diff2>0){
                            while(angle_diff2>0){
                                angle_diff2-=72.0;
                            }
                        }
                    }
                    else if(fx==1&&angle_diff2<0){
                        while(angle_diff2<0){
                            angle_diff2+=72.0;
                        }
                    }
                    else if(fx==1&&angle_diff2>0){
                        angle_diff2=(int)floor(angle_diff2)%72+angle_diff2-floor(angle_diff2);
                    }
                    else if(fx==-1&&angle_diff2>0){
                        while(angle_diff2>0){
                            angle_diff2-=72.0;
                        }
                    }
                    else if(fx==-1&&angle_diff2<0){
                        angle_diff2=(int)floor(angle_diff2)%72+angle_diff2-floor(angle_diff2);
                    }

                        //对angle_diff做跃变与象限跃变处理
#ifdef DEBUG_MODE
//                    std::cout<<"angle"<<angle<<std::endl;
//                        std::cout<<"angle_diff--------------------------------------------------------------------------------------------"<<angle_diff<<std::endl;
//                        std::cout<<"angle_diff2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<angle_diff2<<std::endl;
#endif


                    Eigen::Vector2d z_k;
                    double v1,v2,a;
                    if(running_time!=0){
                        v1=angle_diff/running_time;
                        v2=angle_diff2/running_time;
                        a=(v1)/running_time;
                    }
//                    else{
//                        v1=angle_diff/1.5;
//                        v2=angle_diff2/1.5;
//                        a=(v1)/running_time;
//                    }

                    z_k<< angle_diff,v1;

#ifdef DEBUG_MODE
//                    std::cout<<"z_k:"<<z_k<<endl;
#endif

                    Eigen::Vector2d state=kalman.update(z_k,running_time);        //更新卡尔曼滤波
                                                      //将角度和时间传入卡尔曼滤波后得到的角速度
                    //kalman_speed=speed_recent;
                    //double kalman_angle_recent=state(0,0);                 //待激活装甲板与中心点角度的滤波值

                    pre_angle = state(0,0) /*state(0,0)*/ + running_time * state(1,0) /** running_time * running_time * 0.5*/;
                    if(abs(pre_angle)>90.0)
                       pre_angle=0.0;
//                    if(pre_angle<0) pre_angle+=360.0;
//                    else if(pre_angle>=360) pre_angle-=360.0;
                    pre_angle/=(180.0/M_PI);

                    for(int i=0;i<4;i++){
                        cv::Point2f temp_point;
                        temp_point.x=-(cur_pos_points[i].x*cos(pre_angle)-cur_pos_points[i].y*sin(pre_angle))+rune_center.x;
                        temp_point.y=(cur_pos_points[i].y*cos(pre_angle)+cur_pos_points[i].x*sin(pre_angle))+rune_center.y;
                        pre_cur_pos_points.push_back(temp_point);
                    }
                    cv::Point2f pre_cur_center=(pre_cur_pos_points[0]+pre_cur_pos_points[2])/2;
                    double angle_rad=(angle+state(0,0))/((180.0)/M_PI);
                    double h=70.0*sin(angle_rad)+153.5-30.0;
                    double dist=660;
                    double x=(pre_cur_center.x-SHOW_WIDTH/2.0)*100.0/POINT_DIST(pre_cur_center,rune_center);


#ifdef DEBUG_MODE
                    cv::Mat rune_draw=cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
                    for(int i=0;i<4;i++){
                        cv::circle(rune_draw,sourse_cur_pos_points[i],1,cv::Scalar(0,255,0),1);
                        cv::circle(rune_draw,pre_cur_pos_points[i],1,cv::Scalar(255,0,0),1);
                    }
                    cv::circle(rune_draw,pre_cur_center,1,cv::Scalar(0,0,255),1);
                    cv::circle(rune_draw,cv::Point2f(SHOW_WIDTH/2.0,SHOW_HEIGHT/2.0),1,cv::Scalar(255,255,255),1);
                    cv::imshow("rune_draw",rune_draw);
#endif

                    Eigen::Vector3d tvec;Eigen::Vector3d moto_tvec;
                    angle_solver.getAngle(pre_cur_pos_points,tvec);
                    tvec(0,0)=x;
                    tvec(2,0)=dist;
                    tvec(1,0)=-h;
                    pre_cur_pos_points.clear();
                    angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec,rvec_temp,moto_tvec);
                    this->cur_moto_tvec=moto_tvec;
                    return 1;
                }
                else{
                    return 0;
                }
}


