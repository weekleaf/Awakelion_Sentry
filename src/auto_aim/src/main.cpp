#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <ctime>
#include <mutex>
#include "ros/ros.h"

#include "auto_aim/Settings/Settings.h"
#include "auto_aim/AngleSolver/PnpSolver.h"
#include "auto_aim/Camera/MVVideoCapture.h"
#include "auto_aim/ArmorDetector/ArmorDetector.h"
#include "auto_aim/RuneDetector/inference_api.hpp"
#include "auto_aim/predict/predict.h"
#include "robot_driver/vision_rx_data.h"
#include "robot_driver/vision_tx_data.h"

ros::Publisher vision_pub;
ros::Subscriber vision_sub;
robot_driver::vision_tx_data pc_recv_mesg;
robot_driver::vision_rx_data pc_send_mesg;

void visionCallback(const robot_driver::vision_rx_data::ConstPtr &msg){
    pc_send_mesg.bullet_level = msg->bullet_level;
    pc_send_mesg.direction = msg->direction;
    pc_send_mesg.robot_color = msg->robot_color;
    pc_send_mesg.robot_pitch = msg->robot_pitch;
    pc_send_mesg.robot_yaw = msg->robot_yaw;
    pc_send_mesg.task_mode = msg->task_mode;
    pc_send_mesg.time_stamp = msg->time_stamp;
    pc_send_mesg.visual_valid = msg->visual_valid;
}

using namespace buff_detector;

struct AimData{
    cv::Mat src;
    ArmorObject armor_object;
    AngleSolver angle_solver=AngleSolver(PARAM_CALIBRATION_752);
    std::vector<ArmorObject> cars_map[8];
    double cars_radio[8]={0.0,0,0,0,0,0,53.7,0};
    double running_time=2.0;

    BuffObject buff_object;

    MainSettings main_settings=MainSettings(PARAM_OTHER_PATH,PARAM_CAMERA_PATH);
    bool flag_detector=false;
    bool flag_predictor=true;
};

AimData aim_data;

// PackData pack_data;
// UnpackData unpack_data;
// UnpackData *p_unpack_data = &unpack_data;
// Serial serial(aim_data.main_settings.port_param);

cv::VideoCapture vi(VIDEO_PATH);

void predictThread(AimData* p_aim_data);
void detectThread(AimData *p_aim_data){


#ifdef DEBUG_MODE
    (*p_aim_data).main_settings.setMainParam(WIN_OTHER);
    //main_settings.setCameraParam(WIN_CAMERA);
#endif
    ArmorDetector ad;
    std::vector<ArmorObject> objects;
    std::vector<BuffObject> bobjects;
    ArmorObject best_object;
    ad.initModel(MODEL_PATH);
    DetectorTool detector_tool;

    BuffDetector bd;
    // bd.initModel(MODEL_PATH_BUFF);

    static bool is_detect=false;


    while(ros::ok()){
        ros::spinOnce();
#if (USE_VIDEO==0)
        MVVideoCapture::GetFrame((*p_aim_data).src);
#else
        vi>>(*p_aim_data).src;
#endif
        if(((*p_aim_data).src).empty()){
            std::cout<<"!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }
        else{
#ifdef DEBUG_MODE
            cv::Mat dst;
            cv::resize((*p_aim_data).src,dst,cv::Size(SHOW_WIDTH,SHOW_HEIGHT));
            cv::imshow("save",dst);

#ifdef SAVE_IMG
            static int pic=0;
            std::stringstream num;
            num<<pic;
            cv::imwrite("/media/rm/upan/hero640480/"+num.str()+".jpg",dst);
            pic++;
#endif
    #if (USE_VIDEO==0)
        cv::waitKey(1);
    #else
        cv::waitKey(10);
    #endif


#endif
            cv::resize((*p_aim_data).src,(*p_aim_data).src,cv::Size(SHOW_WIDTH,SHOW_HEIGHT));
        }

        if(((*p_aim_data).src).empty()){
            std::cout<<"src is empty"<<endl;
            
            sleep(1);
            continue;
        }

        

       double time1=cv::getTickCount();
       bool color=1;
//#ifndef DEBUG_MODE
        //获取裁判系统敌方装甲板颜色
    //    if(unpack_data->getStm2PcMesg()->stm32_info_data.robot_color == 1){
    //        //(*p_aim_data).main_settings.enemy_color=red;
    //        color=1;
    //    }
    //    else if(unpack_data->getStm2PcMesg()->stm32_info_data.robot_color == 2){
    //        //(*p_aim_data).main_settings.enemy_color=blue;
    //        color=0;
    //    }
    //    // 接收电控发送的模式切换(比赛前切记打开)
    //    if(!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.task_mode))
    //        (*p_aim_data).main_settings.main_mode = unpack_data->getStm2PcMesg()->stm32_info_data.task_mode;
        if(pc_send_mesg.robot_color == 0)
            color = 0;
        else if(pc_send_mesg.robot_color == 1)
            color = 1;
        if(!std::isnan(pc_send_mesg.task_mode))
            (*p_aim_data).main_settings.main_mode = pc_send_mesg.task_mode;
//#endif

       if(/*(*p_aim_data).main_settings.main_mode == 0*/1){

         ad.detect(((*p_aim_data).src),objects);
         detector_tool=DetectorTool(objects,color);
         //best_object=&objects[0];

         for(int i=0;i<8;i++){
             (*p_aim_data).cars_map[i]=detector_tool.cars_map[i];
         }


         if(detector_tool.bestArmor(&best_object)){

             if(best_object.color==1||best_object.color==3){

                 (*p_aim_data).angle_solver.setTargetSize(23.1,5.7);//big
             }
             else{

                 (*p_aim_data).angle_solver.setTargetSize(15.3,5.7);//small
             }
             (*p_aim_data).armor_object=best_object;

             is_detect=true;

             double time2=cv::getTickCount();
             (*p_aim_data).running_time=(time2-time1)/cv::getTickFrequency();//s             
#ifdef DEBUG_MODE
       cv::Mat src_clone=(*p_aim_data).src.clone();
       std::vector<cv::Scalar> color_list;
       color_list.push_back(cv::Scalar(255,0,0));
       color_list.push_back(cv::Scalar(0,255,0));
       color_list.push_back(cv::Scalar(0,0,255));
       color_list.push_back(cv::Scalar(255,255,0));
       if(&best_object!=NULL)
       for(int i=0;i<4;i++){
           cv::line(src_clone,best_object.apex[i%4],best_object.apex[(i+1)%4],color_list[i],2);
           std::stringstream s;
           std::stringstream d;

           s << best_object.color;
           d << best_object.cls;
           cv::putText(src_clone, "color_size: " + s.str(), best_object.apex[2], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
           cv::putText(src_clone, "class_num: " + d.str(), best_object.apex[3], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
        }

       std::stringstream time;
       time<<1.0/(*p_aim_data).running_time;
       cv::putText(src_clone, "fps: " + time.str(), cv::Point(50,50), cv::FONT_HERSHEY_COMPLEX, 0.9, cv::Scalar(0, 255, 0));
       cv::resize(src_clone,src_clone,cv::Size(SHOW_WIDTH,SHOW_HEIGHT));
       cv::imshow("best_armor",src_clone);
       //cv::waitKey(1);
#endif
         }

       }
       else if((*p_aim_data).main_settings.main_mode == 1){
        //rune_detector
           bd.detect((*p_aim_data).src,bobjects);
           double time2=cv::getTickCount();
           (*p_aim_data).running_time=(time2-time1)/cv::getTickFrequency();//s
           for(auto fan:bobjects){
               if(fan.cls==0){
                   (*p_aim_data).buff_object=fan;
                   (*p_aim_data).angle_solver.setTargetSize(26.0);
                   is_detect=true;
#ifdef DEBUG_MODE
       cv::Mat src_clone=(*p_aim_data).src.clone();
       std::vector<cv::Scalar> color_list;
       color_list.push_back(cv::Scalar(255,0,0));
       color_list.push_back(cv::Scalar(0,255,0));
       color_list.push_back(cv::Scalar(0,0,255));
       color_list.push_back(cv::Scalar(255,255,0));
       color_list.push_back(cv::Scalar(255,0,255));
       if(&(*p_aim_data).buff_object!=NULL)
       for(int i=0;i<5;i++){
           cv::line(src_clone,(*p_aim_data).buff_object.apex[i%5],(*p_aim_data).buff_object.apex[(i+1)%5],color_list[i],2);
           std::stringstream s;
           std::stringstream d;

           s << (*p_aim_data).buff_object.color;
           d << (*p_aim_data).buff_object.cls;
           cv::putText(src_clone, "color_size: " + s.str(), best_object.apex[2], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
           cv::putText(src_clone, "class_num: " + d.str(), (*p_aim_data).buff_object.apex[3], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
        }
       cv::imshow("rune",src_clone);
       //cv::waitKey(1);
#endif
                   break;
               }
           }
       }

       if(/*is_detect==true*/1){
           is_detect=false;
           predictThread(p_aim_data);
       }
     }
}

void predictThread(AimData* p_aim_data){
    Eigen::Vector3d moto_tvec;
    double moto_move_pitch;
    double moto_move_yaw;
    static double last_pitch=0;
    static double last_yaw=0;
    static int lost_flag=0;
    static double during_time=2.0;
    double predict_time=during_time+(*p_aim_data).running_time;//s
    static ArmorPredictTool armor_predict_tool;
    static RunePredictTool rune_predict_tool;
    armor_predict_tool.kalmanInit();
    rune_predict_tool.initKalman();

        double time1=cv::getTickCount();
        if(/*(*p_aim_data).main_settings.main_mode == 0*/1){

             double moto_pitch_angle=pc_send_mesg.robot_pitch;
             double moto_yaw_angle=pc_send_mesg.robot_yaw;
//             double moto_pitch_angle=0.0;
//             double moto_yaw_angle=0.0;
             last_pitch=(moto_pitch_angle);//此外部仅为电控pitch双标
             last_yaw=moto_yaw_angle;
             //std::cout<<"电控yaw角："<<moto_yaw_angle<<std::endl;
             //std::cout<<"电控pitch角："<<moto_pitch_angle<<std::endl;
             double bullet_speed=19.0;

            armor_predict_tool.inputData((*p_aim_data).angle_solver,moto_pitch_angle,moto_yaw_angle,last_pitch,last_yaw,(*p_aim_data).cars_radio,&((*p_aim_data).armor_object),(*p_aim_data).cars_map,bullet_speed,predict_time);

            if(armor_predict_tool.predictArmor()){

                lost_flag=0;
                (*p_aim_data).angle_solver.Camera2Moto(moto_pitch_angle,moto_yaw_angle,armor_predict_tool.tvec_armor,armor_predict_tool.tvec11,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
//                moto_move_yaw+=0.0;
//                moto_move_pitch+=4.0;
#if (ANGLE==0)
                last_pitch=(moto_move_pitch+moto_pitch_angle);//此外部仅为电控pitch双标
                last_yaw=moto_move_yaw+moto_yaw_angle;
#else
                last_pitch=(moto_move_pitch);//此外部仅为电控pitch双标
                last_yaw=moto_move_yaw;
#endif
                if(abs(last_yaw)>=180.0) last_yaw+=-abs(last_yaw)/last_yaw*360.0;
                pc_recv_mesg.aim_pitch=last_pitch+0.0;
                pc_recv_mesg.aim_yaw=last_yaw;
                pc_recv_mesg.visual_valid=1;
                pc_recv_mesg.shoot_valid = 1;
                std::cout<<"pitch到达角"<<last_pitch<<std::endl;
                std::cout<<"pitch转动角"<<last_pitch-moto_pitch_angle<<std::endl;
                std::cout<<"yaw到达角"<<last_yaw<<std::endl;
                std::cout<<"yaw转动角"<<last_yaw-moto_yaw_angle<<std::endl;
            }
            else if(lost_flag<=4){
                pc_recv_mesg.aim_pitch=last_pitch;
                pc_recv_mesg.aim_yaw=last_yaw;
                pc_recv_mesg.visual_valid=1;
                pc_recv_mesg.shoot_valid = 0;
                lost_flag++;
            }
            else{
                pc_recv_mesg.aim_pitch=last_pitch;
                pc_recv_mesg.aim_yaw=last_yaw;
                pc_recv_mesg.visual_valid=0;
                pc_recv_mesg.shoot_valid = 0;
                std::cout<<"!!!!!!!!!!!!!!"<<std::endl;
            }
            std::cout<<"wcnm"<<armor_predict_tool.predictArmor()<<std::endl;
            vision_pub.publish(pc_recv_mesg);
        }

        else if((*p_aim_data).main_settings.main_mode==1){
            double moto_pitch_angle=pc_send_mesg.robot_pitch;
             double moto_yaw_angle=pc_send_mesg.robot_yaw;
            // double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
            // double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;

//            double moto_pitch_angle=0;
//            double moto_yaw_angle=0;
            double bullet_speed=30;
            rune_predict_tool.inputDataRune((*p_aim_data).angle_solver,(*p_aim_data).buff_object,moto_pitch_angle,moto_yaw_angle,bullet_speed,predict_time);

            last_pitch=moto_pitch_angle;
            last_yaw=moto_yaw_angle;
            if(rune_predict_tool.setRuneCoordinary()){
                lost_flag=0;
                Eigen:: Vector3d tvec_temp;
                (*p_aim_data).angle_solver.Camera2Moto(moto_pitch_angle,moto_yaw_angle,rune_predict_tool.cur_moto_tvec,tvec_temp,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
                #if(ANGLE==0)
                if(abs(moto_move_yaw)<10&&abs(moto_move_pitch)<20){
                    last_pitch=moto_move_pitch+moto_pitch_angle;
                    last_yaw=moto_move_yaw+moto_yaw_angle;
                }
                #else
                last_pitch=moto_move_pitch;
                last_yaw=moto_move_yaw;
                #endif


                if(abs(last_yaw)>180.0) last_yaw+=-abs(last_yaw)/last_yaw*360.0;

                std::cout<<"move_yaw"<<moto_move_yaw<<std::endl;
                std::cout<<"move_pitch"<<moto_move_pitch<<std::endl;


                pc_recv_mesg.aim_pitch=last_pitch+0.0;
                pc_recv_mesg.aim_yaw=last_yaw;
                pc_recv_mesg.visual_valid=1;
                pc_recv_mesg.shoot_valid = 1;
            }
            else if(lost_flag<=4){
                pc_recv_mesg.aim_pitch=last_pitch;
                pc_recv_mesg.aim_yaw=last_yaw;
                pc_recv_mesg.visual_valid=1;
                pc_recv_mesg.shoot_valid = 0;
                lost_flag++;
            }
            else{
                pc_recv_mesg.aim_pitch=last_pitch;
                pc_recv_mesg.aim_yaw=last_yaw;
                pc_recv_mesg.visual_valid=0;
                pc_recv_mesg.shoot_valid = 0;
            }
        }
        double time2=cv::getTickCount();
        during_time=(time2-time1)/cv::getTickFrequency();//s
//        std::cout<<"predict_fps"<<1/during_time<<std::endl;
//        std::cout<<"totle_fps"<<1/predict_time<<std::endl;
}







int main(int argc, char *argv[]){
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"auto_aim");
    ros::NodeHandle nh;
    
    vision_pub = nh.advertise<robot_driver::vision_tx_data>("vision_tx_data",1);
    vision_sub = nh.subscribe("/vision_rx_data", 1, visionCallback);
    // serial.openPort("/dev/ttyACM0");
#if (USE_VIDEO == 0)
    std::cout << "MVVideoCapture Init" << std::endl;
    if(-1 == MVVideoCapture::Init())
    {
        std::cout << "MVVideoCapture ERROR!!!" << std::endl;
        return 1;
    }
    else{
        MVVideoCapture::Play();
        MVVideoCapture::SetExposureTime(false, (/*main_settings.debug.expore_time*/12000));//bu yao zi dong tiao bao guang!!!
        MVVideoCapture::SetLargeResolution(true);
        std::cout << "MVVideoCapture Finished!" << std::endl;
    }
#endif


    // std::thread dt(detectThread,&aim_data,&pack_data,&unpack_data,&serial);
    // std::thread st(&UnpackData::processing, p_unpack_data, &serial);
    // dt.join();
    // st.join();
    detectThread(&aim_data);
#ifdef DEBUG_MODE
//      while(1){
//        cv::Mat src_clone=src.clone();
//        //if(main_settings.debug.src==1) cv::imshow("src",src);
//        if(main_settings.debug.detect_armor==1){
//          //添加画图处理
//        }
//        if(main_settings.debug.armor_chosen){
//          //添加画图处理
//        }
//        if(main_settings.debug.predict_point){
//          //
//        }
//        if(main_settings.debug.inaccuracy_point){

//        }
//        if(main_settings.debug.dc_pitch){

//        }
//        if(main_settings.debug.dc_yaw){

//        }
//      }
#endif
    //serial.closeDevice();
    return 0;
}
