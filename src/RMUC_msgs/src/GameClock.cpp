#include <ros/ros.h>
#include <chrono>
#include <fstream>
#include "RMUC_msgs/common.h"
#include "RMUC_msgs/clock.h"

using namespace std;
using namespace ros;

/*
* @brief write and store the log into file easily by <sstream>
*/
struct File
{
    stringstream ss;
    const string absFilepath="/home/siyiya/RMUC_Decision/src/RMUC_msgs/save/attack_duration.txt";
    ofstream File_write;
};

RMUC_msgs::common gameclock_common_msg;
RMUC_msgs::clock gameclock_send_msg;

void gameclock_common_msg_cb(const boost::shared_ptr<RMUC_msgs::common>& msg)
{
    gameclock_common_msg=*msg;
}

int main(int argc,char** argv)
{
    init(argc,argv,"clock_node");
    setlocale(LC_ALL,"");
    Subscriber clock_sub;
    Publisher clock_pub;
    NodeHandle nh;
    File F;
    bool is_file_save=false;  //确保击打前哨站的持续时间的记录只有一次

    Rate r(10);  //gameclock_common_msg发送频率为10Hz，与行为树运行频率一致

    while(ok())
    {
        clock_sub=nh.subscribe("/common_msg",3,&gameclock_common_msg_cb);
        clock_pub=nh.advertise<RMUC_msgs::clock>("clock_msg",6);
        if(gameclock_common_msg.gamestart==1)
        {
            static auto start_time=chrono::high_resolution_clock::now();
            auto current_time=chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now()-start_time);
            uint16_t curt_time=current_time.count();
            cout<<"当前时间为："<<curt_time<<"秒"<<endl;
            uint16_t remaining_time=static_cast<uint16_t>(420)-current_time.count();
            cout<<"剩余时间为："<<remaining_time<<"秒"<<endl;
            gameclock_send_msg.current_time_from_clock=curt_time;
            gameclock_send_msg.remaining_time_from_clock=remaining_time;
            clock_pub.publish(gameclock_send_msg);
            if(gameclock_common_msg.enemy_outpost_HP==0)
            {
                ROS_INFO("get the attack duration!");
                static uint64_t attack_duration=current_time.count();
                cout<<"攻打前哨站的持续时间为:"<<" "<<attack_duration<<"s"<<endl;
                if(!is_file_save)
                {
                    auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());  //to_time_t()的运用：https://blog.csdn.net/hou8389846/article/details/77962343
                    auto real_time=put_time(localtime(&t), "%Y-%m-%d %X");  //put_time()的运用：https://www.yiibai.com/cpp_standard_library/cpp_put_time.html
                    F.ss<<attack_duration;  //"<<"表示attack_duration向ss这个对象输入
                    F.File_write.open(F.absFilepath,ios::app);  //std::ios::app 写入文件尾部
                    if(!F.File_write.is_open())
                    {
                        ROS_ERROR("can't not open the file! check the absFilepath.");
                    }
                    else
                    {
                        F.File_write<<"\n"<<"(#TEST#)["<<real_time<<"]攻打前哨站用时："<<F.ss.rdbuf()<<"s";  //将stringstream类，流的缓冲中的数据，写入File_write里头
                    }
                    F.File_write.close();
                    is_file_save=true;
                }
            }
        }
        r.sleep();
        spinOnce();
    }
    return 0;
}
