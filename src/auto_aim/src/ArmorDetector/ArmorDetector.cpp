#include "auto_aim/ArmorDetector/ArmorDetector.h"
#include "auto_aim/ArmorDetector/inference_api2.hpp"

DetectorTool::DetectorTool()
{
}

DetectorTool::DetectorTool(std::vector<ArmorObject> objects, bool &color_num)
{
    this->Blue_or_Red=color_num;
    this->objects=objects;
    for(auto obj:objects){
            if (this->Blue_or_Red == 0)
            {
                if (obj.color == BLUE_SMALL || obj.color == BLUE_BIG || obj.color == PURPLE_SMALL || obj.color == PURPLE_BIG || obj.cls == 7 || obj.color>=4)
                    continue;
                else{
                    cars_map[obj.cls].push_back(obj);
                }
            }
            else if (this->Blue_or_Red == 1)
            {
                if (obj.color == RED_SMALL || obj.color == RED_BIG || obj.color == PURPLE_SMALL || obj.color == PURPLE_BIG || obj.cls == 7 || obj.color>=4)
                    continue;
                cars_map[obj.cls].push_back(obj);
            }
    }

//    for(auto obj:objects){
//                cars_map[obj.cls].push_back(obj);
//    }

}



bool DetectorTool::bestArmor(ArmorObject *best_object)
{

    static int last_object_cls=-1;
    int max_area=-1;
    ArmorObject* max_area_armor;
    double min_dist=99999.0;
    ArmorObject min_dist_armor;
    bool flag=false;

//    if(&cars_map[0][0]!=NULL){
//        *best_object=cars_map[0][0];
//        cout<<cars_map[0][0].color<<"cars_map[0][0]"<<endl;
//        cout<<(*best_object).color<<"best_object"<<endl;
//        return 1;
//    }
//    else return 0;


    if(last_object_cls==-1){

          for(int i=0;i<8;i++){
              if(/*&cars_map[i][0]!=NULL*/1){
                if(cars_map[i].size()>0){

                    for(auto object:this->cars_map[i]){
                        cv::Point2f armor_center=cv::Point2f((object.apex[0].x+object.apex[2].x)/2,(object.apex[0].y+object.apex[2].y)/2);
                        double camera_point_dist=POINT_DIST(armor_center,CAMERA_CENTER);
                        if(min_dist>camera_point_dist){

                            min_dist=camera_point_dist;
                            min_dist_armor=object;
                            flag=true;
                        }
                    }
                }
              }
            }

          if(flag==false) return 0;
          else{

            *best_object=min_dist_armor;
            last_object_cls=min_dist_armor.cls;

            return 1;
          }
    }


    else{

        if(cars_map[last_object_cls].size()<=0){
            for(int i=0;i<8;i++){
              if(/*&cars_map[i][0]!=NULL*/1){
                if(this->cars_map[i].size()>0){
                for(auto object:this->cars_map[i]){

                        cv::Point2f armor_center=cv::Point2f((object.apex[0].x+object.apex[2].x)/2,(object.apex[0].y+object.apex[2].y)/2);
                        double camera_point_dist=POINT_DIST(armor_center,CAMERA_CENTER);
                        if(min_dist>camera_point_dist){
                            min_dist=camera_point_dist;
                            min_dist_armor=object;
                            flag=true;
                        }
                }
                    if(flag==false) return 0;
                    else{
                        *best_object=min_dist_armor;
                        last_object_cls=min_dist_armor.cls;
                        return 1;
                    }
                }
              }
            }
        }
        else{
            int i=last_object_cls;
             if(this->cars_map[i].size()==1){
                 *best_object=cars_map[i][0];
                 last_object_cls=cars_map[i][0].cls;
             }
             else if(this->cars_map[i][0].area>this->cars_map[i][1].area){
                      *best_object=cars_map[i][0];
                      last_object_cls=cars_map[i][0].cls;
             }
             else{
                  *best_object=cars_map[i][1];
                  last_object_cls=cars_map[i][1].cls;
             }
             return 1;
        }


        return 0;
    }
}

void DetectorTool::setColor(int color_num)
{
    this->Blue_or_Red=(bool) color_num;
}


