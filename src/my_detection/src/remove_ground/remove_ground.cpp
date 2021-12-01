/*
 *                                                     __----~~~~~~~~~~~------___
 *                                    .  .   ~~//====......          __--~ ~~
 *                    -.            \_|//     |||\\  ~~~~~~::::... /~
 *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
 *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
 *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *           '         ~-|      /|    |-~\~~       __--~~
 *                       |-~~-_/ |    |   ~\_   _-~            /\
 *                            /  \     \__   \/~                \__
 *                        _--~ _/ | .-~~____--~-/                  ~~==.
 *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                  -_     ~\      ~~---l__i__i__i--~~_/
 *                                  _-~-__   ~)  \--______________--~~
 *                                //.-~~~-~_--~- |-------~~~~~~~~
 *                                       //.-~~~--\
 *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *                               神兽保佑            永无BUG
 */


/*
 * @Description: 
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-10 21:24:52
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-12 14:49:36
 */#include"remove_ground.h"

RemoveGround::RemoveGround(ros::NodeHandle &nh,ros::NodeHandle &private_nh)
{
    private_nh.param<float>("max_radius", max_radius, 0.0);
    private_nh.param<float>("delta_radius_", delta_radius_, 0.2);
    private_nh.param<float>("delta_angle_", delta_angle_, 0.18);
    private_nh.param<float>("SENSOR_HEIGHT", SENSOR_HEIGHT, 2.2);
    private_nh.param<float>("min_delta_height_", min_delta_height_, 0.05);
    private_nh.param<float>("local_max_slope", local_max_slope, 8.0);
    private_nh.param<float>("global_max_slope", global_max_slope, 5.0);
    private_nh.param<float>("max_delta_radius_", max_delta_radius_, 0.2);
    private_nh.param<int>("radial_num", radial_num, 2000);


}



 void RemoveGround::Process(const PtC::Ptr &in_cloud,PtC::Ptr &non_ground_cloud,PtC::Ptr &ground_cloud)
 {
    
    vector<vector<AR> > out;//out是个二维列表,数据类型为AR
    XYZI2AR(in_cloud,out);//转换格式
    
    Segment(out,non_ground_cloud,ground_cloud);//分割地面
    
 }
RemoveGround::~RemoveGround()
{}

 void RemoveGround::XYZI2AR(const PtC::Ptr &in_cloud,vector<vector<AR> > &out)//XYZI格式转化成极坐标
 {  
    out.resize(radial_num);
    for (size_t i=0;i<in_cloud->size();++i)
    {
        Pt point =in_cloud->points[i];
        AR trans_point;
        trans_point.angle=atan2(point.y,point.x)*180/PI;//角度
        if(trans_point.angle<0)
        {
            trans_point.angle+=360;
        }
        trans_point.point=point;//原始坐标

        trans_point.radius=CalculateR(point);//半径

        if(trans_point.radius>max_radius)
        {
            max_radius=trans_point.radius;
        }

        trans_point.angle_index=(size_t)floor(trans_point.angle/delta_angle_);//角度序号

        trans_point.radius_index=(size_t)floor(trans_point.radius/delta_radius_);//半径序号

       // out_indices[trans_point.angle_index].push_back(i);
        
        out[trans_point.angle_index].push_back(trans_point);
        
    }
    
    for (size_t i=0;i<out.size();++i)
    {
        sort(out[i].begin(),out[i].end(),[](const AR &a,const AR &b){return a.radius<b.radius;});//按半径升序排列
    }
    
 }

//划分地面与非地面
void RemoveGround::Segment(vector<vector<AR> >&out,PtC::Ptr &non_ground_cloud,PtC::Ptr &ground_cloud)
{
    //PtC::Ptr unfiltered_non_ground_cloud(new PtC);
    //遍历点云
    for(size_t i=0;i<out.size();++i)//遍历每个角度
    {
        float prev_height=-SENSOR_HEIGHT;//上一条射线点的z值，初始设为传感器高度
        float prev_radius=0.0;//上一条射线点的半径，初始设为0
        bool prev_ground=false;//上一条射线点是否属于地面点
        bool current_ground=false;//此条射线点是否属于地面点
        for(size_t j=0;j<out[i].size();++j)//遍历每个角度上的每条射线
        {
            float current_height=out[i][j].point.z;//当前射线z值
            float current_radius=out[i][j].radius;//当前射线半径
            float distance_radius=current_radius-prev_radius;//前后两条射线半径差
            float distance_height=current_height-prev_height;//前后射线
            float distance_height_threshold=distance_radius*tan(DEG2RAD(local_max_slope));//前后点高度差值阈值
            //float height_threshold=current_radius*tan(DEG2RAD(global_max_slope));
            float height_threshold=current_radius*tan(DEG2RAD(global_max_slope))*(1-current_radius/(max_radius+0.1));//当此点处于最大坡度时，距离地面高度阈值
            //当两点距离过小时，设高度距离为min_delta_height_
            if((distance_radius>delta_radius_)&&(distance_height<min_delta_height_))
            {
                distance_height=min_delta_height_;
            }
                
            //    |distance_height|<distance_height_threshold
            if((distance_height<=distance_height_threshold)&&(distance_height>=-distance_height_threshold))
            {
                if (prev_ground)
                {
                    current_ground=true;//上一点为地面，设此点也为地面
                }
                else
                {
                    //|SENSOR_HEIGHT+current_height|<current_radius*tan(global_max_slope)
                    if(((SENSOR_HEIGHT+current_height)<=height_threshold)&&((SENSOR_HEIGHT+current_height)>=-height_threshold))
                    {
                        current_ground=true;
                    }
                    else
                    {
                        current_ground=false;
                    }
                }
            }

            else
            {
                //当两点半径距离大于最大阈值时，以下面条件判断
                if((distance_radius>max_delta_radius_)&&
            ((SENSOR_HEIGHT+current_height)<=height_threshold)&&
            ((SENSOR_HEIGHT+current_height)>=-height_threshold))
                {
                    current_ground=true;
                }
                else
                {
                    current_ground=false;
                }
            }
            if(!current_ground)//输出非地面点
            {
                non_ground_cloud->points.push_back(out[i][j].point);
            }
            else
            {
                ground_cloud->points.push_back(out[i][j].point);
            }
            prev_height=current_height;//更新高度，半径
            prev_radius=current_radius;
        }
    }
    // //去除部分未滤除的点
    // pcl::StatisticalOutlierRemoval<Pt> sor;
    // sor.setInputCloud(unfiltered_non_ground_cloud);
    // sor.setMeanK(50);//20个临近点
    // sor.setStddevMulThresh(1.2);//距离大于1倍标准方差

    // sor.filter(*non_ground_cloud);

}
