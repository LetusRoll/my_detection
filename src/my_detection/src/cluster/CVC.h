/*
 * @Description: cluster
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-12 14:51:26
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-22 20:04:14
 */


#ifndef CVC_H
#define CVC_H

#include "my_detection/point_types.h"
#include <stdlib.h>
#include<vector>
#include <unordered_map>
#include<limits>
using std::vector;
using std::unordered_map;
#define PI 3.14159

struct APR
{
    float theta;//与X轴夹角
    float rho;//径长
    float phi;  //与XY平面夹角
};

struct Voxel
{
    //int cluster_id=-1;
    int theta_index;
    int rho_index;
    int phi_index;
    vector<int> index;//存储点云原始序号
    
};

inline float CalculateAngle(float &x,float &y)
{
    float temp_angle=0.0;
    if((x==0)&&(y==0))
    {
        temp_angle=0.0;
    }
    else if(y>=0)
    {
        temp_angle=(float)atan2(y,x)*180/PI;
    }
    else if(y<0)
    {
        temp_angle=(float)atan2(y,x)*180/PI+360;
    }
    return temp_angle;
};

inline bool compare_cluster(std::pair<int,int> a,std::pair<int,int> b)
{
    return a.second>b.second;
}

class CVC
{
    private:
    float min_rho=std::numeric_limits<float>::max();//rho的最小值
    float max_rho=std::numeric_limits<float>::min();//rho的最大值
    float min_phi;
    float max_phi;
    float delta_theta;
    float delta_rho;
    float delta_phi;//体素三个维度的最小单位
    int theta_num,rho_num,phi_num;//体素在三个维度上的个数
    //float max_dist_;//两个体素中心点的最大距离
    int min_cluster_num_;
    public:
    CVC(ros::NodeHandle &nh,ros::NodeHandle &private_nh);
    void Process(const PtC::Ptr &in_cloud,vector<int> &cluster_indices);
    ~CVC();
    //转换点云格式
    void CalculateAPR(const PtC::Ptr &in_cloud,vector<APR> &APR_cloud);
    //创建Hash表
    void BuildHashTable(const vector<APR> &APR_cloud,std::unordered_map<int,Voxel> &map);
    //聚类的主函数
    vector<int> Cluster(const PtC::Ptr &in_cloud,std::unordered_map<int,Voxel> &map,const vector<APR> &APR_cloud);
    //寻找近邻
    void FindNeighbors(const PtC::Ptr &in_cloud,std::unordered_map<int,Voxel> &map,const int &theta_index,const int &rho_index,const int &phi_index,vector<int> &neighbors);
    //融合两个序号不同的点云
    void Merge(vector<int> &cluster_indices,int a,int b);
    //删除掉个数小于10的点云簇
    void SelectMajorCluster(vector<int> &cluster_indices,vector<int> &cluster_index);
    //计算点云中心点坐标
    Pt CalculateCenter(const PtC::Ptr &in_cloud,int self_index,std::unordered_map<int,Voxel> &map);
    //计算两个点距离是否大于阈值
    bool CalculateDist(const Pt pt1,const Pt pt2);
};

#endif