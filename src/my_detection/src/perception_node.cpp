
/*
 * @Description: 
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-15 21:46:34
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-25 10:47:33
 */
/*
 * 
 *    ┏┓　　　┏┓
 *  ┏┛┻━━━┛┻┓
 *  ┃　　　　　　　┃
 *  ┃　　　━　　　┃
 *  ┃　＞　　　＜　┃
 *  ┃　　　　　　　┃
 *  ┃...　⌒　...　┃
 *  ┃　　　　　　　┃
 *  ┗━┓　　　┏━┛
 *      ┃　　　┃　
 *      ┃　　　┃
 *      ┃　　　┃
 *      ┃　　　┃  神兽保佑
 *      ┃　　　┃  代码无bug　　
 *      ┃　　　┃
 *      ┃　　　┗━━━┓
 *      ┃　　　　　　　┣┓
 *      ┃　　　　　　　┏┛
 *      ┗┓┓┏━┳┓┏┛
 *        ┃┫┫　┃┫┫
 *        ┗┻┛　┗┻┛
 */



#include"perception.h"
#include<ros/ros.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"my_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    Perception perception(nh,private_nh);
    
    return 0;
}


