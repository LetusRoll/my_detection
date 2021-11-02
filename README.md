# my_detection
Ground segmentation , point cloud clustering based on CVC(Curved Voxel Clustering) and polygonal box
本项目使用设置地面坡度阈值的方法，滤除地面点，使用三维弯曲体素聚类法完成点云的聚类，包围盒参数由Apollo库的common_lib和object_builders_lib
得出,代码还有一些中文注释。
运行步骤：

`git clone https://github.com/cxy329/my_detection.git`

`cd my_detection`

`catkin_make -DCATKIN_WHITELIST_PACKAGES="common_lib"`

`catkin_make -DCATKIN_WHITELIST_PACKAGES="object_builders_lib"`

`catkin_make -DCATKIN_WHITELIST_PACKAGES="my_detection"`

接下来配置launch文件（velodyne_points.launch）
运行

`roslaunch my_detection velodyne_points.launch`


![2021-11-02 20-06-18 的屏幕截图](https://user-images.githubusercontent.com/56507063/139843004-ac3e47df-af02-4c80-9968-ce5f72e6edb7.png)
