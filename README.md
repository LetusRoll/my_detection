# my_detection
Ground segmentation , point cloud clustering based on CVC(Curved Voxel Clustering) and polygonal box(classify vehicles simply by length,width and height)
本项目使用设置地面坡度阈值的方法，滤除地面点，使用三维弯曲体素聚类法完成点云的聚类，包围盒参数由Apollo库的common_lib和object_builders_lib
得出,代码还有一些中文注释。
运行步骤：

`https://github.com/C-Xingyu/my_detection.git`

`cd my_detection`

`catkin_make -DCATKIN_WHITELIST_PACKAGES="common_lib"`

`catkin_make -DCATKIN_WHITELIST_PACKAGES="object_builders_lib"`

`catkin_make -DCATKIN_WHITELIST_PACKAGES="my_detection"`

接下来配置launch文件（velodyne_points.launch）
运行

`roslaunch my_detection velodyne_points.launch`


![2021-11-02 20-09-33 的屏幕截图](https://user-images.githubusercontent.com/56507063/139843561-a8476c82-d2d6-4ba0-ae2f-7d7072e883cf.png)

![2021-11-02 20-38-47 的屏幕截图](https://user-images.githubusercontent.com/56507063/139848240-388eb313-17ac-4e8b-a8c0-f7002eeb63f5.png)
