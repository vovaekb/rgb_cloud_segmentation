# rgb_cloud_segmentation
Application for the segmentation of RGB-D point cloud of scene by means of OpenCV MeanShift method.

Idea: research on methods for segmenting RGB-D point cloud observed from the viewpoint above. 
Problem: humanoid robot PR2 explores the table from above. Some objects laying on the table have very thin boundaries (e.g. cups) when observed from the viewpoint above thus making detecting such objects difficult. 
During this project research on different image segmentation methods from OpenCV library was carried out.
As a result MeanShift method shown best results and was choosen for segmenting RGB-D point cloud of scene.


## Using
Compile project:
```
cd rgb_cloud_segmentation
mkdir build
cmake ..
make
```

Run application
```
./rgb_segment <scene_cloud>.pcd
```
RGB image of scene will be saved to scene_rgb.jpg. Segmentation result will be saved in areas.jpg.

![ScreenShot](https://raw.github.com/vovaekb/rgb_cloud_segmentation/master/images/scene_rgb.jpg)
![ScreenShot](https://raw.github.com/vovaekb/rgb_cloud_segmentation/master/images/areas.jpg)
