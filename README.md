# rgb_cloud_segmentation
Application for the segmentation of RGB-D point cloud of scene by means of OpenCV MeanShift method.

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
