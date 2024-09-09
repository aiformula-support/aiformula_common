# Compressed Image Viewer

## Package Description
Subscribe `sensor_msgs::msg::CompressedImage`, display it by `cv::imshow`.

## Usage
```sh
$ colcon build --symlink-install --packages-up-to compressed_image_viewer
$ ros2 launch compressed_image_viewer compressed_image_viewer.launch.py
```

## Subscriber & Publisher
- Subscribe
    - `/aiformula_visualization/zed/left_image/compressed`
