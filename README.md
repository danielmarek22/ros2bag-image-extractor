# ros2bag-image-extractor
Simple utility script for extracting images from database created by ros2 bag. 
The nodes I have been working with were something like this: `camera_left/raw`, `camera_right/raw`, `camera_depth/depth`. This script is designed to extract images published from this kind of nodes, but it should be easy to adjust it for any other type of Image publisher.
