Located here are several python conversion scripts used to parse and orgnaize the raw data collected from rosbags. If you would like to use them and repurpose them, install the following dependencies.

The `local_odom_conversion.py` script provides conversion from gnss latitude/longitude/altitude data in the rosbags to a cartesian coordinate frame. It also writes and parses out specific sensor topics to a new bag.

```
pip install pygeodesy rosbags
```

The `bag_to_images.py` script converts images stored in the bag file to JPG files.

```
pip install opencv-python
```