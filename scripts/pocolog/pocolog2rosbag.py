import os
import argparse
import cv2 as cv
import math
import numpy as np
import tqdm
from pocolog_pybind import *
from pathlib import Path

# ROS
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Pocolog to ROSbag')
    parser.add_argument('pocolog_file', type=str, help='Path to pocolog.log file')
    parser.add_argument('--port_image', '-pi', type=str, default='/camera_spinnaker.image_frame', help='Pocolog port for images')
    parser.add_argument('--topic_image', '-ti', type=str, default='/camera_spinnaker/image_raw', help='ROS topic name for images')
    parser.add_argument('--port_pose', '-pp', type=str, default='/vrpn.pose_sample', help='Pocolog port for poses')
    parser.add_argument('--topic_pose', '-tp', type=str, default='/vrpn/pose', help='ROS topic name for poses')
    parser.add_argument('--port_events', '-pe', type=str, default='', help='Pocolog port for events')
    parser.add_argument('--topic_events', '-te', type=str, default='', help='ROS topic name for events')
    parser.add_argument('--port_imu', '-pimu', type=str, default='', help='Pocolog port for imu data')
    parser.add_argument('--topic_imu', '-timu', type=str, default='', help='ROS topic name for imu data')
    parser.add_argument('--flip_y', action='store_true', help='Flip the image along y-axis')

    args = parser.parse_args()

    log_filepath = Path(args.pocolog_file)
    output_file = log_filepath.parent / (log_filepath.stem + '.bag')

    port_image = args.port_image
    topic_image = args.topic_image
    port_pose = args.port_pose
    topic_pose = args.topic_pose
    port_events = args.port_events
    topic_events = args.topic_events
    port_imu = args.port_imu
    topic_imu = args.topic_imu

    if os.path.exists(output_file):
        print('Detected existing rosbag: {}.'.format(output_file))
        print('Will overwrite the existing bag.')
    colors = {'reset':'\033[0m', 'green':'\033[32m'}
    print(colors['green'] + "** Conveting Pocolog to ROSbag file: {}".format(str(output_file)+colors['reset']))
    outbag = rosbag.Bag(str(output_file), 'w')

    multi_file_index = pocolog.MultiFileIndex()
    multi_file_index.create_index([str(log_filepath)])
    streams = multi_file_index.get_all_streams()

    #print(colors['green']+"[*] Images: "+colors['reset'], end="\n")
    stream = streams[port_image]
    idx, prev_ts = 0, None
    pbar = tqdm.tqdm(total=stream.get_size())
    for t in range(stream.get_size()):
        value = stream.get_sample(t)
        py_value = value.cast(recursive=True)
        value.destroy()
        ts = base.Time.from_microseconds(py_value['time']['microseconds'])
        height, width, channels = py_value['size']['height'], py_value['size']['width'], py_value['pixel_size']
        depth = py_value['data_depth']
        img = np.array(py_value['image'], dtype=np.uint8)
        img = img.reshape(height, width, channels)
        if args.flip_y: img = np.flip(img, axis=1)
        delta_t = ts.to_seconds() - prev_ts.to_seconds() if prev_ts is not None else 0
        #print("t: {} [{:.3f}] height {} width {} image {}".format(ts.to_seconds(), delta_t, height, width, img.size))
        try:
            stamp_ros = rospy.Time(nsecs=int(ts.to_microseconds()*1e03))
            rosimage = Image()
            rosimage.header.seq = t
            rosimage.header.stamp = stamp_ros
            rosimage.height = height
            rosimage.width = width
            rosimage.step = width * depth * channels
            rosimage.encoding = "rgb8"
            rosimage.data = img.tobytes()
            outbag.write(topic_image, rosimage, stamp_ros)
        except:
            print("error in writing images into rosbag ", output_file)
        #input('Press ENTER to continue...')
        prev_ts = ts
        idx = idx + 1
        pbar.update(1)

    #print(colors['green']+"[*] Poses: "+colors['reset'], end="\n")
    stream = streams[port_pose]
    idx, prev_ts = 0, None
    pbar = tqdm.tqdm(total=stream.get_size())
    for t in range(stream.get_size()):
        value = stream.get_sample(t)
        py_value = value.cast(recursive=True)
        value.destroy()
        ts = base.Time.from_microseconds(py_value['time']['microseconds'])
        delta_t = ts.to_seconds() - prev_ts.to_seconds() if prev_ts is not None else 0
        try:
            stamp_ros = rospy.Time(nsecs=int(ts.to_microseconds()*1e03))
            rospose = PoseStamped()
            rospose.header.seq = t
            rospose.header.stamp = stamp_ros
            rospose.header.frame_id = py_value['targetFrame']
            rospose.pose.position.x = py_value['position']['data'][0]
            rospose.pose.position.y = py_value['position']['data'][1]
            rospose.pose.position.z = py_value['position']['data'][2]
            rospose.pose.orientation.x = py_value['orientation']['im'][0]
            rospose.pose.orientation.y = py_value['orientation']['im'][1]
            rospose.pose.orientation.z = py_value['orientation']['im'][2]
            rospose.pose.orientation.w = py_value['orientation']['re']
            outbag.write(topic_pose, rospose, stamp_ros)
        except:
            print("error in writing poses into rosbag ", output_file)
        #input('Press ENTER to continue...')
        prev_ts = ts
        idx = idx + 1
        pbar.update(1)

    if port_events is not '' and topic_events is not '':
        from dvs_msgs.msg import Event, EventArray
        stream = streams[port_events]
        idx, prev_ts = 0, None
        pbar = tqdm.tqdm(total=stream.get_size())
        for t in range(stream.get_size()):
            value = stream.get_sample(t)
            py_value = value.cast(recursive=True)
            value.destroy()
            ts = base.Time.from_microseconds(py_value['time']['microseconds'])
            height, width = py_value['height'], py_value['width']
            try:
                stamp_ros = rospy.Time(nsecs=int(ts.to_microseconds()*1e03))
                rosevents = EventArray()
                rosevents.header.seq = t
                rosevents.header.stamp = stamp_ros
                rosevents.height = height
                rosevents.width = width
                for e in py_value['events']:
                    ts = rospy.Time(nsecs=int(e.ts.to_microseconds()*1e03))
                    rosevents.events.append(Event(e.x, e.y, ts, e.p))
                outbag.write(topic_events, rosevents, stamp_ros)
            except:
                print("error in writing events into rosbag", output_file)
            pbar.update(1)
            #input('Press ENTER to continue...')

    if port_imu is not '' and topic_imu is not '':
        stream = streams[port_imu]
        pbar = tqdm.tqdm(total=stream.get_size())
        for t in range(stream.get_size()):
            value = stream.get_sample(t)
            py_value = value.cast(recursive=True)
            value.destroy()
            ts = base.Time.from_microseconds(py_value['time']['microseconds'])
            acc = np.array(py_value['acc']['data'])
            gyro = np.array(py_value['gyro']['data'])
            try:
                stamp_ros = rospy.Time(nsecs=int(ts.to_microseconds()*1e03))
                rosimu = Imu()
                rosimu.header.seq = t
                rosimu.header.stamp = stamp_ros
                rosimu.orientation.x = np.nan
                rosimu.orientation.y = np.nan
                rosimu.orientation.z = np.nan
                rosimu.orientation.w = np.nan
                rosimu.angular_velocity.x = gyro[0]
                rosimu.angular_velocity.y = gyro[1]
                rosimu.angular_velocity.z = gyro[2]
                rosimu.linear_acceleration.x = acc[0]
                rosimu.linear_acceleration.y = acc[1]
                rosimu.linear_acceleration.z = acc[2]
                outbag.write(topic_imu, rosimu, stamp_ros)
            except:
                print("error in writing imu data into rosbag", output_file)
            pbar.update(1)
            #input('Press ENTER to continue...')

    if outbag is not None:
        outbag.close()
