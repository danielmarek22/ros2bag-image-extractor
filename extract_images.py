#!/usr/bin/env python

import os
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
import argparse

def deserialize_img(msg, dims):
    # Deserialize the message
    deserialized_msg = deserialize_message(msg, Image)
    deserialized_img = np.asarray(deserialized_msg.data)

    # Extract the timestamp for filename creation
    timestamp = deserialized_msg.header.stamp

    # Get back the original dimensions of image
    img_shape = (deserialized_msg.height, deserialized_msg.width, dims)
    deserialized_img = np.reshape(deserialized_img, img_shape)
    return (deserialized_img, timestamp)

def convert_db3_to_images(db_file, output_dir, image_type):
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Create StorageOptions
    storage_options = StorageOptions(
        uri=db_file,
        storage_id='sqlite3',
    )

    # Open the bag file
    reader = SequentialReader()
    converter_options = ConverterOptions('', '')

    # Open the bag file for reading
    reader.open(storage_options, converter_options)

    # Iterate through messages in the bag
    while reader.has_next():
        topic, msg, t = reader.read_next()

        # Check if the message is an image
        if image_type == 'raw':
            if topic.endswith('/raw'):
                try:
                    deserialized_img, timestamp = deserialize_img(msg, 3)
                    tokens = topic.split('/')
                    topic_name = tokens[2]
                    
                    image_filename = os.path.join(output_dir, '{}_{}_{}.jpg'.format(topic_name, timestamp.sec, timestamp.nanosec))

                    # Save the image to a file
                    cv2.imwrite(image_filename, deserialized_img)
                    print('Image saved to {}'.format(image_filename))

                except Exception as e:
                    print('Error processing image: {}'.format(e))

        if image_type == 'depth':
            if topic.endswith('/depth'):
                try:
                    deserialized_img, timestamp = deserialize_img(msg, 2)
                    image_filename = os.path.join(output_dir, 'image_depth_{}_{}.jpg'.format(timestamp.sec, timestamp.nanosec))

                    # Save image to a file
                    cv2.imwrite(image_filename, deserialized_img[:,:, 1])
                    print('Image saved to {}'.format(image_filename))

                except Exception as e:
                    print('Error processing image: {}'.format(e))
        


if __name__ == '__main__':
    # Specify argument parser for getting paths and types of stored data
    parser = argparse.ArgumentParser(description='Extract images from rosbag database file.')
    parser.add_argument('db_file_path', type=str,
                    help='path to the rosbag db3 file')
    parser.add_argument('output_folder', type=str,
            help='path to the rosbag db3 file')
    parser.add_argument('image_type', choices=['raw', 'depth'], type=str,
                        help='choose what type of image is extracted')

    args = parser.parse_args()        

    # Specify the path to the .db3 file
    db_file_path = args.db_file_path

    # Specify the output directory for images
    output_directory = args.output_folder

    # Specify the image type
    img_type = args.image_type

    # Convert the .db3 file to images
    convert_db3_to_images(db_file_path, output_directory, img_type)
