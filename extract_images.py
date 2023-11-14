#!/usr/bin/env python

import os
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
import argparse


def convert_db3_to_images(db_file, output_dir):
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
        if topic.endswith('/raw'):
            try:
                # Deserialize the message
                deserialized_msg = deserialize_message(msg, Image)
                deserialized_img = np.asarray(deserialized_msg.data)

                # Get back the original dimensions of image
                img_shape = (deserialized_msg.height, deserialized_msg.width, 3)
                deserialized_img = np.reshape(deserialized_img, img_shape)
 
                # Extract the timestamp
                timestamp = deserialized_msg.header.stamp

                # Save the image to a file
                image_filename = os.path.join(output_dir, 'image_{}_{}.jpg'.format(timestamp.sec, timestamp.nanosec))
                cv2.imwrite(image_filename, deserialized_img)
                cv2.imshow("deserialized img",deserialized_img)
                # cv2.waitKey()

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

    args = parser.parse_args()        

    # Specify the path to the .db3 file
    db_file_path = args.db_file_path

    # Specify the output directory for images
    output_directory = args.output_folder

    # Convert the .db3 file to images
    convert_db3_to_images(db_file_path, output_directory)