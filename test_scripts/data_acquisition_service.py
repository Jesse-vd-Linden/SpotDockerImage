# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use the data acquisition client"""

import argparse
import sys
import time

import cv2
import numpy as np

from google.protobuf.struct_pb2 import Struct

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import data_acquisition_pb2
from bosdyn.client.data_acquisition import DataAcquisitionClient
from bosdyn.client.data_acquisition_helpers import (acquire_and_process_request,
                                                    cancel_acquisition_request, download_data_REST,
                                                    issue_acquire_data_request,
                                                    make_time_query_params)
from bosdyn.client.image import build_image_request


def data_acquisition():
    """A simple example of using the data acquisition client to request data and check status."""

    sdk = bosdyn.client.create_standard_sdk('DataAcquisitionClientExample')
    robot = sdk.create_robot("192.168.1.109")
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    # Create data acquisition clients
    data_acq_client = robot.ensure_client(DataAcquisitionClient.default_service_name)

    now = robot.time_sync.robot_timestamp_from_local_secs(time.time())
    group_name = f'DataAcquisitionExample_{now.ToJsonString().replace(":", "-")}'

    # Get the start time so we can download all data from this example.
    start_time_secs = time.time()
    
    capture_size = 100
    for i in range(capture_size):
        # Request 1 contains only internal metadata capture requests and an image request.
        acquisition_requests = data_acquisition_pb2.AcquisitionRequestList()
        acquisition_requests.image_captures.extend([
            data_acquisition_pb2.ImageSourceCapture(
                image_service='image', image_request=build_image_request('back_fisheye_image'))
        ])
        acquisition_requests.data_captures.extend([
            data_acquisition_pb2.DataCapture(name='robot-state'),
            data_acquisition_pb2.DataCapture(name='detailed-position-data'),
            data_acquisition_pb2.DataCapture(name='basic-position-data')
        ])

        acquire_and_process_request(data_acq_client, acquisition_requests, group_name,
                                    'InternalAcquisitions')

    # Get the end time, and download all the data from the example.
    
    end_time_secs = time.time()
    print(f"Taking {capture_size} images took {end_time_secs - start_time_secs} seconds")
    query_params = make_time_query_params(start_time_secs, end_time_secs, robot)
    download_data_REST(query_params, "192.168.1.109", robot.user_token, destination_folder='.')


def main():
    data_acquisition()


if __name__ == '__main__':
    if not main():
        sys.exit(1)