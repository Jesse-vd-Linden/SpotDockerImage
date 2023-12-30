#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import pickle
import threading
from scipy.spatial.transform import Rotation as R
from collections import Counter
from pathlib import Path


class GestureClassificationNode:
    """_summary_    
    """
    
    def __init__(self):
        self.clf = pickle.load(open(Path(__file__).parent.parent.joinpath("models/Poly2SVC.model"), 'rb'))
        self.ppl = pickle.load(open(Path(__file__).parent.parent.joinpath("models/training.ppl"), 'rb')) # sklearn preprocessing pipeline
        self.pub_chatter = rospy.Publisher('/chatter', String, queue_size=2)
        self.frame_buffer = []
        self.sliding_window = 90
        self.last_gesture = "NoGesture"
        self.last_execution_time = time.time()
        self.execution_duration = 3 # duration for action execution

        self.label_decoding=[
            'fist',#'None'
            'turn_to_left',#'TurnLeft',
            'turn_to_right',#'TurnRight',
            'walk_to_forward', #'Forwards',
            'walk_to_backward',#'Backwards'
            'walk_to_left',#'GoLeft',
            'walk_to_right',#'GoRight',
        ]
        
        self.invalid_gesture = "NoGesture"
        self.vote_threshold = 0.7
        self.conf_threshold = 0.83

    def frequency_query(self, labels: np.ndarray):
        data = Counter(labels)
        return data.most_common(1)[0]

    def check_threshold(self, labels: np.ndarray, confidence: np.ndarray):
        # only when most labels are the same and the lowest confidence level is above 0.83, return the gesture
        label, frequency = self.frequency_query(labels)
        if frequency/len(labels) < self.vote_threshold:
            return self.invalid_gesture
        if confidence.mean() < self.conf_threshold:
            return self.invalid_gesture
        if label == 0: # manually check the fist gesture
            return self.invalid_gesture
        
        return self.label_decoding[label]

    def recognize_gesture(self):
        current_frame_buffer = np.array(self.frame_buffer).reshape(-1, 78)
        x = self.ppl.transform(current_frame_buffer)
        y = self.clf.predict_proba(x)
        
        labels = np.argmax(y, axis=1)
        confidence = np.max(y, axis=1)
        res = self.check_threshold(labels, confidence)
        current_time = time.time()
        if res != self.last_gesture:
            if res == "NoGesture":
                # publish and continue
                self.pub_chatter.publish(f"{res}")
                self.last_gesture = res
            else:
                # wait for the execution of the last valid action
                if current_time - self.last_execution_time >= self.execution_duration:
                    self.last_execution_time = current_time
                    self.pub_chatter.publish(f"{res}")
                    self.last_gesture = res

    def callback_gesture(self, data):
        if len(self.frame_buffer) < self.sliding_window:
            self.frame_buffer.append(data.data)
        else:
            self.frame_buffer.pop(0)
            self.frame_buffer.append(data.data)

            self.recognize_gesture()
    
    def callback_show_keypoints(self, data):
        array = np.array(data.data).reshape(-1,78)
        self.hand_keypoint_list.append(array)
        print(array)

    def run(self):
        rospy.init_node('gesture_recognizer', anonymous=True)
        rospy.Subscriber("/hand_keypoints", Float32MultiArray, self.callback_gesture)
        rospy.spin()

if __name__ == "__main__":

    gc = GestureClassificationNode()
    gc.run()