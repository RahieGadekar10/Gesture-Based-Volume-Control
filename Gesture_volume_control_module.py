import mediapipe as mp
import numpy as np
import cv2
import time
from ctypes import cast, POINTER
import pycaw
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import math
from playsound import playsound


class detector():

    def __init__(self, mode=False, num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.7,
                 max_num_hands=1):
        self.mode = mode
        self.num_hands = num_hands
        self.min_detection_confidence = min_detection_confidence
        self.max_detection_confidence = min_tracking_confidence
        self.max_num_hands = max_num_hands

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(self.mode, self.num_hands,
                                         self.min_detection_confidence, self.max_detection_confidence)

    def landmarks(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.flip(frame, 1)
        self.results = self.hands.process(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if self.results.multi_hand_landmarks:
            for hand in self.results.multi_hand_landmarks:
                # ****************Commented the line to hide the detections**********************
                continue
                # self.mp_draw.draw_landmarks(frame , hand , self.mp_hands.HAND_CONNECTIONS)
        return frame

    def list_points(self, frame):
        points = []
        if self.results.multi_hand_landmarks:
            for hand in self.results.multi_hand_landmarks:
                for id, lm in enumerate(hand.landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    points.append([id, cx, cy])
        return points

