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
import Gesture_volume_control_module as gvc

def main():
    cap = cv2.VideoCapture(0)
    ptime = 0
    ctime = 0
    detect = gvc.detector()
    length = 0
    devices = AudioUtilities.GetSpeakers()
    interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
    volume = cast(interface, POINTER(IAudioEndpointVolume))

    while True:
        ret, frame = cap.read()
        frame = detect.landmarks(frame)
        points = detect.list_points(frame)
        if len(points) != 0:
            x1, y1 = points[4][1], points[4][2]
            x2, y2 = points[8][1], points[8][2]
            length = math.hypot(x2 - x1, y2 - y1)
            volrange = volume.GetVolumeRange()
            maxvol = volrange[1]
            minvol = volrange[0]
            print(minvol)
            vol = np.interp(length, [30, 300], [minvol, maxvol])
            volume.SetMasterVolumeLevel(vol, None)
            if vol == maxvol:
                cv2.putText(frame, "Volume Max", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            if vol == minvol:
                cv2.putText(frame, "Volume Min", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            # minvol, maxvol, volume = detect.getvolume()
            if ((points[20][2]) > (points[18][2])):
                volume.SetMasterVolumeLevel(vol, None)
                time.sleep(1.4)
        ctime = time.time()
        fps = 1 / (ctime - ptime)
        ptime = ctime

        cv2.putText(frame, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0XFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
