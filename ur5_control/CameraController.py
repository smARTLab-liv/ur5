import cv2
import threading
from std_msgs.msg import String
import rospy
import pathlib


class CameraController:

    def __init__(self, folder=False):
        self.folder = folder
        # rospy.Subscriber("gelsight", String, lambda x : self.callback(x))
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # cv2.namedWindow('frame')
        if self.folder:
            pathlib.Path(self.folder).mkdir(parents=True)
        self.capture_and_display()

    def capture_img(self):
        frame = None
        for i in range(30):
            ret, frame = self.cap.read()
            key = cv2.waitKey(1)
        return frame

    #
    def capture_and_display(self):
        frame = self.capture_img()
        cv2.imshow('frame', frame)
        key = cv2.waitKey(10)
        return frame

    def capture_display_and_save(self, i):
        frame = self.capture_and_display()
        if self.folder:
            cv2.imwrite(str(self.folder) + str(i) + '.png', frame)


    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
