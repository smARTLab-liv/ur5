import cv2
import threading
from std_msgs.msg import String
import rospy
import pathlib

MAX_DIFF = (640 * 480 * 3 * 255)


def absolute_square_diff(img1, img2):
    return cv2.absdiff(img1, img2)
    # abs = np.absolute(diff)
    # return np.square(diff)


def absolute_square_diff_sum(img):
    return np.sum(img)


def diff_to_percent(diff):
    return diff / MAX_DIFF


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
        self.background = self.capture_and_display()


    def capture_img(self):
        frame = None
        for i in range(30):
            ret, frame = self.cap.read()
            key = cv2.waitKey(1)
        return frame

    def capture_and_display(self):
        frame = self.capture_img()
        cv2.imshow('frame', frame)
        key = cv2.waitKey(10)
        return frame


    def detect_touch(self):
        frame = self.capture_img(self)
        diff_frame = absolute_square_diff(self.background, frame)
        diff = diff_to_percent(absolute_square_diff_sum(diff_frame))

        touching = diff > 0.015

        if (touching):
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (100, 200)
            fontScale = 1
            fontColor = (0, 0, 255)
            lineType = 2

            cv2.putText(frame, 'Touching!',
                        bottomLeftCornerOfText,
                        font,
                        fontScale,
                        fontColor,
                        lineType)
            print('touching')
        return touching




    def capture_display_and_save(self, i):
        frame = self.capture_and_display()
        if self.folder:
            cv2.imwrite(str(self.folder) + str(i) + '.png', frame)


    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
