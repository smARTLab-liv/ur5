#! /usr/bin/env python

import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

background = None

MAX_DIFF = (640 * 480 * 3 * 255)


def nothing(x):
    pass


def touch_area(img1, img2, threshold):
    bw_img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    bw_img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    diff = cv2.absdiff(bw_img1, bw_img2)

    ret, binary = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    return binary


def absolute_square_diff(img1, img2):
    return cv2.absdiff(img1, img2)
    # abs = np.absolute(diff)
    # return np.square(diff)


def absolute_square_diff_sum(img):
    return np.sum(img)


def diff_to_percent(diff):
    return diff / MAX_DIFF


def capture():
    global background

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.CV_CAP_PROP_BUFFERSIZE, 3)

    cv2.namedWindow('frame')
    cv2.createTrackbar('T', 'frame', 0, 100, nothing)
    cv2.setTrackbarPos('T', 'frame', 15)

    # cv2.createTrackbar('PT', 'frame', 0, 255, nothing)
    # cv2.setTrackbarPos('PT', 'frame', 20)

    diffs = []
    intensities = []
    were_touching = False
    started_touching = False
    try:
        t0 = time.time()
        print('Skipping first frames...')
        while (time.time() - t0 < 5):
            cap.read()

        print('Capturing...')

        while (True):

            ret, frame = cap.read()

            kernel = np.ones((5, 5), np.float32) / 25
            frame = cv2.filter2D(frame, -1, kernel)

            if background is None:
                background = frame

            # px_threshold = cv2.getTrackbarPos('PT', 'frame')

            diff_frame = absolute_square_diff(background, frame)
            # binary_image = touch_area(background, frame, px_threshold)
            diff = diff_to_percent(absolute_square_diff_sum(diff_frame))

            # area = np.count_nonzero(binary_image)
            # intensity = 0 if area == 0 else (area / (640 * 480))
            # print(intensity)

            diffs.append(diff)
            # intensities.append(intensity)
            diffs = diffs[-10:]
            intensities = intensities[-10:]

            # print(diff)
            plt.clf()
            plt.plot(intensities, label='Intensities')
            plt.plot(diffs, label='Diffs')
            plt.legend()

            plt.ylim(0, 0.4)
            plt.ylabel('some numbers')
            plt.pause(0.01)
            # plt.show()

            threshold = cv2.getTrackbarPos('T', 'frame') / 1000.0

            touching = diff > threshold

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

            cv2.imshow('frame', frame)
            #
            # cv2.imshow('diff', diff_frame)
            # cv2.imshow('background', background)
            # cv2.imshow('binary', binary_image)

            key = cv2.waitKey(1) & 0xFF

            # if not were_touching and touching:
            #     started_touching = True
            #
            # if were_touching and not touching:
            #     started_touching = False
            #
            # if not started_touching and not touching:
            #     background = frame
            #
            # if intensity < 0.15:
            #     background = ((0.9 * background) + (0.1 * frame)).astype(np.uint8)

            were_touching = touching
        # ret, frame = cap.read()
        # key = cv2.waitKey(1000) & 0xFF
        # gray = frame  # cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # if key == ord('q'):
        #     break
        # elif key == ord('c') and to_folder:
        #     status = cv2.imwrite(str(to_folder) + str(i) + '.png', gray)
        #     if status:
        #         print("Image written to file-system : ", str(i))
        #         i += 1

        # Display the resulting frame

    except KeyboardInterrupt:
        cap.release()
        cv2.destroyAllWindows()
        plt.close()
        raise


if __name__ == '__main__':
    capture()
