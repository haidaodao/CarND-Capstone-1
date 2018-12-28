from styx_msgs.msg import TrafficLight
import cv2
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = TrafficLight.UNKNOWN
        # HSV allows count color within hue range
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        RED_MIN1 = np.array([0, 100, 100], np.unit8)
        RED_MAX1 = np.array([10, 255, 255], np.uint8)

        RED_MIN2 = np.array([160, 100, 100], np.unit8)
        RED_MAX2 = np.array([179, 255, 255], np.uint8)

        red_mask1 = cv2.inRange(hsv_img, RED_MIN1, RED_MAX1)
        red_mask2 = cv2.inRange(hsv_img, RED_MIN2, RED_MAX2)
        if cv2.countNonZero(red_mask1) + cvcv2.countNonZero(red_mask1) > 50:
            light = TrafficLight.RED

        YELLOW_MIN = np.array([40.0/360*255, 100, 100], np.unit8)
        YELLOW_MAX = np.array([66.0/360*255, 255, 255], np.uint8)

        yellow_mask = cv2.inRange(hsv_img, YELLOW_MIN, YELLOW_MAX)
        if cv2.countNonZero(yellow_mask) > 50:
            light = TrafficLight.YELLOW

        GREEN_MIN = np.array([40.0/360*255, 100, 100], np.unit8)
        GREEN_MAX = np.array([66.0/360*255, 255, 255], np.uint8)

        green_mask = cv2.inRange(hsv_img, GREEN_MIN, GREEN_MAX)
        if cv2.countNonZero(green_mask) > 50:
            light = TrafficLight.GREEN

        return light
