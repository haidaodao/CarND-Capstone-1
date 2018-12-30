import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image

class TLClassifier(object):
    def __init__(self):
        self.cropped_tl_bb_pub = rospy.Publisher('/cropped_bb', Image, queue_size=1)
        self.bridge = CvBridge()

    def detect_light_state(self, bb_image):
        height, width, channels = bb_image.shape


        # Crop respective red, yellow, and green sections
        red_area = bb_image[0:height//3, 0:width]
        yellow_area = bb_image[height//3:2*height//3, 0:width]
        green_area = bb_image[2*height//3:height, 0:width]

        coefficients_red = [0.1, 0.1, 0.8]
        coefficients_yellow = [0.114, .587, .299]
        coefficients_green = [0.1, 0.8, 0.1]

        red_area = cv2.transform(red_area, np.array(coefficients_red).reshape((1,3)))
        yellow_area = cv2.transform(yellow_area, np.array(coefficients_yellow).reshape((1,3)))
        green_area = cv2.transform(green_area, np.array(coefficients_green).reshape((1,3)))

        bb_image = np.concatenate((red_area, yellow_area, green_area), axis=0)

        height, width = bb_image.shape

        mask = np.zeros((height, width), np.unit8)

        width_offset = 3
        height_offset = 4
        cv2.ellipse(mask, (width//2, 1*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)
        cv2.ellipse(mask, (width//2, 3*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)
        cv2.ellipse(mask, (width//2, 5*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)

        bb_image = np.multiply(bb_image, mask)

        bb_image = cv2.inRange(bb_image, 210, 255)

        # Partition into Red, Yellow and Green Areas
        red_area = bb_image[0:height//3, 0:width]
        yellow_area = bb_image[height//3: 2*height//3, 0:width]
        green_area = bb_image[2*height//3: height, 0:width]
	    # Count the number of non-zero pixels
        red_count = cv2.countNonZero(red_area)
        yellow_count = cv2.countNonZero(yellow_area)
        green_count = cv2.countNonZero(green_area)

        # Publish the image for diagnostics
        self.cropped_tl_bb_pub.publish(self.bridge.cv2_to_imgmsg(bb_image, "mono8"))

        # Default state is unknown
        state = TrafficLight.UNKNOWN
	    # Determine which color had max non-zero pixels
        if red_count > yellow_count and red_count > green_count:
            print ('Red Light Detected!')
            state = TrafficLight.RED
        elif yellow_count > red_count and yellow_count > green_count:
            print ('Yellow Light Detected!')
            state = TrafficLight.YELLOW
        elif green_count > red_count and green_count > yellow_count:
            print ('Green Light Detected!')
            state = TrafficLight.GREEN
        else:
            print ('Warning! Unable to determine Light state')

        return state



    def get_classification(self, image, TL_BB_list, simulator_mode):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
         # if list is empty, return UNKNOWN
        if not TL_BB_list:
            return TrafficLight.UNKNOWN

        else:
            # We consider the BB with highest probability (at index 0)
            xmin = TL_BB_list[0].xmin
            xmax = TL_BB_list[0].xmax
            ymin = TL_BB_list[0].ymin
            ymax = TL_BB_list[0].ymax

            # cropped image
            bb_image = image[ymin:ymax, xmin:xmax]

            # Check if running in simulator mode
            if int(simulator_mode) == 1:

                light = TrafficLight.UNKNOWN
                # HSV allows count color within hue range
                hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                RED_MIN1 = np.array([0, 100, 100], np.unit8)
                RED_MAX1 = np.array([10, 255, 255], np.uint8)

                RED_MIN2 = np.array([160, 100, 100], np.unit8)
                RED_MAX2 = np.array([179, 255, 255], np.uint8)

                red_mask1 = cv2.inRange(hsv_img, RED_MIN1, RED_MAX1)
                red_mask2 = cv2.inRange(hsv_img, RED_MIN2, RED_MAX2)
                if cv2.countNonZero(red_mask1) + cvcv2.countNonZero(red_mask1) > 40:
                    print("RED!")
                    light = TrafficLight.RED

                YELLOW_MIN = np.array([40.0/360*255, 100, 100], np.unit8)
                YELLOW_MAX = np.array([66.0/360*255, 255, 255], np.uint8)

                yellow_mask = cv2.inRange(hsv_img, YELLOW_MIN, YELLOW_MAX)
                if cv2.countNonZero(yellow_mask) > 30:
                    print("YELLOW!")
                    light = TrafficLight.YELLOW

                GREEN_MIN = np.array([40.0/360*255, 100, 100], np.unit8)
                GREEN_MAX = np.array([66.0/360*255, 255, 255], np.uint8)

                green_mask = cv2.inRange(hsv_img, GREEN_MIN, GREEN_MAX)
                if cv2.countNonZero(green_mask) > 30:
                    light = TrafficLight.GREEN

                return light
            else:
                return self.detect_light_state(bb_image)
