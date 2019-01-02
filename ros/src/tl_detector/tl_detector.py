#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import PyKDL


from  scipy.spatial import KDTree
import math
import numpy as np

#from darknet_ros_msgs.msg import BoundingBox
#from darknet_ros_msgs.msg import BoundingBoxes

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.lights = []
        self.traffic_light_waypoint_indexes = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        #sub5 = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detected_bb_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        #self.simulator_mode = rospy.get_param("/simulator_mode")
        #self.simulator_mode = True

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        #if int(self.simulator_mode) == 0:
        #    self.cropped_tl_bb_pub = rospy.Publisher('/cropped_bb', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        #self.TL_BB_list = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        '''self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        '''

        self.waypoints = waypoints

        stop_line_positions = self.config['stop_line_positions']

        idx = 0
        for line in stop_line_positions:
            traffic_light = TrafficLight()
            traffic_light.pose = PoseStamped()
            traffic_light.pose.pose.position.x = line[0]
            traffic_light.pose.pose.position.y = line[1]
            traffic_light.pose.pose.position.z = 0.0
            self.traffic_light_waypoint_indexes.append([idx,self.get_closest_waypoint(traffic_light.pose.pose)])
            idx += 1

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location
        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
        # get transform between pose of camera and world frame

        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #Use tranform and rotation to calculate 2D position of light in image
        f = 2300
        x_offset = -30
        y_offset = 340
        fx = f
        fy = f
        piw = PyKDL.Vector(point_in_world.x,point_in_world.y,point_in_world.z)
        R = PyKDL.Rotation.Quaternion(*rot)
        T = PyKDL.Vector(*trans)
        p_car = R*piw+T

        # x = -p_car[1]/p_car[0]*fx+image_width/2
        # y = -p_car[2]/p_car[0]*fx+image_height/2
        x = -p_car[1]/p_car[0]*fx+image_width/2 + x_offset
        y = -p_car[2]/p_car[0]*fx+image_height/2+y_offset

        return (int(x), int(y))
    '''def detected_bb_cb(self, msg):
        # Clear the list
        self.TL_BB_list = []
        # Parameters: Diagnonal size thresholds
        simulator_bb_size_threshold = 85 #px
        site_bb_size_threshold = 40 #px
        # min probability of detection
        simulator_bb_probability = 0.85
        site_bb_probability = 0.25

        #if int(self.simulator_mode) == 1:
        prob_thresh = simulator_bb_probability
        size_thresh = simulator_bb_size_threshold
        #else:
        #    prob_thresh = site_bb_probability
        #    size_thresh = site_bb_size_threshold

        for bb in msg.bounding_boxes:
            # Simulator mode: Bounding Box class should be 'traffic light' with probability >= 85%
            # Site Mode: Bounding Box class should be 'traffic light' with probability >= 25%
            if str(bb.Class) == 'traffic light' and bb.probability >= prob_thresh:
                # Simulator mode: If diagonal size of bounding box is more than 85px
                # Site mode: If diagonal size of bounding box is more than 80px
                if math.sqrt((bb.xmin - bb.xmax)**2 + (bb.ymin - bb.ymax)**2) >= size_thresh:
                    self.TL_BB_list.append(bb)

                    # if running in site mode/ROS bag mode
                    if int(self.simulator_mode) == 0:
                        The ROS bag version only has video data. Hence no waypoints are loaded and get light function is not called.
                        So to check detection in ROS bag video, we do TL state classification here itself.

                        # Get the camera image
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                        # Crop image
                        bb_image = cv_image[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
                        self.light_classifier.detect_light_state(bb_image)
    '''
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = -1
        closest_dist = -1
        a = (pose.position.x, pose.position.y)
        distance = lambda a,b: (a[0]-b[0]) ** 2 + (a[1] - b[1]) ** 2
        if self.waypoints is not None:
            p = self.waypoints.waypoints
            for i in range(len(p)):
                b = (p[i].pose.pose.position.x,p[i].pose.pose.position.y)
                d = distance(a,b)

                if (closest_dist==-1) or (closest_dist>d):
                    closest_dist = d
                    closest_index = i
        #closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        x, y = self.project_to_image_plane(light.pose.pose.position)

        if (x<0) or (y<0) or (x>=cv_image.shape[1]) or (y>=cv_image.shape[0]):
            return False


        imm = cv_image
        crop = 90
        xmin = x - crop if (x-crop) >= 0 else 0
        ymin = y - crop if (y-crop) >= 0 else 0

        xmax = x + crop if (x + crop) <= imm.shape[1]-1 else imm.shape[1]-1
        ymax = y + crop if (y + crop) <= imm.shape[0]-1 else imm.shape[0]-1
        imm_cropped = imm[ymin:ymax,xmin:xmax]
        # cv2.imshow("test",imm_cropped)
        # cv2.waitKey(5)
        return self.light_classifier.get_classification(imm_cropped)


    '''def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose.pose)
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
        if closest_light:
            state = self.get_light_state(closest_light)
            return light_wp_idx, state

        return -1, TrafficLight.UNKNOWN
    '''
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """


        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']

        car_position_index = 100000 #MAX initialize
        if(self.pose and self.waypoints):
            car_position_index = self.get_closest_waypoint(self.pose.pose)


        #TODO find the closest visible traffic light (if one exists)


        for traffic_light_waypoint_indexe in self.traffic_light_waypoint_indexes:
            if traffic_light_waypoint_indexe[1] > car_position_index:
                light = True
                light_wp = traffic_light_waypoint_indexe[1]
                tl_idx = traffic_light_waypoint_indexe[0]
                break
        # print('n lights {} n tl_waypoints {}, index {}'.format(
        #     len(self.lights),len(self.traffic_light_waypoint_indexes),tl_idx))
        if light:
            state = self.get_light_state(self.lights[tl_idx])
            # print('light found is a ',state)
            # print ' [-] light_wp = '
            # print light_wp
            # print ' [-] stopline wp :'
            # print self.waypoints.waypoints[light_wp]
            #
            # print ' ------------------------ '
            return light_wp, state
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
