from copy import copy
import numpy as np
import rospy as rp
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray


class GridMap(object):
    def __init__(self):
        self.map = None
        self.start = None
        self.end = None
        self.resolution = None
        self.width = None
        self.height = None
        self.delta_t = 2.0 # 10 ms
        self.L = 0.04

        self.parent = {}
        rp.init_node('graph_search', log_level=rp.DEBUG)
        rp.Subscriber('map', OccupancyGrid, self.map_callback)
        rp.Subscriber('point_start', Marker, self.set_start)
        rp.Subscriber('point_end', Marker, self.set_end)
        self.path_pub = rp.Publisher('path', Path, queue_size=10)
        self.search_pub = rp.Publisher('search', Marker, queue_size=10)
        while self.map is None or self.start is None or self.end is None:
            rp.sleep(0.1)
        print("Object initialized!")

    def map_callback(self, data):
        self.resolution = data.info.resolution
        self.width = data.info.width * self.resolution
        self.height = data.info.height * self.resolution
        map = np.array(data.data)
        map = np.reshape(map, (data.info.height, data.info.width))
        #map = np.reshape(map, (data.info.width, data.info.height))
        #map = np.transpose(map)
        self.map = map

    def get_marker_xy(self, marker):
        x = marker.pose.position.x
        y = marker.pose.position.y
        return x, y

    def set_start(self, data):
        x, y = self.get_marker_xy(data)
        theta = np.pi  #np.pi/2
        self.start = (x, y, theta, 0, 0)

    def set_end(self, data):
        x, y = self.get_marker_xy(data)
        self.end = (x, y, 0, 0, 0)

    def publish_search(self):
        marker = Marker()
        def add_point(p):
            pt = Point()
            pt.x = p[0]
            pt.y = p[1]
            pt.z = 0.
            marker.points.append(pt)
        marker.header.frame_id = "map"
        marker.header.stamp = rp.Time.now()
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.color.r = 0.
        marker.color.g = 0.
        marker.color.b = 1.
        marker.color.a = 0.5
        marker.scale.x = 0.1 * self.resolution
        for k, v in self.parent.items():
            if v is None: continue
            add_point(k)
            add_point(v)
        self.search_pub.publish(marker)

    @staticmethod
    def dist(pos1, pos2):
        distance = ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)**0.5
        return distance

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for index_p, p in enumerate(path):
            if index_p + 1 < len(path):
                u_s = path[index_p + 1][3]
                u_phi = path[index_p + 1][4]
                for iteration in range(1, 101):
                    theta_t = p[2] + (u_s/100 * iteration) / self.L * np.tan(u_phi) * self.delta_t
                    x_t = p[0] + (u_s/100 * iteration) * np.cos(theta_t) * self.delta_t
                    y_t = p[1] + (u_s/100 * iteration) * np.sin(theta_t) * self.delta_t
                    pose = PoseStamped()
                    pose.pose.position.x = x_t
                    pose.pose.position.y = y_t
                    pose.pose.position.z = 0.001
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 1
                    pose.header.frame_id = 'map'
                    pose.header.stamp = rp.Time.now()
                    path_msg.poses.append(pose)
                    if self.dist(self.end, (x_t, y_t)) < 0.01:
                        break
        self.path_pub.publish(path_msg)

