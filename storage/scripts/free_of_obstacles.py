#!/usr/bin/env python
from robotnik_msgs.msg import State
from cpswarm_msgs.srv import OutOfBounds, OutOfBoundsResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf
import math
import rospy

class out_of_map:

    def __init__(self):

        rospy.init_node("free_in_map_node")

        self.global_costmap_height = 5
        self.global_costmap_width = 5
        self.global_costmap_resolution = 0.05
        self.global_costmap_data = []

        self.local_costmap_height = 5
        self.local_costmap_width = 5
        self.local_costmap_resolution = 0.05
        self.local_costmap_data = []

        self.tf_listener = tf.TransformListener()
        
        # Subscribers
        # topic_name, msg type, callback, queue_size
        self.topic_sub = rospy.Subscriber('move_base/global_costmap/costmap', OccupancyGrid, self.globalcostmapCb, queue_size = 1)

        #self.topic_sub = rospy.Subscriber('move_base/local_costmap/costmap', OccupancyGrid, self.localcostmapCb, queue_size = 1)

        # Service Servers
        self.service_server = rospy.Service('~out_of_global_costmap', OutOfBounds, self.globalserviceCb)
        #self.service_server = rospy.Service('~free_in_local_costmap', OutOfBounds, self.localserviceCb)

    def start(self):
        rospy.spin()
        return

    def globalserviceCb(self,req):
        ret_outofmap = OutOfBoundsResponse()
        if abs(req.pose.position.x) > (self.global_costmap_width * self.global_costmap_resolution) or abs(req.pose.position.y) > (self.global_costmap_height * self.global_costmap_resolution) :
            ret_outofmap.out = False
            return ret_outofmap
        #rospy.loginfo('Pos x %d, y: %d' % (req.pose.position.x, req.pose.position.y))
        mx = (int)(abs((req.pose.position.x + self.global_costmap_origin_x)) / self.global_costmap_resolution)
        my = (int)(abs((req.pose.position.y + self.global_costmap_origin_y)) / self.global_costmap_resolution)
        #rospy.loginfo('Cell x %d, y: %d' % (mx, my))
        pos_costmap_array = my * self.global_costmap_width + mx
        #rospy.loginfo('Pos in array %d, size: %d' % (pos_costmap_array, len(self.global_costmap_data)))
        global_costmap_value = self.global_costmap_data[int(pos_costmap_array)-1]
        #rospy.loginfo('Costmap value in the coordinate %d' % global_costmap_value)
        #TODO maybe check surroundings
        if global_costmap_value>70:
            rospy.loginfo('free_of_obstacles at global_costmap: There is something in the coordinate')    
            ret_outofmap.out = False
        else:
            rospy.loginfo('free_of_obstacles at global_costmap: Place free')
            ret_outofmap.out = True
        return ret_outofmap

    #TODO: This is NOT finished. It is NOT working
    def localserviceCb(self,req):
        ret_outofmap = OutOfBoundsResponse()	
       
        if abs(req.pose.position.x) > (self.local_costmap_width * self.local_costmap_resolution) or abs(req.pose.position.y) > (self.local_costmap_height * self.local_costmap_resolution) :
            ret_outofmap.out = False
            return ret_outofmap
        rospy.loginfo('Pos x %d, y: %d' % (req.pose.position.x, req.pose.position.y))
        mx = (int)(abs((req.pose.position.x + self.local_costmap_origin_x)) / self.local_costmap_resolution)
        my = (int)(abs((req.pose.position.y + self.local_costmap_origin_y)) / self.local_costmap_resolution)
        rospy.loginfo('Cell x %d, y: %d' % (mx, my))
        pos_costmap_array = my * self.local_costmap_width + mx
        rospy.loginfo('Pos in array %d, size: %d' % (pos_costmap_array, len(self.local_costmap_data)))
        local_costmap_value = self.local_costmap_data[int(pos_costmap_array)-1]
        rospy.loginfo('Costmap value in the coordinate %d' % local_costmap_value)
        #TODO maybe check surroundings
        if local_costmap_value > 0:
            rospy.loginfo('There is something in the coordinate')    
            ret_outofmap.out = False
        else:
            rospy.loginfo('Place free')
            ret_outofmap.out = True
        return ret_outofmap
        
					
    def globalcostmapCb(self, msg):
        self.global_costmap_height = msg.info.height
        self.global_costmap_width = msg.info.width
        self.global_costmap_resolution = msg.info.resolution
        self.global_costmap_data = msg.data
        self.global_costmap_origin_x = msg.info.origin.position.x
        self.global_costmap_origin_y = msg.info.origin.position.y
        #rospy.loginfo('costmap resolution:%s' % self.global_costmap_resolution)
        #rospy.loginfo('costmap height:%s' % self.global_costmap_height)
        #rospy.loginfo('costmap width:%s' % self.global_costmap_width)
        #rospy.loginfo('costmap or_x:%s' % self.global_costmap_origin_x)
        #rospy.loginfo('costmap or_y:%s' % self.global_costmap_origin_y)

    def localcostmapCb(self, msg):
        self.local_costmap_height = msg.info.height
        self.local_costmap_width = msg.info.width
        self.local_costmap_resolution = msg.info.resolution
        self.local_costmap_data = msg.data
        self.local_costmap_origin_x = msg.info.origin.position.x
        self.local_costmap_origin_y = msg.info.origin.position.y
        rospy.loginfo('costmap resolution:%s' % self.local_costmap_resolution)
        rospy.loginfo('costmap height:%s' % self.local_costmap_height)
        rospy.loginfo('costmap width:%s' % self.local_costmap_width)
        rospy.loginfo('costmap or_x:%s' % self.local_costmap_origin_x)
        rospy.loginfo('costmap or_y:%s' % self.local_costmap_origin_y)


if __name__ == '__main__':
    arc_foo = out_of_map()
    arc_foo.start()