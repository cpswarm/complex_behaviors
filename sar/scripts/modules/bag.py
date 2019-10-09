#!/usr/bin/python


import math
import shapely.geometry as geo
import xml.etree.ElementTree as xml
import rosbag


class Bag:
    '''
    a class for storing and analyzing bag file contents
    '''

    def __init__(self, folder, file, launch_file):
        '''
        initialize class
        :param string folder: absolute path of the bag file directory
        :param string file: name of the bag file
        :param string launch_file: launch file that was launched to generate the bag file
        '''
        # absolute path of the bag file directory
        self.folder = folder

        # name of the bag file
        self.file = file

        # id of bag file
        self.id = file.split('_')[3]

        # coverage starting time
        self.tstart = 0.0

        # coverage ending time
        self.tstop = 0.0

        # first target found time
        self.found = 0.0

        # first target rescued time
        self.rescued = 0.0

        # list of goal coordinates
        self.goal = []

        # list pose coordinates
        self.pose = []

        # number of messages in bag file
        self.bag_msg_count = 0

        # topics in bag file
        self.bag_topics = {}

        # the fov of the UAV in meters on ground right and left
        self.fov = 0

        # the UAV trajectory as line object
        self.traj = None

        # get the global pose origin from launch file
        self.origin = [0,0]
        launch = xml.parse(launch_file).getroot()
        for include in launch.iter('include'):
            args = include.findall('arg')
            found = False
            for arg in args:
                if arg.get('name') == 'id' and arg.get('value') == self.id:
                    found = True
                    break
            if found:
                for arg in args:
                    if arg.get('name') == 'x':
                        self.origin[0] = float(arg.attrib['value'])
                    if arg.get('name') == 'y':
                        self.origin[1] = float(arg.attrib['value'])
                break

    def info(self, ns, verbose=False):
        '''
        get infos about a bag file
        and store the results in class variables
        :param string ns: name space of topics
        :param bool verbose: whether to be verbose (default False)
        '''
        # open bag to read details
        bag = rosbag.Bag(self.folder + '/' + self.file)

        # read statistics
        self.bag_msg_count = bag.get_message_count()
        info = bag.get_type_and_topic_info()[1]
        self.bag_topics = dict([(topic, (info[topic].msg_type, info[topic].message_count)) for topic in info])

        # read begin and end times of coverage
        # assuming at most one goal and result message
        for topic,msg,t in bag.read_messages(ns + 'uav_coverage/goal'):
            self.tstart = t.secs + t.nsecs / 1000000000.0
            break
        for topic,msg,t in bag.read_messages(ns + 'uav_coverage/result'):
            self.tstop = t.secs + t.nsecs / 1000000000.0
            break

        # read target found and rescued times
        # considering only the first target
        for topic,msg,t in bag.read_messages(ns + 'target_found'):
            self.found = t.secs + t.nsecs / 1000000000.0 - self.tstart
            break
        for topic,msg,t in bag.read_messages(ns + 'bridge/events/mission_abort'): # target_rescued event not existent
            self.rescued = t.secs + t.nsecs / 1000000000.0 - self.tstart
            break

        t0 = 0.0;
        for topic,msg,t in bag.read_messages(ns + 'bridge/events/launch'):
            t0 = t.secs + t.nsecs / 1000000000.0
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'cmd/takeoff/goal'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'cmd/takeoff/result'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'bridge/events/mission_start'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'uav_coverage/goal'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'bridge/events/cps_selection'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'uav_coverage/result'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'cmd/assign_task/goal'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'target_found'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'cps_selected'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'cmd/assign_task/result'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'uav_tracking/goal'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'target_lost'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'uav_tracking/cancel'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'uav_tracking/result'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break
        for topic,msg,t in bag.read_messages(ns + 'bridge/events/mission_abort'):
            print("{0:.2f} {1}".format(t.secs + t.nsecs / 1000000000.0 - t0, topic))
            break

        # close bag
        bag.close()

        # print statistics about the current bag file contents
        if verbose:
            print
            print("\x1b[1;34m" + "{0}".format(self.file) + "\x1b[0m")
            print("No. Messages\tTopic (Message Type)")
            print("------------\t--------------------")
            for topic, info in self.bag_topics.items():
                print("{0:6d} msgs\t{1} ({2})".format(info[1], topic, info[0]))
            print("------------\n{0:6d} total".format(self.bag_msg_count))

    def parse(self, ns, res_space, verbose=False):
        '''
        parse the messages of a bag file
        and store the results in class variables
        :param string ns: name space of topics
        :param float res_space: spatial resolution in meter to filter the input data by (minimum distance between two consecutive coordinates)
        :param bool verbose: whether be verbose (default False)
        '''
        # open bag to read messages
        bag = rosbag.Bag(self.folder + '/' + self.file)

        # read goal coordinates
        goal_coords = [msg[1].pose.position for msg in bag.read_messages(ns + 'pos_controller/goal_position/local')]
        self.goal = [[coord.x + self.origin[0] for coord in goal_coords], [coord.y + self.origin[1] for coord in goal_coords]]

        # read actual pose coordinates
        for topic,msg,t in bag.read_messages(ns + 'mavros/local_position/pose'):
            if len(self.pose) == 0:
                self.pose.append((t.secs + t.nsecs / 1000000000.0, msg.pose.position.x + self.origin[0], msg.pose.position.y + self.origin[1], msg.pose.position.z))
            elif math.hypot(msg.pose.position.x + self.origin[0] - self.pose[-1][1], msg.pose.position.y + self.origin[1] - self.pose[-1][2]) > res_space:
                self.pose.append((t.secs + t.nsecs / 1000000000.0, msg.pose.position.x + self.origin[0], msg.pose.position.y + self.origin[1], msg.pose.position.z))

        # validate end time of coverage
        if self.tstop == 0.0 and len(self.pose) > 0:
            self.tstop = self.pose[-1][0]

        # trim poses according to begin and end times
        self.pose = [p for p in self.pose if self.tstart <= p[0] and p[0] <= self.tstop]

        if verbose:
            print("Obtained {0} goal coordinates".format(len(self.goal[0])))
            print("Obtained {0} pose coordinates".format(len(self.pose)))

        # close bag
        bag.close()

    def process(self, fov, verbose=False):
        '''
        process bag file contents to get data
        :param float fov: field of view of the UAVs in radian
        :param bool verbose: whether to be verbose (default False)
        '''
        # compute altitude
        alt = 0
        if len(self.pose) > 0:
            alts = [p[3] for p in self.pose]
            alt = sum(alts) / len(alts)

        # area visible to UAVs in each direction (inflation of line)
        self.fov = alt * math.tan(fov/2)

        if verbose:
            print("Tracking camera field of view:\n  fov = {0:.2f} m * tan({1:.2f}/2) = {2:.2f} m".format(alt, fov, self.fov))

        # create a line string that represents the trajectory of the UAV
        if len(self.pose) > 0:
            self.traj = geo.LineString(self.path(self.tstop))
        else:
            self.traj = geo.LineString()

        if verbose:
            #print("Average velocity:\n  v = {0:.2f}/{1:.2f} = {2:.2f}".format(self.traj.length, self.time[-1]-self.time[0], self.traj.length / (self.time[-1]-self.time[0])))
            if self.tstop != self.tstart:
                print("Average velocity:\n  v = {0:.2f} m / {1:.2f} s = {2:.2f} m/s".format(self.traj.length, self.tstop-self.tstart, self.traj.length / (self.tstop-self.tstart)))


    def path(self, tmax):
        '''
        generate a path of the UAV as list of poses
        :param float tmax: the maximum time up to which poses are included
        '''
        i = 0
        while i < len(self.pose) and self.pose[i][0] <= tmax:
            yield self.pose[i][1], self.pose[i][2]
            i += 1

        # make sure a path has at least length of two
        if len(self.pose) > 0:
            if i == 0:
                yield self.pose[0][1], self.pose[0][2]
                yield self.pose[0][1], self.pose[0][2]

            if i == 1:
                yield self.pose[0][1], self.pose[0][2]
