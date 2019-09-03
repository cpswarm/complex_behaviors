#!/usr/bin/python


import sys
import numpy as np
import shapely.geometry as geo

from modules import bag


class Run:
    '''
    a class for storing the bag file contents of one (simulation) run with multiple UAVs
    '''

    def __init__(self, folder, bag_files, launch_file, area_poly):
        '''
        initialize class
        :param string folder: absolute path of the bag file directory
        :param string bag_files: name of the bag files for this run
        :param string launch_file: launch file that was launched to generate the bag files
        :param polygon area_poly: maximum coverable area
        '''
        # date and time of run
        self.id = bag_files[0].split('_')[4].split('.')[0]

        # list of bags
        self.bags = [bag.Bag(folder, bf, launch_file) for bf in bag_files]

        # number of UAVs in this run
        self.length = len(self.bags)

        # maximum coverable area
        self.area_poly = area_poly
        self.area_max = area_poly.area

        # time when the first UAV started
        self.tmin = .0

        # time when the last UAV stopped
        self.tmax = .0

        # first target found time
        self.found = 0.0

        # first target rescued time
        self.rescued = 0.0

        # average field of view of the UAVs
        self.fov = .0

    def process(self, name_space, res_space, fov, verbose=False):
        '''
        process all bags of this run
        :param string name_space: name space of topics
        :param float res_space: spatial resolution in meter to filter the input data by (minimum distance between two consecutive coordinates)
        :param float fov: field of view of the UAVs in radian
        :param bool verbose: whether to be verbose (default False)
        '''
        for i,b in enumerate(self.bags):
            if not verbose:
                sys.stdout.write("{0}... ".format(i+1))
                sys.stdout.flush()

            # get bag information
            b.info(name_space, verbose)

            # parse bag file
            b.parse(name_space, res_space, verbose)

            # process bag file data
            b.process(fov, verbose)

        # time when the first UAV started
        self.tmin = min([b.tstart for b in self.bags])

        # time when the last UAV stopped
        self.tmax = max([b.tstop for b in self.bags])

        # time when the first target was found
        self.found = min([b.found for b in self.bags])

        # time when the first target was rescued
        self.rescued = min([b.rescued for b in self.bags])

        # average field of view of the UAVs
        self.fov = np.mean([b.fov for b in self.bags])

    def area(self, t):
        '''
        calculate the percentage of area covered by all UAVs after a specific time
        :param float t: time up to which to calculate the covered area, values less than 0 will return the total covered area
        '''
        # total covered area
        if t < 0:
            return geo.MultiLineString([geo.LineString(b.path(b.tstop)) for b in self.bags]).buffer(self.fov).intersection(self.area_poly).area / self.area_max

        # no more area covered after UAV stopped
        elif t > self.tmax:
            return np.nan

        # calculate area
        else:
            return geo.MultiLineString([geo.LineString(b.path(t)) for b in self.bags]).buffer(self.fov).intersection(self.area_poly).area / self.area_max

    def time(self, p, tmin=-1, tmax=-1):
        '''
        calculate the time it took all UAVs to cover a specific area
        :param float p: percentage of covered area, 0 will return the starting time, values less than 0 will return the stopping time
        :param float tmin: lower bound of time (default self.tmin)
        :param float tmax: upper bound of time (default self.tmax)
        '''
        # default values
        if tmin < self.tmin:
            tmin = self.tmin
        if tmax < self.tmin:
            tmax = self.tmax

        # starting time
        if p == 0:
            return tmin

        # stopping time
        elif p < 0:
            return tmax

        # find time
        else:
            return self.regula_falsi(p, tmin, tmax)

    def regula_falsi(self, p, a, b, e=0.001, m=5):
        '''
        find time that corresponds approximately to the given coverage percentage
        :param float p: percentage to find time for
        :param float a: lower bound of time (default self.tmin)
        :param float b: upper bound of time (default self.tmax)
        :param float e: half of upper bound for error (default: 0.001)
        :param int m: maximum number of iterations (default: 5)
        '''
        # initialize areas of bounds
        area_a = self.area(a)
        area_b = self.area(b)

        # initialize sought time
        c = a

        # search time for maximum number if iterations
        for i in xrange(m):
            # sought time out of bounds
            if area_a == area_b:
                return np.nan

            # update estimate of time
            c = (b*area_a - a*area_b + a*p - b*p) / (area_a - area_b)
            area_c = self.area(c)

            # time close enough
            if np.abs(p - area_c) < e:
                break

            # update bounds
            if area_c < p:
                a = c
                area_a = area_c
            else:
                b = c
                area_b = area_c

        return c
