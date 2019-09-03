#!/usr/bin/python3


import os

from modules import log


class CheckSimulation:
    '''
    a class for checking if a simulation is running properly
    '''

    def __init__(self, folder, timeout, max_fails):
        '''
        initialize class
        :param string folder: absolute path of the log file directory
        :param int timeout: maximum time in seconds that one robot is allowed to do nothing before it is assumed that it failed
        :param int max_fails: maximum number of allowed consecutive failures
        '''
        # absolute path of the log file directory
        self.folder = folder

        # maximum time in seconds that one robot is allowed to do nothing before it is assumed that it failed
        self.timeout = timeout

        # maximum number of allowed consecutive failures
        self.max_fails = max_fails

        # mavros log files
        self.mavros_logs = dict([(f.split('_')[1].split('-')[0], log.Log(folder, f)) for f in os.listdir(folder) if f.find("-mavros-") > 0])

        # coverage log files
        self.coverage_logs = dict([(f.split('_')[1].split('-')[0], log.Log(folder, f)) for f in os.listdir(folder) if f.find("uav_coverage") > 0])

    def failure(self):
        '''
        check if any robot stopped working
        '''
        # check if any robot did not produce any output at all
        for i,cl in self.coverage_logs.items():
            if cl.t == 0:
                return True

        # check if any robot did not produce any output within timeout
        tmax = max([cl.t for i,cl in self.coverage_logs.items()])
        for i,cl in self.coverage_logs.items():
            if cl.t < tmax - self.timeout:
                return True

        return False

    def localization_failure(self):
        '''
        check if the simulation failed due to localization issues
        '''
        for i,ml in self.mavros_logs.items():
            if ml.localization_fails(self.coverage_logs[i].t) >= self.max_fails:
                return True

        return False
