#!/usr/bin/python3


class Log:
    '''
    a class for storing and analyzing log file contents
    '''

    def __init__(self, folder, file):
        '''
        initialize class
        :param string folder: absolute path of the log file directory
        :param string file: name of the log file
        '''
        # absolute path of the log file directory
        self.folder = folder

        # name of the log file
        self.file = file

        # latest time stamp in log file
        self.t = 0

        # failure notices
        self.failures = []

        # read file
        with open(self.folder + self.file, 'r') as f:
            # iterate lines in reverse order
            for l in reversed(f.readlines()):
                # get latest time stamp
                if len(l.split('[')) > 2 and len(l.split('[')[2].split(',')) > 1:
                    t = float(l.split('[')[2].split(',')[1].strip(' ]'))
                    if self.t == 0:
                        self.t = t
                else:
                    t = 0

                # get failure notices
                if (l.split(':')[-1])[1:25] == "requesting home position":
                    self.failures.append(t)
                else:
                    break

    def localization_fails(self, time):
        '''
        get the number of failure notices after a given time
        :param float time: time after which the failures are counted
        '''
        for n,t in enumerate(self.failures):
            if t < time:
                return n

        return len(self.failures)
