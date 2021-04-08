#!/usr/bin/python


import os
import sys


def select_run(log_path, prefix, suffix, multi):
    '''
    let the user select one simulation / experimental run
    :param string log_path: the path where to look for the bag files
    :param string prefix: prefix of the bag files
    :param string suffix: suffix of the bag files
    :param bool multi: whether multiple runs can be selected
    '''
    # invalid log path
    if not os.path.isdir(log_path):
        sys.stderr.write("Invalid input directory {0}\n".format(log_path))
        sys.exit(1)

    # get list of runs
    bags = [f.split('_')[-1][:-4] for f in os.listdir(log_path) if f.startswith(prefix) and f.endswith(suffix)]
    runs = sorted(set(bags))

    # no available runs
    if len(runs) == 0:
        sys.stderr.write("No sar bag files found in {0}\n".format(log_path))
        sys.stderr.write("The required file name format is {0}_uav_<i>_<date + time>{1}\n".format(prefix, suffix))
        sys.exit(1)

    # only one run available
    if len(runs) == 1:
        yield runs[0]

    # let the user choose the desired run
    else:
        for i in xrange(len(runs)):
            print("[{0:2d}] {1} ({2} CPS)".format(i+1, runs[i], bags.count(runs[i])))
        if multi:
            selected = [int(s) for s in raw_input("Select multiple runs (separate by space, 0 for all): ").split(' ')]
            # select all
            if selected == [0]:
                selected = [i+1 for i in xrange(len(runs))]
        else:
            selected = [int(raw_input("Select run: "))]

        # TODO: assert that all runs have same number of uavs

        # return all selected runs
        for s in selected:
            yield runs[s-1]


def get_bags(log_path, prefix, suffix, multi):
    '''
    get a list of bag files for the selected run
    :param string log_path: the path where to look for the bag files
    :param string prefix: prefix of the bag files
    :param string suffix: suffix of the bag files
    :param bool multi: whether multiple runs can be selected
    '''
    # select run
    runs = select_run(log_path, prefix, suffix, multi)

    # get list of selected bag files
    return [sorted([f for f in os.listdir(log_path) if f.find(run) > 0]) for run in runs]
