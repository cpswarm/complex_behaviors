#!/usr/bin/python
# -*- coding: utf-8 -*-


import sys


def print_progress(iteration, total, prefix='', suffix='', decimals=1, length=50, fill='█'):
    '''
    print iterations progress
    call in a loop to create terminal progress bar
    :param int iteration: current iteration
    :param int total: total iterations
    :param str prefix: prefix (default '')
    :param str suffix: suffix (default '')
    :param int decimals: positive number of decimals in percent complete (default 1)
    :param int length: character length of bar (default 50)
    :param str fill: bar fill character (default '█')
    '''
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)

    sys.stdout.write("\r{0} |{1}| {2}% {3}".format(prefix, bar, percent, suffix))

    if iteration == total:
        sys.stdout.write('\n')

    sys.stdout.flush()
