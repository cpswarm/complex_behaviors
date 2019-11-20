#!/usr/bin/python3

###############
#   IMPORTS   #
###############

import sys
import time
import subprocess as sp
import argparse

from modules import check_simulation


##############
#   PARAMS   #
##############

# parse command line arguments
parser = argparse.ArgumentParser(description="Run the search and rescue coverage simulation.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('dir', help="directory containing the ROS log files")
parser.add_argument('-f', '--fails', type=int, default=10, metavar='F', help="maximum number of failure notices read from the log files before terminating the run")
parser.add_argument('-l', '--launch_file', default='gazebo_2', metavar='L', help="name of the launch file from the sar package which is executed")
parser.add_argument('-m', '--max_time', type=int, default=3600, metavar='M', help="maximum time in seconds that one simulation run is allowed to take")
parser.add_argument('-r', '--runs', type=int, default=1, metavar='R', help="number of runs to perform")
parser.add_argument('-t', '--timeout', type=int, default=60, metavar='T', help="maximum time in seconds that one robot is allowed to do nothing before terminating the run")
args = parser.parse_args()


#################
#   EXECUTION   #
#################

# command to execute
cmd = ["stdbuf", "-o", "L", "roslaunch", "sar", args.launch_file + ".launch"]

# confirm details
print("\x1b[1;34m" + "Coverage simulation" + "\x1b[0m")
print("\x1b[1;34m" + "Running command: {0} ".format(' '.join(cmd) + "\x1b[0m"))
print("\x1b[1;34m" + "Execute {0} runs".format(args.runs) + "\x1b[0m")
print("\x1b[1;34m" + "Timeout {0}s".format(args.max_time) + "\x1b[0m")
if input("\x1b[1;34m" + "Start? [Y/n] " + "\x1b[0m") == 'n':
    sys.exit()

# repeat experiment for the given number of runs
for r in range(args.runs):
    print("")
    print("\x1b[1;34m" + "Starting run {0}".format(r+1) + "\x1b[0m")

    # execute command
    sim = sp.Popen(cmd)

    # starting time
    start = time.time()

    # wait 90s
    print("\x1b[1;34m" + "Waiting for 90s before monitoring progess" + "\x1b[0m")
    time.sleep(90)

    # while still running:
    while sim.poll() == None:
        # measure run time
        duration = time.time() - start
        print("\x1b[1;34m" + "Running {0}/{1} for {2:.0f}s (max {3}s)".format(r+1, args.runs, duration, args.max_time) + "\x1b[0m")

        # check if max time exceeded
        if time.time() - start > args.max_time:
            print("\x1b[1;34m" + "Reached maximum time, stopping simulation..." + "\x1b[0m")
            break

        # check log files for problems
        check = check_simulation.CheckSimulation(args.dir, args.timeout, args.fails)

        # check if all robots are working
        if check.failure():
            print("\x1b[1;34m" + "Not all robots are working, stopping simulation..." + "\x1b[0m")
            break

        # check if localization failed
        if check.localization_failure():
            print("\x1b[1;34m" + "Localization failed, stopping simulation..." + "\x1b[0m")
            break

        # wait 5s
        time.sleep(5)

    # stop execution
    sim.terminate()

    # wait 60s
    if r < args.runs - 1:
        print("\x1b[1;34m" + "Waiting for 60s before the next run" + "\x1b[0m")
        time.sleep(60)

print("\x1b[1;34m" + "Completed {0} runs".format(r+1) + "\x1b[0m")
