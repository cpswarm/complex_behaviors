#!/usr/bin/python
# -*- coding: utf-8 -*-

###############
#   IMPORTS   #
###############

import os
import sys
import argparse
import math
import numpy as np
import scipy.stats as st
import matplotlib.pyplot as plt
import shapely.geometry as geo
import descartes as des
import yaml
import rospkg

from modules import bag_selection
from modules import run
from modules import progress_bar


def main():
    ##############
    #   PARAMS   #
    ##############

    # parse command line arguments
    parser = argparse.ArgumentParser(description="Analyze the ROS bag files of the search and rescue coverage in terms of area covered over time.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('input', help="folder containing the bag files")
    parser.add_argument('-a', '--aggregate', action='store_true', help="aggregate the data of multiple runs")
    parser.add_argument('--conf', type=float, default=0.9, metavar='C', help="confidence for plotting error bars (only in combination with -a)")
    parser.add_argument('-d', '--display', action='store_true', help="display figures visualizing the results")
    parser.add_argument('--fov', type=float, default=0.64, metavar='F', help="the field of view of the UAVs in radian")
    parser.add_argument('-l', '--launch_file', default='gazebo', metavar='L', help="prefix of the launch file name which specifies the starting coordinates of the N UAVs (format <L>_<N>.launch)")
    parser.add_argument('--name_space', default='', metavar='N', help="name space of the messages in the bag files (e.g. /drone1/), required for hardware bag files")
    parser.add_argument('-o', '--output', action='store_true', help="output the results in a text file in the PWD")
    parser.add_argument('--percentages', type=list, default=[0.5, 0.75, 0.875, 0.9375, 0.96875, 1], metavar='P', help="the percentages of covered area to calculate the time for (only in combination with -t)")
    parser.add_argument('--prefix', default='sar', metavar='P', help="prefix of the bag files")
    parser.add_argument('-r', '--res_time', type=int, default=1000, metavar='R', help="the time resolution of the output data in seconds (not in combination with -t)")
    parser.add_argument('-s', '--res_space', type=float, default=0.1, metavar='S', help="the spatial resolution in meter to filter the input data by (minimum distance between two consecutive coordinates)")
    parser.add_argument('--sar_package', default='sar', metavar='P', help="package that contains the launch file which specifies the starting coordinates of the UAVs")
    parser.add_argument('--suffix', default='.bag', metavar='U', help="suffix of the bag files")
    parser.add_argument('-t', '--times', action='store_true', help="output result are times to reach certain percentages of covered area as opposed to the progress of covered area over time")
    parser.add_argument('-v', '--verbose', action='store_true', help="be verbose")
    parser.add_argument('--yaml_package', default='area_provider', metavar='P', help="package that contains the yaml param file which specifies the coordinates of the environment")
    parser.add_argument('--yaml_file', default='area_provider', metavar='Y', help="name of the yaml param file which specifies the coordinates of the environment")
    args = parser.parse_args()


    ####################
    #   PROCESS BAGS   #
    ####################

    # get a list of bag files for selected run(s)
    bag_files = bag_selection.get_bags(args.input, args.prefix, args.suffix, args.aggregate)

    if args.verbose:
        print("Selected bag files:")
        for bs in bag_files:
            for b in bs:
                print("  {0}".format(b))
            if len(bs) > 1:
                print

    # read size of area
    rospack = rospkg.RosPack()
    with open(rospack.get_path(args.yaml_package) +'/param/' + args.yaml_file + '.yaml', 'r') as stream:
        try:
            params = yaml.load(stream)
            area_max = geo.Polygon(geo.LinearRing(zip(params['area_x'], params['area_y'])))
            if args.verbose:
                print("Total coverable area:\n  {0} m²".format(area_max.area))
        except yaml.YAMLError as exc:
            print(exc)
            sys.exit(1)

    # data of each run
    launch_file = rospack.get_path(args.sar_package) + '/launch/' + args.launch_file + '_'
    runs = [run.Run(args.input, bf, launch_file + str(len(bf)) + '.launch', area_max) for bf in bag_files]

    # process every run
    for i,r in enumerate(runs):
        if not args.verbose:
            sys.stdout.write("Process {0} bags of run {1}/{2}: ".format(r.length, i+1, len(runs)))
            sys.stdout.flush()

        r.process(args.name_space, args.res_space, args.fov, args.verbose) # TODO: parallelize

        if not args.verbose:
            print("ok")


    ####################
    #   PREPARE DATA   #
    ####################

    if args.display or args.output:
        # processed data of each run
        # [x values, y values for each run, id of each run]
        data = [[], [], []]

        # time taken to cover specific percentages of area
        if args.times:
            if args.verbose:
                print

            # x axis
            data[0] = args.percentages

            # y axis
            for i,r in enumerate(runs):
                progress_bar.print_progress(0, len(args.percentages), prefix="Prepare run {0}/{1}:".format(i+1, len(runs)))
                times = []
                for j,p in enumerate(args.percentages):
                    # lower bound of time
                    if len(times) > 0:
                        tmin = times[-1]
                    else:
                        tmin = 0
                    # find time
                    times.append(r.time(p, tmin)) # TODO: parallelize
                    progress_bar.print_progress(j+1, len(args.percentages), prefix="Prepare run {0}/{1}:".format(i+1, len(runs)))
                # store data
                data[1].append(times)
                data[2].append(r.id)

        # percentage of area covered over time
        else:
            # x axis
            tmin = math.floor(min([r.tmin for r in runs]))
            tmax = math.ceil(max([r.tmax for r in runs]))
            data[0] = np.arange(tmin, tmax, args.res_time)

            # y axis
            for i,r in enumerate(runs):
                area_final = r.area(-1)
                if args.verbose:
                    print
                    print("Run {0} covered:\n  {1:.1f} %".format(i+1, area_final * 100))
                progress_bar.print_progress(0, len(data[0])*(len(data[0])+1)/2, prefix="Prepare run {0}/{1}:".format(i+1, len(runs)))
                areas = []
                for j,t in enumerate(data[0]):
                    areas.append(r.area(t)) # TODO: parallelize
                    progress_bar.print_progress((j+1)*(j+2)/2, len(data[0])*(len(data[0])+1)/2, prefix="Prepare run {0}/{1}:".format(i+1, len(runs)))
                data[1].append(areas)
                data[2].append(r.id)


    ##############
    #   OUTPUT   #
    ##############

    # display plots
    if args.display:
        # create plots
        fig = plt.figure()
        if len(data[1]) > 1:
            ax2 = fig.add_subplot(111)
            ax2.set_title('Progress')
            if args.times:
                ax2.set_xlabel('Covered Area')
                ax2.set_ylabel('Time')
            else:
                ax2.set_xlabel('Time')
                ax2.set_ylabel('Covered Area')
        # plot trajectories only for a single run
        else:
            ax1 = fig.add_subplot(211)
            ax1.set_title('Area')
            ax1.set_xlabel('X')
            ax1.set_ylabel('Y')
            ax2 = fig.add_subplot(212)
            ax2.set_title('Progress')
            if args.times:
                ax2.set_xlabel('Covered Area')
                ax2.set_ylabel('Time')
            else:
                ax2.set_xlabel('Time')
                ax2.set_ylabel('Covered Area')
            plt.subplots_adjust(hspace=0.5)

            # plot trajectories in space
            for i,b in enumerate(runs[0].bags):
                # path
                x,y = b.traj.xy
                ax1.plot(x, y, lw=1, color='C'+str(i%10), label="Pose "+b.id)
                ax1.plot(b.goal[:][0], b.goal[:][1], lw=1, color='C'+str(i%10), linestyle='--', label="Goal "+b.id)
                ax1.scatter(b.goal[:][0], b.goal[:][1], s=5, c='C'+str(i%10))
                # covered area
                a = b.traj.buffer(runs[0].fov)
                ax1.add_patch(des.PolygonPatch(a, fc='C'+str(i%10), ec='C'+str(i%10), alpha=0.5, label="Area "+b.id))
            ax1.legend()

        # plot data
        for d in data[1]:
            ax2.plot(data[0], d)

        # show plots
        plt.show()

    # output data to file
    if args.output:
        # output file name
        file_name = runs[0].bags[0].file.split('_')[-1][:-4] + '.txt'

        # check if file is existing and can be overwritten
        exists = os.path.isfile(file_name)
        if exists:
            overwrite = raw_input("\n{0} already exists, overwrite? [Y/n] ".format(file_name)) != 'n'

        # write file
        if not exists or overwrite:
            # aggregate data and compute statistics
            if len(data[1]) > 1:
                mean = np.nanmean(data[1], axis=0)
                var = np.nanvar(data[1], axis=0)
                df = len(data[1]) - 1 # degrees of freedom
                alpha = 1 - args.conf
                t = st.t.ppf(1 - alpha/2, df) # upper critical point for t-distribution
                ci = t * np.sqrt(var / len(data[1])) # confidence-interval half-length
                error = ci / abs(mean)

            # write data
            with open(file_name, 'w') as f:
                # write header
                if len(data[1]) > 1:
                    if args.times:
                        f.write("area")
                    else:
                        f.write("time")
                    for x in data[0]:
                        f.write(" {0}".format(x))
                else:
                    if args.times:
                        f.write("area time")
                    else:
                        f.write("time area")
                f.write("\n")

                # write data
                if len(data[1]) > 1:
                    # all data
                    for i,d in enumerate(data[1]):
                        f.write(data[2][i])
                        for y in d:
                            f.write(" {0}".format(y))
                        f.write("\n")
                    # statistics
                    f.write("mean")
                    for y in mean:
                        f.write(" {0}".format(y))
                    f.write("\n")
                    f.write("{0}%ci".format(args.conf*100))
                    for y in ci:
                        f.write(" {0}".format(y))
                    f.write("\n")
                    f.write("error")
                    for y in error:
                        f.write(" {0}".format(y))
                    f.write("\n")

                else:
                    for i,x in enumerate(data[0]):
                        f.write("{0} {1}\n".format(x, data[1][0][i]))


if __name__ == '__main__':
    main()
