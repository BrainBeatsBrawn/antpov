#!/usr/bin/env python3

import numpy as np

USAGE = """
After running a track through the image aligner, it will be in the coordinates of the baseline space - from (0,0) at the top left, to (1,1) at the bottom right. This program takes in a new pair of top left and bottom right coordinates and converts the given track into coordinates between those two.
The output is printed directly to the terminal so it can be piped into a new csv file.

 Example usage:
    $ python to-world-coords.py [path to csv file in baseline space] [top left coord] [bottom right coord]
    $ python to-world-coords.py some/file/path.csv "51.2, 23.1" "-2,-4"
"""

import sys

if __name__ == "__main__":
    if len(sys.argv) < 4 or sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print(USAGE)
        exit()

    # Parse the width and height as floats.
    vecFromString = lambda s: np.asarray([float(n.strip()) for n in s.split(",")])
    top_left = vecFromString(sys.argv[2])
    bottom_right = vecFromString(sys.argv[3])

    diff = bottom_right-top_left

    # Open associated labels file
    labels_file = sys.argv[1]
    labels_file = labels_file.replace ("-warped.csv", "_labels.csv")
    # print ("Opening associated labels file: ".format(labels_file))

    # I'll pack the labels up into an unsigned integer with bits meaning: 0:bush, 1:cookie, 2:shadow, 3:visibility
    # I'm assuming exactly as many rows in the "_labels.csv" as in the "-warped.csv"
    counter = np.uint32(0)
    myflags = np.array([], dtype=np.uint32)
    with open(labels_file, 'r') as f:
        for line in f:
            if not line.startswith("#"):
                # want line, but everything from the 3rd col.
                labeldata = vecFromString (line)
                bush = np.uint32(labeldata[2])
                cookie = np.uint32(labeldata[3])
                shadow = np.uint32(labeldata[4])
                visibility = np.uint32(labeldata[5])
                #print ("Flags: {},{},{},{}".format(bush,cookie,shadow,visibility))
                myflags = np.append (myflags, np.uint32(bush + (2 * cookie) + (4 * shadow) + (8 * visibility)))
                counter = counter + 1

    counter = np.uint32(0)
    with open(sys.argv[1], 'r') as f:
        for line in f:
            # Preserve headings
            if line.startswith("#"):
                print(line.strip())
            else:
                # Scale the actual coordinates (which should be the first two)
                inbound_point = vecFromString(line)
                new_point = top_left + inbound_point * diff
                print(f"{new_point[0]},{new_point[1]},{myflags[counter]}")
                counter = counter + 1
