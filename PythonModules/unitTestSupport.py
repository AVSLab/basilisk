#
#   Unit Test Support Script
#

import math

#
#   function to convert seconds to an integer nanoseconds value
#
def sec2nano(time):
    return int(time*1E9)

#   variable to convert nano-seconds to seconds
NANO2SEC = 1E-9

#   variable to convert degrees to radians
D2R = (math.pi/180.)

#
#   function to check if an array of values is the same as the truth values
#
def isArrayEqual(result, truth, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
        return 0

    for i in range(0,dim):
        if (math.fabs(result[i+1] - truth[i]) > accuracy):
            return 0    # return 0 to indicate the array's are not equal

    return 1            # return 1 to indicate the two array's are equal


#
#   function to check if an array of values are zero
#
def isArrayZero(result, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    if dim < 1:
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
        return 0

    for i in range(0,dim):
        if (math.fabs(result[i+1]) > accuracy):
            return 0    # return 0 to indicate the array's are not equal

    return 1            # return 1 to indicate the two array's are equal


#
#   function to check if a double equals a truth value
#
def isDoubleEqual(result, truth, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    if (math.fabs(result[1] - truth) > accuracy):
        return 0    # return 0 to indicate the doubles are not equal

    return 1        # return 1 to indicate the doubles are equal


