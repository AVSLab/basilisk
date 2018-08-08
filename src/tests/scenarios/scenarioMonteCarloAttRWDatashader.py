''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

#
#
# Purpose: Example of graphing via datashader
#


import inspect
import math
import os
import numpy as np
import shutil
import matplotlib.pyplot as plt
import datetime

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))
# @endcond

from Basilisk import __path__
bskPath = __path__[0]


import numpy as np
import pandas as pd
import holoviews as hv
import datashader as ds
from bokeh.plotting import show
import datashader.transfer_functions as tf
from datashader.utils import export_image
from holoviews.operation.datashader import datashade
from functools import partial
from datashader.colors import colormap_select, Greys9, viridis
from colorcet import fire
from bokeh.palettes import GnBu9
from matplotlib.cm import jet
# This import is for reaggregating data when zooming if that is ever pursued
from datashader.bokeh_ext import InteractiveImage
from itertools import izip, count
import matplotlib.pyplot as plt
from scipy.misc import imread
import matplotlib.cbook as cbook
from bokeh.plotting import figure, show, output_file
from bokeh.models import Range1d


# Here are the name of some messages that we want to retain or otherwise use
inertial3DConfigOutputDataName = "guidanceInertial3D"
attErrorConfigOutputDataName = "attErrorInertial3DMsg"
mrpControlConfigOutputDataName = "LrRequested"
rwMotorTorqueConfigOutputDataName = "rw_torque_Lr"
mrpControlConfigInputRWSpeedsName = "reactionwheel_output_states"
sNavObjectOutputTransName = "simple_trans_nav_output"
fswRWVoltageConfigVoltageOutMsgName = "rw_voltage_input"

# Create data retention names based on what data we want to save
rwMotorTorqueConfigOutputDataName_motorTorque = rwMotorTorqueConfigOutputDataName + ".motorTorque"
attErrorConfigOutputDataName_sigma_BR = attErrorConfigOutputDataName + ".sigma_BR"
attErrorConfigOutputDataName_omega_BR_B = attErrorConfigOutputDataName + ".omega_BR_B"
sNavObjectOutputTransName_r_BN_N = sNavObjectOutputTransName + ".r_BN_N"
mrpControlConfigInputRWSpeedsName_wheelSpeeds = mrpControlConfigInputRWSpeedsName + ".wheelSpeeds"
fswRWVoltageConfigVoltageOutMsgName_voltage = fswRWVoltageConfigVoltageOutMsgName + ".voltage"


# Set global dataframes to populate with data when executing callbacks
rwMotorTorqueConfigOutputDataName_motorTorque_dataFrame = pd.DataFrame()
attErrorConfigOutputDataName_sigma_BR_dataFrame = pd.DataFrame()
attErrorConfigOutputDataName_omega_BR_B_dataFrame = pd.DataFrame()
sNavObjectOutputTransName_r_BN_N_dataFrame = pd.DataFrame()
mrpControlConfigInputRWSpeedsName_wheelSpeeds_dataFrame = pd.DataFrame()
fswRWVoltageConfigVoltageOutMsgName_voltage_dataFrame = pd.DataFrame()


# Create a list of retained data names to loop through while graphing. The CSV files are named by this list of strings
# so you can use this list as a way to loop through all of the data and graph.
retainedDataList = [attErrorConfigOutputDataName_sigma_BR,
                    attErrorConfigOutputDataName_omega_BR_B,
                    mrpControlConfigInputRWSpeedsName_wheelSpeeds, fswRWVoltageConfigVoltageOutMsgName_voltage, rwMotorTorqueConfigOutputDataName_motorTorque]

# Global list of dataframes that are populated from executing callback
# The order of these dataframes MUST be the same order as the retainedDataList.
# The retainedDataList[0] is the string name that corresponds to globalDataFrames[0]. This way we can look through
# both lists using zip(). By changing these lists, you've basically completed everything for
# retaining and graphing data other then having lines such as:
# retentionPolicy.addMessageLog(rwMotorTorqueConfigOutputDataName, [("motorTorque", range(5))], samplingTime)
globalDataFrames = [attErrorConfigOutputDataName_sigma_BR_dataFrame, attErrorConfigOutputDataName_omega_BR_B_dataFrame,
                   mrpControlConfigInputRWSpeedsName_wheelSpeeds_dataFrame,
                   fswRWVoltageConfigVoltageOutMsgName_voltage_dataFrame, rwMotorTorqueConfigOutputDataName_motorTorque_dataFrame]

# List of y axis labels, same order as globalDataFrames etc.
yAxisLabelList = ["Attitude Error Sigma_br", "Attitude Error omage_br_b",
                  "Reaction Wheel Speeds", "Flight Software Voltage Out", "Reaction Wheel Motor Torque"]
mainDirectoryName = "data/"
subDirectories = ["/mc1_data/", "/mc1_assets/"]

rwOutName = ["rw_config_0_data", "rw_config_1_data", "rw_config_2_data"]


# This method is used to populate the dataframe for the retained data of a simulation.
# It is called once for each run of the simulation, overlapping the plots
# A small optimization if the user wants the data to take up a little less space, is to
# maintain a single timeDataFrame and have all of the graphs refer to this dataframe.
def plotSim(data, retentionPolicy):

    # Get the data from messages using the global data names
    global globalDataFrames

    for i, dataframe, index in izip(count(), globalDataFrames, retainedDataList):
        dataMessage = data["messages"][index]
        globalDataFrames[i] = updateDataframes(dataMessage, dataframe)

# Write directories based on monte carlo name and sub directory name
def writeDirectories():
    for subDirectoryName in subDirectories:
        path = mainDirectoryName + subDirectoryName
        systemWriteDirectories(path)

    output_file("data/mc1_assets/mc1_graphs.html")

# Helper function for writing directories
def systemWriteDirectories(path):
    try:
        os.makedirs(path)
    except OSError:
        print "Creating failed, may be a directory there already"

# This methods adds the next run of the monte carlo below the previous run, with the two runs separated
# by a row of NaN values.
def updateDataframes(data, dataframe):
    # Using pd.concat we can put the new run to the right of the current right run instead of below.
    # This is not how datashader needs the data, however it is an example of maniuplating data
    # into a different shape with pandas
    # result = pd.concat([df, dataframe], axis = 1, sort = False)

    df = pd.DataFrame(data, columns=None)

    # Create a dataframe full of NaN values to append below the previous run and above the next run to append
    nandf = pd.DataFrame([np.nan])
    df = df.append(nandf, ignore_index=True)
    result = df.append(dataframe, ignore_index=True)
    return result

# This method configures the path for each file, and saves the dataframe as a csv
# The path is updated after csv write to change the filename. monteCarloName and mainDirectoryName
# can also be global variables if wanted.
# There is no need for having the index in the csv file for our data,
# it would be a waste of space.
def saveDataframesToFile():

    print "beginning writing csv..", datetime.datetime.now()

    for dataframe, index in zip(globalDataFrames, retainedDataList):
        path = mainDirectoryName + subDirectories[0] + "/" + index + ".csv"
        dataframe.to_csv(path, encoding = 'utf-8', index = False)

    print "done writing csv..", datetime.datetime.now()


# This method is the driver method for graphing all of the data. It loops through the retained data list (strings)
# and graphs the corresponding csv file for each retained data
def graph(fromCSV):
    if fromCSV:
        for data, yAxisLabel in zip(retainedDataList, yAxisLabelList):
            configureGraph(data, [], yAxisLabel, fromCSV)
    else:
        for data, dataFrame, yAxisLabel in zip(retainedDataList, globalDataFrames, yAxisLabelList):
            configureGraph(data, dataFrame, yAxisLabel, fromCSV)

# This method reads data from the csv files, and converts them into dataframes. It currently plots
# the data via holoviews framework, and datashades the plot. It passes this plot to the bokeh front end
# where the datashaded plot is used as a basis for the plot in the html file. Since the plot is datashaded,
# the html file is small ~2mbs.

# In addition, this method illustrates how to save the datashaded plots as a stack of images, and then save
# the image as a file.

# If the user would rather use plot via bokeh graphing library follow this code:
# (will take a long time for montecarlos) but may be useful, and also color coordinates every run
# colsPerRun = len(df.columns) / NUMBER_OF_RUNS
# for column in xrange(0, df.shape[1], colsPerRun):
#     length = len(Spectral6)
#     color = Spectral6[column % length]
#     p.multi_line([timeDF.iloc[:,0], timeDF.iloc[:,0], timeDF.iloc[:,0]], [df.iloc[:,column], df.iloc[:,column + 1], df.iloc[:,column + 2]],
#     color=[color, color, color], alpha=[0.3, 0.3, 0.3], line_width=2)
#     output_file("data/mc1/"+data+".html")
#     save(p)
def configureGraph(dataName, dataFrame, yAxisLabel, fromCSV):
    # Read csv file and create a dataframe from it.
    # If the user doesn't want to write any data to disc, the user can not write any data
    # and instead just use the global dataframes to plot the data. However, writing to file
    # can be advantageous since you can toggle ONLY_GRAPH to skip all of the simulating and
    # solely graph the data.
    print "beginning graphing process", datetime.datetime.now()

    if fromCSV:
        df = pd.read_csv(
            "data/mc1_data/" + dataName + ".csv")
    else:
        df = dataFrame

    # Find the outliers of the graph to identify erroneous runs.
    # Must do this before concatanating the columns so we can correspond
    # an outlier to a specific run
    # findOutliers(df, data)

    # Few lines to help with create different color maps.
    background = "black"
    cm = partial(colormap_select, reverse=(background != "black"))
    grays2 = cm([(i, i, i) for i in np.linspace(0, 255, 99)])
    grays2 += ["red"]


    # Concat the columns so all of the columns are now in 2 column and have been concatanated
    # If you'd rather keep all of columns and plot one against another column multiple times do this:
    # This plots columns labeled 1,2,3 against column 0 (time) and combines them into a layout. then you
    # datashade that layout instead of datashading the curves (doing it this way
    # mean the columns won't be labeled as x,y):
    # curvesx = hv.Curve(df[['0', '1']])
    # curvesy = hv.Curve(df[['0', '2']])
    # curvesz = hv.Curve(df[['0', '3']])
    # layout = curvesx * curvesy * curvesz

    df = concat_columns(df)

    #
    # NOW PLOTTING VIA HOLOVIEWS, BOKEH, AND DATASHADER.
    # Create html file with graphs and axis information etc.
    #

    # Plot the columns (x,y)
    curves = hv.Curve(df)

    # Instantiate a renderer using bokeh's interface, and generating an html file
    renderer = hv.renderer('bokeh').instance(fig='html')

    # Pass a datashaded version of the layout to the get_plot function, to return a bokeh figure
    # called 'plot'. Then set the figure details such as title, dimensions, axis labels etc.
    # Then finally, show the plot in the browser.
    # passing a value for cmap within the datashade function call will change the color of the
    # plots in the html. See below for more examples of cmaps
    # plot = renderer.get_plot(datashade(curves, dynamic = False, cmap= cm(viridis)).opts(plot=dict(fig_size=1000, aspect='equal'))).state
    plot = renderer.get_plot(datashade(curves, dynamic = False).opts(plot=dict(fig_size=1000, aspect='equal'))).state

    plot.plot_width = 800

    #If you want to zoom in on the image, set it here.
    plot.x_range = Range1d(df.x.min(), df.x.max())
    plot.y_range = Range1d(df.y.min(), df.y.max())

    plot.plot_height = 500
    plot.title.text = dataName
    plot.xaxis.axis_label = "Time"
    plot.yaxis.axis_label = yAxisLabel
    show(plot)

    #
    # NOW PLOTTING VIA SOLELY DATASHADER AND SAVING THE GRAPHS AS PNGS.
    #

    # Create an empty list of imgs to be filled, and set the canvas to put the images on.
    # This can be useful for combining multiple images of graphs that have been aggregated and shaded accordingly
    # to the number of points on the curve that cross the pixel (ds.count)
    # imgs = []
    # imgs.append(img)
    # stacked = tf.stack(*imgs)
    # export_image(stacked, data)

    # Set range of x and y axis (zooming via code)
    x_range = df.x.min(), df.x.max()
    y_range = df.y.min(), df.y.max()

    # Set the width and height of the images dimensions
    height = 800
    width = 2 * height

    # Instantiate a canvas object to put the graphs on
    cvs = ds.Canvas(plot_height=height, plot_width=width, x_range=x_range, y_range=y_range)

    # Compute a reduction by pixel, mapping data to pixels as a line.
    # from columns x and y, and correspond the darkness of the color
    # to the number of of curves that cross the pixel
    # (darker means more curves go on that pixel).
    agg = cvs.line(df, 'x', 'y', ds.count())

    # How can be 'linear' 'log' or 'eq_hist'. Here is a collection of different calls
    # to create the same image with different color schemes
    # create_image(agg, ['green', 'yellow', 'red'], 'log', 'none', dataName+"_green_yellow_red")
    # create_image(agg, ['green', 'yellow', 'red'], 'log', 'black', dataName+"_green_yellow_red_blackbg")
    # create_image(agg, cm(Greys9, 0.25), "eq_hist", 'black', dataName+"greys9_blackbg")
    # create_image(agg, cm(fire, 0.2), 'log','black', dataName+"_fire_blackbg")
    # create_image(agg, ['aqua', 'lime', 'fuchsia', 'yellow'], 'log','black', dataName+"_aqua_lime_blackbg")
    # create_image(agg, GnBu9, 'log', 'black', dataName+"_gnu_blackbg")
    # create_image(agg, jet, 'log', 'black', dataName+"jet_blackbg")
    # create_image(agg, cm(viridis), "eq_hist", 'white', dataName+"_viridis")
    create_image(agg, 'default', 'eq_hist', 'white', dataName + "_default")

    print "done graphing...", datetime.datetime.now()

# Helper function to create an image based on agg, color, the function to determine depth
# background and name of the file to export.
def create_image(agg, color, how, background, name):
    if color != 'default':
        img = tf.shade(agg, cmap=color, how=how)
    else:
        img = tf.shade(agg, how=how)
    if background != 'none':
        img = tf.set_background(img, background)
    export_image(img, name)


# This method changes the shape of our dataframe from:
# 0 1 2 3
# 1 3 4 2
# NAN row
# 1 2 5 6
# # # # #
# To:
# x y
# 1 3
# NAN
# 1 4
# NAN ...
# In addition to changing the shape, the columns are now named x and y. This was done
# to simply the datashading process to simply aggregate an image from an x and y column,
# instead of aggregating an image from a time, x, y, z columns. However, this removes any
# value of the data. There is no separating which points refer to which columns of data and instead
# just combines them all.
def concat_columns(df, separator=np.NaN):
    x = df.columns[0]
    df = df.append({x:separator}, ignore_index=True)
    return pd.concat([df[[x,c]].rename(columns={x:'x', c:'y'}) for c in df.columns[1:]])

# If working within a jupyter notebook, or using a local server. We can use this example of
# method as an argument for the InteractiveImage method that allows the data to be
# re-aggregated when zooming in.
def image_callback(x_range, y_range, w, h):
    cvs = ds.Canvas(plot_width=w, plot_height=h, x_range=x_range, y_range=y_range)
    df = pd.read_csv("data/mc1_data/" + retainedDataList[1]+".csv")
    agg = cvs.line(df, '0', '1', ds.count())
    img = tf.shade(agg)
    tf.shade(agg, how='eq_hist')
    return tf.dynspread(img, threshold=1)

# This method finds the outliers of the dataframe to identify runs with rogue behavior.
def findOutliers(df, data):
    from numpy import mean, std
    from scipy import stats
    # print df.describe()
    col = 1
    print data, df.quantile(.1)
    print data, df.quantile(.99)
    # outliers = df[df[col] > df[col].mean() + 3 * df[col].std()]
    # print outliers

#
# Some driver methods for what the datashader should do based on pytest information.
# this is mostly just used from the main monte carlo file.
#
def writeDataSaveFilesGraph():
    writeDirectories()
    saveDataframesToFile()
    graph(True)

def graphCurrentData():
    graph(True)

def graphWithoutCSV():
    writeDirectories() #used for images..no data is written.
    graph(False)


# This method is given a datashader image object, and saves it as a png file under
# a directory called "image". This is a variant of the export_image function
# built into the datashader API to make a little simpler to use)
def export_image(img, filename, fmt=".png", _return=True):
    export_path = mainDirectoryName + subDirectories[1]
    if not os.path.exists(export_path):
        os.mkdir(export_path)

    img.to_pil().save(os.path.join(export_path, filename + fmt))
    return img if _return else None
