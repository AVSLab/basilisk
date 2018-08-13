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
import os
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
from Basilisk.utilities import macros
import datashader as ds
import datashader.transfer_functions as tf
from datashader.utils import export_image
from holoviews.operation.datashader import datashade
from functools import partial
from datashader.colors import colormap_select, Greys9, viridis
from colorcet import fire
from bokeh.palettes import GnBu9
from matplotlib.cm import jet

# This import is for reaggregating data when zooming if that is ever pursued
# from datashader.bokeh_ext import InteractiveImage

from itertools import izip, count
from bokeh.plotting import show, output_file
from bokeh.models import Range1d

# This is only for the phantomjs framework
# from bokeh.io import export_png

mainDirectoryName = "data/"

retainedDataList = []

globalDataFrames = []

saveData = False

timeUnit = "seconds"
listOfGraphs = []
subDirectories = ["/mc1_data/", "/mc1_assets/"]
densityHow = ""
holoviewsImageResolution = 0

whichGraphingStyle = "holoviews_datashader"

class DatashaderGraph:
    def __init__(self, dataIndex, yaxislabel = "Y Value", xaxislabel = "Time", title = "", color = "default",
                 graphRanges = [(0, 0), (0, 0)], dpi = 300, dimension = (800, 400), macro = macros.NANO2SEC):
        self.dataIndex = dataIndex
        self.yaxislabel = yaxislabel
        self.xaxislabel = xaxislabel
        self.title = title
        self.ranges = graphRanges
        self.macro = macro
        # Set the width, height of the graph. A 2:1 ratio is suggested. This is width / height of
        # the pngs in pixel, or the width / height of the bokeh figures.
        self.dimension = dimension
        # datashader color. Can be a list of colors, or a predefined color scheme from the datashading interface:
        # options: jet, viridis, GnBu, greys, fire.
        # Or list of colors for least dense to most dense such as: ['green', 'yellow', 'red']
        self.color = color
        self.dpi = dpi


# interface for other sims. maybe have this be a list of tuples with correspond axis names with each name.
def configure(dataConfiguration, graphingTechnique = "holoviews_datashader",
              htmlName = "mc_graphs.html", directories = ["/mc1_data/", "/mc1_assets/"]):

    global listOfGraphs
    listOfGraphs = dataConfiguration

    for graph in dataConfiguration:
        retainedDataList.append(graph.dataIndex)
        globalDataFrames.append(pd.DataFrame())

    global subDirectories
    if directories != subDirectories:
        subDirectories = []
        for subdirectory in directories:
            subDirectories.append(subdirectory)

    global whichGraphingStyle
    whichGraphingStyle = graphingTechnique

    output_file("data" + subDirectories [1] + htmlName)

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

    for graph, data, dataframe in zip(listOfGraphs, retainedDataList, globalDataFrames):
        configureGraph(dataFrame = dataframe if not fromCSV else [], dataName = data,
                       graph = graph, fromCSV = fromCSV)

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
def configureGraph(dataName, fromCSV, dataFrame, graph):
    # Read csv file and create a dataframe from it.
    # If the user doesn't want to write any data to disc, the user can not write any data
    # and instead just use the global dataframes to plot the data. However, writing to file
    # can be advantageous since you can toggle ONLY_GRAPH to skip all of the simulating and
    # solely graph the data.
    print "beginning graphing process", datetime.datetime.now()

    if fromCSV:
        df = pd.read_csv(
            "data/" + subDirectories[0] + dataName + ".csv", index_col=False)
        df['0'] = df['0'] * graph.macro

    else:
        df = dataFrame
        df[0] = df[0] * graph.macro

    df = concat_columns(df)

    if whichGraphingStyle == "holoviews_datashader":
        holoviews_interface(dataName, df, graph.yaxislabel, graph.xaxislabel, graph.ranges, graph.color, graph.dimension, graph.title)
    elif whichGraphingStyle == "only_datashader":
        datashade_interface(dataName, df, graph.ranges, graph.color, graph.dimension)
    elif whichGraphingStyle == "both":
        holoviews_interface(dataName, df, graph.yaxislabel, graph.xaxislabel, graph.ranges, graph.color, graph.dimension, graph.title)
        datashade_interface(dataName, df, graph.ranges, graph.color, graph.dimension)

def getColorScheme(color):
    # Few lines to help with create different color maps.
    background = "black"
    cm = partial(colormap_select, reverse=(background != "black"))
    grays2 = cm([(i, i, i) for i in np.linspace(0, 255, 99)])
    grays2 += ["red"]

    if color == "jet":
        return jet
    elif color == "viridis":
        return cm(viridis)
    elif color == "GnBu":
        return GnBu9
    elif color == "greys":
        return cm(Greys9, 0.25)
    elif color == "fire":
        return cm(fire, 0.2)

def holoviews_interface(dataName, df, yAxisLabel, xAxisLabel, ranges, color, dimension, title):


    # Concat the columns so all of the columns are now in 2 column and have been concatanated
    # If you'd rather keep all of columns and plot one against another column multiple times do this:
    # This plots columns labeled 1,2,3 against column 0 (time) and combines them into a layout. then you
    # datashade that layout instead of datashading the curves (doing it this way
    # mean the columns won't be labeled as x,y):
    # curvesx = hv.Curve(df[['0', '1']])
    # curvesy = hv.Curve(df[['0', '2']])
    # curvesz = hv.Curve(df[['0', '3']])
    # layout = curvesx * curvesy * curvesz


    #
    # NOW PLOTTING VIA HOLOVIEWS, BOKEH, AND DATASHADER.
    # Create html file with graphs and axis information etc.
    #

    # Plot the columns (x,y)
    curves = hv.Curve(df)

    colorScheme = getColorScheme(color)
    # Instantiate a renderer using bokeh's interface, and generating an html file
    renderer = hv.renderer('bokeh').instance(fig='html')

    # Pass a datashaded version of the layout to the get_plot function, to return a bokeh figure
    # called 'plot'. Then set the figure details such as title, dimensions, axis labels etc.
    # Then finally, show the plot in the browser.
    # passing a value for cmap within the datashade function call will change the color of the
    # plots in the html. See below for more examples of cmaps
    plot = renderer.get_plot(
        datashade(curves, dynamic=False, cmap=colorScheme , width = dimension[0], height = dimension[1]).opts(plot=dict(aspect='equal', dpi = 4000))).state
    # plot = renderer.get_plot(datashade(curves, dynamic = False).opts(plot=dict(fig_size=1000, aspect='equal'))).state

    plot.plot_width = dimension[0]
    plot.plot_height = dimension[1]

    # If you want to zoom in on the image, set it here.

    plot.x_range = Range1d(df.x.min() if ranges[0][0] == 0 else ranges[0][0], df.x.max() if ranges[0][1] == 0 else \
    ranges[0][1])
    plot.y_range = Range1d(df.y.min() if ranges[1][0] == 0 else ranges[1][0], df.y.max() if ranges[1][1] == 0 else \
    ranges[1][1])


    plot.title.text = title
    plot.xaxis.axis_label = xAxisLabel
    plot.yaxis.axis_label = yAxisLabel
    show(plot)

    # Only use this if using the phantomjs framework. (not currently supported)
    # if saveFigures:
    #     export_png(plot, mainDirectoryName + subDirectories[1] + dataName+".png")

    print "done graphing...", datetime.datetime.now()

# Function that creates datashaded images and saves them as png.
def datashade_interface(dataName, df, ranges, color, dimension):
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
    x_range = df.x.min() if ranges[0][0] == 0 else ranges[0][0], df.x.max() if ranges[0][1] == 0 else ranges[0][1]
    y_range = df.y.min() if ranges[1][0] == 0 else ranges[1][0], df.y.max() if ranges[1][1] == 0 else ranges[1][1]

    # Set the width and height of the images dimensions
    height = dimension[1]
    width = dimension[0]

    # Instantiate a canvas object to put the graphs on
    cvs = ds.Canvas(plot_height=height, plot_width=width, x_range=x_range, y_range=y_range)

    # Compute a reduction by pixel, mapping data to pixels as a line.
    # from columns x and y, and correspond the darkness of the color
    # to the number of of curves that cross the pixel
    # (darker means more curves go on that pixel).
    agg = cvs.line(df, 'x', 'y', ds.count())

    # How can be 'linear' 'log' or 'eq_hist'. Here is a collection of different calls
    # to create the same image with different color schemes

    colorScheme = getColorScheme(color)
    create_image(agg, colorScheme, 'log', 'white', dataName)
# Helper function to create an image based on agg, color, the function to determine depth
# background and name of the file to export.
def create_image(agg, color, how, background, name):
    if color != None:
        img = tf.shade(agg, cmap=color, how=how)
    else:
        img = tf.shade(agg, how=how)

    if background != 'none':
        img = tf.set_background(img, background)

    export_image(img, name+"_datashaded")

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


def datashaderDriver(DATASHADER_FOUND):
    if DATASHADER_FOUND == False:
        print "datashader library not found"
        return

    print saveData
    print "showing graphs via datashader"
    if saveData:
        # Write the data to csv files, and then read from it and graph.
        writeDataSaveFilesGraph()
    else:
        # After the monte carlo is complete, parse the global dataframes for the data to plot
        # NOT the csv files. (to be used if user doesn't want to save data which can take some time)
        graphWithoutCSV()


# Customer method for saving into scenario folder, easier to separate.
def savePlotForDoxy(img, filename, fmt = ".png", _return=True):
    path = os.path.dirname(os.path.abspath(filename))
    export_path = path + "/../../../docs/Images/Scenarios/" + filename + fmt
    img.to_pil().save(os.path.join(export_path))
    return img if _return else None
# This method is given a datashader image object, and saves it as a png file under
# a directory called "image". This is a variant of the export_image function
# built into the datashader API to make a little simpler to use)
def export_image(img, filename, fmt=".png", _return=True):
    export_path = mainDirectoryName + subDirectories[1]
    if not os.path.exists(export_path):
        os.mkdir(export_path)

    img.to_pil().save(os.path.join(export_path, filename + fmt))
    return img if _return else None
