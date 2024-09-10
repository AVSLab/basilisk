import os
import pandas as pd
import numpy as np
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, HoverTool, ColorBar, BasicTicker, LinearColorMapper, Range1d, Tabs, TabPanel
from bokeh.palettes import Viridis256
from bokeh.layouts import column
from bokeh.io import curdoc

class MonteCarloPlotter:
    def __init__(self, dataDir):
        self.dataDir = dataDir
        self.data = {}
        self.plots = {}
        self.save_as_static = False
        self.staticDir = "/plots/"

    def load_data(self, variables):
        for variable in variables:
            file_path = os.path.join(self.dataDir, f"{variable}.data")
            if os.path.exists(file_path):
                df = pd.read_pickle(file_path)
                if not pd.api.types.is_datetime64_any_dtype(df.index):
                    df.index = pd.to_datetime(df.index, unit='ns')
                self.data[variable] = df

    def create_plot(self, df, title, y_label):
        plots = []
        component_map = {0: 'x', 1: 'y', 2: 'z'}
        num_runs = len(set(col[0] for col in df.columns))
        color_mapper = LinearColorMapper(palette=Viridis256, low=0, high=num_runs-1)
        time_seconds = (df.index - df.index[0]).total_seconds()

        for component in range(3):
            component_label = component_map[component]
            p = figure(title=f"{title} - {component_label.upper()} Component", 
                       x_axis_label='Time (s)', y_axis_label=f"{y_label} ({component_label})",
                       width=800, height=400, tools='pan,wheel_zoom,box_zoom,reset')

            # Create independent ranges for each plot
            p.x_range = Range1d(time_seconds.min(), time_seconds.max())
            y_values = df.iloc[:, component::3].values
            y_min, y_max = np.nanmin(y_values), np.nanmax(y_values)
            y_range = y_max - y_min
            p.y_range = Range1d(y_min - 0.1*y_range, y_max + 0.1*y_range)

            for run in range(num_runs):
                col_name = (run, component)
                if col_name in df.columns:
                    source = ColumnDataSource(data=dict(x=time_seconds, y=df[col_name], run_num=[run]*len(time_seconds)))
                    p.line('x', 'y', source=source, 
                           color=color_mapper.palette[int(run * (len(color_mapper.palette) - 1) / (num_runs - 1))], 
                           line_alpha=0.9, line_width=3)

            hover = HoverTool(tooltips=[('Time', '$x{0.000} s'),
                                        (f'{y_label} ({component_label})', '$y{0.0000}'),
                                        ('Run', '@run_num')],
                              mode='mouse')
            p.add_tools(hover)

            color_bar = ColorBar(color_mapper=color_mapper, ticker=BasicTicker(desired_num_ticks=10),
                                 label_standoff=12, border_line_color=None, location=(0, 0))
            p.add_layout(color_bar, 'right')

            plots.append(p)

        return plots

    def show_plots(self):
        if self.plots:
            tabs = []
            for variable, plots in self.plots.items():
                tab_panels = [TabPanel(child=plot, title=f"{component.upper()} Component") 
                              for plot, component in zip(plots, ['x', 'y', 'z'])]
                tabs.append(TabPanel(child=Tabs(tabs=tab_panels), title=variable))
            
            tabs_layout = Tabs(tabs=tabs)
            curdoc().add_root(tabs_layout)

    def get_downsampled_plots(self):
        downsampled_data = {key: df.groupby(df.index.floor('1s')).mean()
                            for key, df in self.data.items()
                            if not df.empty}
        
        for variable, df in downsampled_data.items():
            self.plots[variable] = self.create_plot(df, variable, variable.split('.')[-1])
        
        return downsampled_data
