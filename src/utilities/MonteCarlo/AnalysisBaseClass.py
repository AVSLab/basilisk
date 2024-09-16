import os
import pandas as pd
import numpy as np
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, HoverTool, Select, ColorBar, BasicTicker, LinearColorMapper
from bokeh.palettes import Viridis256
from bokeh.layouts import column, row
from bokeh.io import curdoc
import time

class MonteCarloPlotter:
    def __init__(self, dataDir):
        self.dataDir = dataDir
        self.data = {}
        self.current_plot = None
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
                self.smallest_interval = pd.Timedelta(df.index.to_series().diff().min())
                print(f"Smallest interval in data: {self.smallest_interval}")

    def create_plot(self, variable, component):
        df = self.data[variable]
        num_runs = len(set(col[0] for col in df.columns))
        time_seconds = (df.index - df.index[0]).total_seconds()

        print(f"Creating plot for {variable} - {component} component")
        print(f"Number of runs: {num_runs}")
        print(f"Time range: {time_seconds.min()} to {time_seconds.max()} seconds")
        print(f"Number of data points per run: {len(df)}")

        p = figure(title=f"{variable} - {component.upper()} Component", 
                   x_axis_label='Time (s)', y_axis_label=f"{variable.split('.')[-1]} ({component})",
                   width=800, height=400, tools='pan,wheel_zoom,box_zoom,reset')

        component_index = ['x', 'y', 'z'].index(component)
        y_values = df.iloc[:, component_index::3].values
        y_min, y_max = np.nanmin(y_values), np.nanmax(y_values)
        y_range = y_max - y_min
        p.y_range.start = y_min - 0.1*y_range
        p.y_range.end = y_max + 0.1*y_range

        print(f"Y range: {y_min} to {y_max}")

        xs = [time_seconds for _ in range(num_runs)]
        ys = [df.iloc[:, component_index + i*3].values for i in range(num_runs)]
        
        # Create color mapper
        color_mapper = LinearColorMapper(palette=Viridis256, low=0, high=num_runs-1)
        
        source = ColumnDataSource(data=dict(xs=xs, ys=ys, color=list(range(num_runs))))
        
        p.multi_line(xs='xs', ys='ys', line_color={'field': 'color', 'transform': color_mapper},
                     line_alpha=0.1, line_width=1, source=source,
                     level='underlay', nonselection_alpha=0.1, selection_alpha=0.5)

        hover = HoverTool(tooltips=[("Time", "$x{0.00} s"), (variable.split('.')[-1], "$y{0.0000}"), ("Run", "$index")],
                          mode='mouse')
        p.add_tools(hover)

        # Add color bar
        color_bar = ColorBar(color_mapper=color_mapper,
                             label_standoff=12,
                             border_line_color=None,
                             location=(0,0),
                             title="Run Number",
                             ticker=BasicTicker(desired_num_ticks=10))
        p.add_layout(color_bar, 'right')

        return p

    def update_plot(self, attr, old, new):
        variable = self.variable_select.value
        component = self.component_select.value
        new_plot = self.create_plot(variable, component)
        self.plot_column.children[1] = new_plot

    def show_plots(self):
        variables = list(self.data.keys())
        components = ['x', 'y', 'z']

        self.variable_select = Select(title="Variable", options=variables, value=variables[0])
        self.component_select = Select(title="Component", options=components, value=components[0])

        initial_plot = self.create_plot(self.variable_select.value, self.component_select.value)

        self.variable_select.on_change('value', self.update_plot)
        self.component_select.on_change('value', self.update_plot)

        self.plot_column = column(row(self.variable_select, self.component_select), initial_plot)

        curdoc().add_root(self.plot_column)
        print("Plot should be visible now. If not, check the browser console for any errors.")

    def get_downsampled_plots(self):
        # This method is no longer needed, but we'll keep it for compatibility
        return self.data

