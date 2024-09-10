import os
import pandas as pd
import numpy as np
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, HoverTool, Legend, ColorBar, BasicTicker, LinearColorMapper
from bokeh.palettes import Viridis256
from bokeh.layouts import column
from bokeh.io import curdoc

class MonteCarloPlotter:
    def __init__(self, dataDir):
        self.dataDir = dataDir
        self.data = {}
        self.plots = {}
        self.debug_info = []

    def load_data(self, variables):
        self.debug_info.append(f"Loading data for variables: {variables}")
        for variable in variables:
            file_path = os.path.join(self.dataDir, f"{variable}.data")
            self.debug_info.append(f"Looking for file: {file_path}")
            if os.path.exists(file_path):
                self.debug_info.append(f"File found: {file_path}")
                df = pd.read_pickle(file_path)
                self.debug_info.append(f"DataFrame shape: {df.shape}")
                self.debug_info.append(f"DataFrame columns: {df.columns}")
                self.debug_info.append(f"DataFrame index: {df.index}")
                if not pd.api.types.is_datetime64_any_dtype(df.index):
                    df.index = pd.to_datetime(df.index, unit='ns')
                self.data[variable] = df
            else:
                self.debug_info.append(f"File not found: {file_path}")
        
        self.debug_info.append(f"Data loaded. Number of variables: {len(self.data)}")

    def create_bokeh_plot(self, df, title):
        self.debug_info.append(f"Creating plot for {title}")
        p = figure(title=title, x_axis_label='Time', y_axis_label='Value', width=800, height=400)
        
        for col in df.columns:
            source = ColumnDataSource(data=dict(x=df.index, y=df[col]))
            p.line('x', 'y', source=source, line_width=2, alpha=0.8)
        
        p.add_tools(HoverTool(tooltips=[("Time", "@x{%F %T}"), ("Value", "@y{0.00000}")],
                              formatters={"@x": "datetime"}))
        
        self.debug_info.append(f"Plot created for {title}")
        return p

    def create_datashader_plot(self, df, title, y_label, max_points=1000):
        plots = []
        component_map = {0: 'x', 1: 'y', 2: 'z'}
        num_runs = len(set(col[0] for col in df.columns))
        color_mapper = LinearColorMapper(palette=Viridis256, low=0, high=num_runs-1)
        time_seconds = (df.index - df.index[0]).total_seconds()

        for component in range(3):
            component_label = component_map[component]
            p = figure(title=f"{title} - {component_label.upper()} Component", x_axis_label='Time (s)', y_axis_label=f"{y_label} ({component_label})",
                       width=800, height=400, tools='pan,wheel_zoom,box_zoom,reset')

            for run in range(num_runs):
                col_name = (run, component)
                if col_name not in df.columns:
                    continue
                
                source = ColumnDataSource(data=dict(x=time_seconds, y=df[col_name], run_num=[run]*len(time_seconds)))
                p.line('x', 'y', source=source, color=color_mapper.palette[int(run * (len(color_mapper.palette) - 1) / (num_runs - 1))], 
                       line_alpha=0.9, line_width=3, legend_label=f"Run {run}")

            hover = HoverTool(
                tooltips=[
                    ('Time', '$x{0.000} s'),
                    (f'{y_label} ({component_label})', '$y{0.0000}'),
                    ('Run', '@run_num')
                ],
                mode='mouse'
            )
            p.add_tools(hover)

            color_bar = ColorBar(color_mapper=color_mapper, ticker=BasicTicker(desired_num_ticks=10),
                                 label_standoff=12, border_line_color=None, location=(0, 0))
            p.add_layout(color_bar, 'right')

            p.x_range.start = time_seconds.min()
            p.x_range.end = time_seconds.max()
            y_min = min(df[[(run, component) for run in range(num_runs)]].min().min() for component in range(3))
            y_max = max(df[[(run, component) for run in range(num_runs)]].max().max() for component in range(3))
            p.y_range.start = y_min
            p.y_range.end = y_max

            p.legend.click_policy = "hide"
            p.legend.location = "top_left"

            plots.append(p)

        layout = column(*plots)
        return layout

    def generate_plots(self):
        self.debug_info.append("Generating plots")
        for variable, df in self.data.items():
            self.plots[variable] = self.create_bokeh_plot(df, variable)
        self.debug_info.append(f"Generated {len(self.plots)} plots")

    def show_plots(self):
        self.debug_info.append("Showing plots")
        if self.plots:
            layout = column(*self.plots.values())
            curdoc().add_root(layout)
            self.debug_info.append("Added plots to document")
        else:
            self.debug_info.append("No plots were created")

    def get_debug_info(self):
        return self.debug_info
