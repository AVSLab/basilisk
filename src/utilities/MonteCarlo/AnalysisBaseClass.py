import os
import pandas as pd
import numpy as np
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, HoverTool, Select, ColorBar, BasicTicker, LinearColorMapper, TextInput, Button, Div
from bokeh.palettes import Viridis256
from bokeh.layouts import column, row, Spacer, layout
from bokeh.io import curdoc
import time
import re

class MonteCarloPlotter:
    def __init__(self, dataDir):
        self.dataDir = dataDir
        self.data = {}
        self.current_plot = None
        self.save_as_static = False
        self.staticDir = "/plots/"
        self.initial_interval = '1s'
        self.num_runs = 0
        self.total_data_size = 0
        self.title_div = None

    def load_data(self, variables):
        self.total_data_size = 0  # Reset total data size before loading
        for variable in variables:
            file_path = os.path.join(self.dataDir, f"{variable}.data")
            if os.path.exists(file_path):
                df = pd.read_pickle(file_path)
                if not pd.api.types.is_datetime64_any_dtype(df.index):
                    df.index = pd.to_datetime(df.index, unit='ns')
                self.data[variable] = df
                self.smallest_interval = pd.Timedelta(df.index.to_series().diff().min())
                print(f"Smallest interval in data: {self.smallest_interval}")
                
                # Calculate number of runs and total data size
                self.num_runs = max(self.num_runs, len(set(col[0] for col in df.columns)))
                self.total_data_size += df.memory_usage(deep=True).sum() / (1024 * 1024 * 1024)  # Size in GB

        print(f"Data loaded: {self.num_runs} runs, {self.total_data_size:.2f} GB total")

    def create_plot(self, variable, component, run_numbers=None):
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
        color_mapper = LinearColorMapper(palette=Viridis256, low=1, high=num_runs)
        
        # Plot background lines (all runs) with low alpha
        background_source = ColumnDataSource(data=dict(
            xs=xs, ys=ys, color=list(range(1, num_runs+1))
        ))
        p.multi_line(xs='xs', ys='ys', line_color={'field': 'color', 'transform': color_mapper},
                     line_alpha=0.5, line_width=3, source=background_source,
                     level='underlay')
        
        if run_numbers is not None:
            # Plot selected runs with full opacity
            xs_selected = [xs[i] for i in run_numbers]
            ys_selected = [ys[i] for i in run_numbers]
            colors_selected = [i+1 for i in run_numbers]
            source = ColumnDataSource(data=dict(xs=xs_selected, ys=ys_selected, color=colors_selected))
            p.multi_line(xs='xs', ys='ys', line_color={'field': 'color', 'transform': color_mapper},
                         line_alpha=1.0, line_width=3, source=source,
                         level='overlay')
        else:
            source = background_source

        # Add hover tool
        hover = HoverTool(
            tooltips=[
                ("Time", "$x{0.00} s"),
                (variable.split('.')[-1], "$y{0.0000}"),
                ("Run", "@color")
            ],
            mode='mouse',
            point_policy='snap_to_data',
            line_policy='nearest',
            renderers=[p.renderers[-1]]  # Apply to the last added renderer (selected runs or all runs)
        )
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
        run_numbers = self.parse_run_numbers(self.run_input.value)
        new_plot = self.create_plot(variable, component, run_numbers)
        self.plot_column.children[-1] = new_plot

    def update_title(self):
        if self.title_div:
            self.title_div.text = (f"Monte Carlo Visualization: {self.num_runs} runs, "
                                   f"{self.total_data_size:.2f} GB total")

    def parse_run_numbers(self, input_string):
        if not input_string.strip():
            return None
        # Split the input string by commas, spaces, or semicolons
        run_strings = re.split(r'[,;\s]+', input_string)
        try:
            # Convert to integers and subtract 1 (for 0-based indexing)
            return [int(run) - 1 for run in run_strings if run]
        except ValueError:
            print("Invalid input. Please enter run numbers separated by commas, spaces, or semicolons.")
            return None

    def update_button_callback(self):
        self.update_plot(None, None, None)

    def show_plots(self):
        variables = list(self.data.keys())
        components = ['x', 'y', 'z']

        # Add title
        self.title_div = Div(text="Monte Carlo Visualization",
                             styles={'text-align': 'center', 'font-size': '18px', 'margin-bottom': '10px', 'color': '#555'})

        self.variable_select = Select(title="Variable", options=variables, value=variables[0], width=200)
        self.component_select = Select(title="Component", options=components, value=components[0], width=200)
        
        # Create a container for the search bar and button
        search_container = Div(text="Enter run numbers (comma, space, or semicolon separated):", 
                               styles={'display': 'flex', 'flex-direction': 'column', 'margin-bottom': '10px'})
        
        self.run_input = TextInput(title="", width=300)  # Remove title from TextInput
        self.search_button = Button(label="Search", button_type="primary", width=100)
        self.search_button.on_click(self.update_button_callback)

        # Add a note about pressing Enter
        self.enter_note = Div(text="<i>Tip: You can also press Enter after entering run numbers to update the plot.</i>", 
                              styles={'font-size': '12px', 'color': '#666666', 'margin': '5px 0'})

        initial_plot = self.create_plot(self.variable_select.value, self.component_select.value)

        self.variable_select.on_change('value', self.update_plot)
        self.component_select.on_change('value', self.update_plot)
        self.run_input.on_change('value', self.update_plot)

        # Create a centered layout
        self.plot_column = column(
            self.title_div,
            row(self.variable_select, Spacer(width=20), self.component_select),
            Spacer(height=20),
            column(search_container,
                   row(self.run_input, Spacer(width=20), self.search_button)),
            self.enter_note,
            Spacer(height=20),
            initial_plot,
            width=900,  # Adjust this value based on your preferred overall width
            sizing_mode="stretch_width"
        )

        # Update the title after creating the layout
        self.update_title()

        # Return the layout instead of adding it to curdoc
        return self.plot_column

    def get_downsampled_plots(self):
        # This method is no longer needed, but we'll keep it for compatibility
        return self.data

