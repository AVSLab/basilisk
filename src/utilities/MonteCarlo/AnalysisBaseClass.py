import os
import pandas as pd
import numpy as np
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, HoverTool, Select, ColorBar, BasicTicker, LinearColorMapper, TextInput, Button, Div, SaveTool
from bokeh.palettes import Viridis256
from bokeh.layouts import column, row, Spacer, Row
from bokeh.io import output_file, save
import re

class MonteCarloPlotter:
    def __init__(self, dataDir, save_plots=False, doc_dir=None):
        self.dataDir = dataDir
        self.data = {}
        self.current_plot = None
        self.save_as_static = False
        self.staticDir = "/plots/"
        self.num_runs = 0
        self.total_data_size = 0
        self.title_div = None
        self.status_indicator = None
        self.save_plots = save_plots
        self.plot_save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../docs/source/_images/Scenarios/"))
        self.doc_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../docs/source/_images/Scenarios/"))
        if self.save_plots:
            os.makedirs(self.plot_save_dir, exist_ok=True)
            os.makedirs(self.doc_dir, exist_ok=True)

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

                # Calculate number of runs and total data size
                self.num_runs = max(self.num_runs, len(set(col[0] for col in df.columns)))
                num_components = len(df.columns) // self.num_runs
                self.total_data_size += df.memory_usage(deep=True).sum() / (1024 * 1024 * 1024)  # Size in GB

    def create_status_indicator(self):
        status_text = Div(text="Plot Status:", styles={
            'font-size': '14px',
            'margin-right': '10px',
            'color': '#555'
        })

        self.status_indicator = Div(text="●", styles={
            'color': 'green',  # Start with green
            'font-size': '24px',
            'margin-right': '10px'
        })

        status_explanation = Div(text="(Green: Ready, Red: Loading/Updating)", styles={
            'font-size': '12px',
            'color': '#777'
        })

        return row(status_text, self.status_indicator, status_explanation,
                   align='center', styles={'margin-bottom': '10px'})

    def update_status(self, is_loading=False):  # Default to False (green/ready)
        if self.status_indicator:
            color = 'red' if is_loading else 'green'
            self.status_indicator.styles['color'] = color

    def create_plot(self, variable, component, run_numbers=None):
        self.update_status(is_loading=True)
        df = self.data[variable]
        num_runs = len(set(col[0] for col in df.columns))
        time_seconds = (df.index - df.index[0]).total_seconds()
        tools = 'pan,wheel_zoom,box_zoom,reset,save'
        p = figure(title=f"{variable} - component {component}",
                   x_axis_label='Time (s)', y_axis_label=f"{variable.split('.')[-1]} (component {component})",
                   width=800, height=400, tools=tools,
                   sizing_mode="fixed")

        # Determine the number of components
        num_components = len(df.columns) // num_runs

        # Find the index of the selected component
        component_index = int(component) - 1  # Subtract 1 for 0-based indexing

        # Ensure component_index is within bounds
        if component_index >= num_components:
            component_index = num_components - 1

        # Extract y values for the selected component across all runs
        y_values = df.iloc[:, component_index::num_components].values
        y_min, y_max = np.nanmin(y_values), np.nanmax(y_values)
        y_range = y_max - y_min
        p.y_range.start = y_min - 0.1*y_range
        p.y_range.end = y_max + 0.1*y_range

        xs = [time_seconds for _ in range(num_runs)]
        ys = [df.iloc[:, component_index + i*num_components].values for i in range(num_runs)]

        # Create color mapper
        color_mapper = LinearColorMapper(palette=Viridis256, low=1, high=num_runs)

        # Calculate opacity based on number of runs
        base_opacity = max(0.1, min(0.8, 1 - (num_runs / 1000)))

        # Plot background lines (all runs) with calculated opacity
        background_source = ColumnDataSource(data=dict(
            xs=xs, ys=ys, color=list(range(1, num_runs+1))
        ))
        p.multi_line(xs='xs', ys='ys', line_color={'field': 'color', 'transform': color_mapper},
                     line_alpha=base_opacity, line_width=2, source=background_source,
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
            renderers=[p.renderers[-1]]
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

        # Add event listeners for zooming and panning
        p.on_event('pan_end', self.handle_plot_event)
        p.on_event('zoom_end', self.handle_plot_event)

        # Customize the SaveTool
        save_tool = p.select(type=SaveTool)[0]
        save_tool.filename = f"{variable}_{component}_plot"

        self.update_status(is_loading=False)
        return p

    def update_plot(self, attr, old, new):
        self.update_status(is_loading=True)
        variable = self.variable_select.value
        component = self.component_select.value
        run_numbers = self.parse_run_numbers(self.run_input.value)
        new_plot = self.create_plot(variable, component, run_numbers)

        # Find the plot in the layout and replace it
        for i, child in enumerate(self.plot_column.children):
            if isinstance(child, Row) and len(child.children) == 3:  # The row containing the plot
                if isinstance(child.children[1], figure):  # The middle child is the plot
                    child.children[1] = new_plot  # Replace just the plot, not the entire row
                    break

        self.update_status(is_loading=False)

    def update_title(self):
        if self.title_div:
            self.title_div.text = f"<div style='text-align: center; width: 100%;'>Monte Carlo Visualization: {self.num_runs} runs, {self.total_data_size:.2f} GB total</div>"

    def parse_run_numbers(self, input_string):
        if not input_string.strip():
            return None
        # Split the input string by commas, spaces, or semicolons
        run_strings = re.split(r'[,;\s]+', input_string)
        try:
            # Convert to integers and subtract 1 (for 0-based indexing)
            return [int(run) - 1 for run in run_strings if run]
        except ValueError:
            return None

    def update_button_callback(self):
        self.update_plot(None, None, None)

    def show_plots(self):
        variables = list(self.data.keys())

        def update_component_options(attr, old, new):
            variable = self.variable_select.value
            num_components = len(self.data[variable].columns) // self.num_runs
            components = [str(i) for i in range(1, num_components + 1)]
            self.component_select.options = components
            self.component_select.value = components[0]  # Always start with the first component

        # Add title with centering style
        self.title_div = Div(text="Monte Carlo Visualization",
                             styles={'text-align': 'center', 'font-size': '24px', 'margin-bottom': '20px', 'color': '#555', 'width': '100%'})

        self.variable_select = Select(title="Variable", options=variables, value=variables[0], width=200)
        self.variable_select.on_change('value', update_component_options)

        # Initialize component options based on the first variable
        initial_variable = variables[0]
        initial_num_components = len(self.data[initial_variable].columns) // self.num_runs
        initial_components = [str(i) for i in range(1, initial_num_components + 1)]
        self.component_select = Select(title="Component", options=initial_components, value=initial_components[0], width=200)

        # Create input elements before using them in the layout
        self.run_input = TextInput(title="", width=300)
        self.search_button = Button(label="Search", button_type="primary", width=100)
        self.search_button.on_click(self.update_button_callback)

        # Create a container for the search bar and button, ensuring it's centered
        search_container = column(
            Div(text="Enter run numbers (comma, space, or semicolon separated):",
                styles={'text-align': 'center', 'margin-bottom': '10px', 'width': '100%'}),
            row(Spacer(width=20), self.run_input, Spacer(width=20), self.search_button, Spacer(width=20),
                sizing_mode="fixed", align="center"),
            align="center"
        )

        # Add a note about pressing Enter
        self.enter_note = Div(text="<i>Tip: You can also press Enter after entering run numbers to update the plot.</i>",
                              styles={'font-size': '12px', 'color': '#666666', 'margin': '5px 0', 'text-align': 'center', 'width': '100%'})

        initial_plot = self.create_plot(self.variable_select.value, self.component_select.value)

        # Save all initial plots if enabled
        self.save_all_initial_plots()

        self.variable_select.on_change('value', self.update_plot)
        self.component_select.on_change('value', self.update_plot)
        self.run_input.on_change('value', self.update_plot)

        # Create a centered layout with space for the close button
        self.plot_column = column(
            self.create_status_indicator(),
            self.title_div,
            row(Spacer(width=20), self.variable_select, Spacer(width=20),
                self.component_select, Spacer(width=20),
                sizing_mode="fixed", align="center"),
            Spacer(height=20),
            search_container,
            self.enter_note,
            Spacer(height=20),
            row(Spacer(width=20), initial_plot, Spacer(width=20),
                sizing_mode="fixed", align="center"),
            Spacer(height=20),  # Space for close button
            sizing_mode="stretch_width",
            align="center"
        )

        # Wrap the entire layout in a column for additional centering control
        centered_layout = column(
            self.plot_column,
            sizing_mode="stretch_both",
            align="center"
        )

        # Update the title after creating the layout
        self.update_title()

        return centered_layout

    def get_downsampled_plots(self):
        # This method is no longer needed, but we'll keep it for compatibility
        return self.data

    def handle_plot_event(self, event):
        self.update_status(is_loading=True)
        # The actual zooming/panning is handled by Bokeh
        # We just need to update the status when it's done
        self.update_status(is_loading=False)

    def save_plot(self, plot, variable, component):
        if not self.save_plots:
            return

        base_filename = f"{variable}_{component}"

        # Save as HTML
        html_filename = os.path.join(self.plot_save_dir, f"{base_filename}.html")
        output_file(html_filename)
        save(plot)

        # Generate RST content for documentation
        rst_content = self.generate_rst_content(variable, component, html_filename)

        # Save RST content
        rst_filename = os.path.join(self.doc_dir, f"MonteCarloPlots_{base_filename}.rst")
        with open(rst_filename, 'w') as f:
            f.write(rst_content)

    def generate_rst_content(self, variable, component, filename):
        title = f"{variable} - Component {component.upper()}"
        underline = "=" * len(title)
        content = f"""
{title}
{underline}

.. raw:: html
   :file: {filename}

This plot shows component {component.upper()} of the {variable} variable.
"""
        return content

    def save_all_initial_plots(self):
        if not self.save_plots:
            return

        for variable in self.data.keys():
            for component in ['1', '2', '3']:
                plot = self.create_plot(variable, component)
                self.save_plot(plot, variable, component)

    def update_plot_callback(self, attr, old, new):
        self.update_plot(None, None, None)
