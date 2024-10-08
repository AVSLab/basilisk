import os
import logging
from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import MonteCarloPlotter
from bokeh.io import curdoc
from bokeh.layouts import column
from bokeh.models import Div

logging.basicConfig(level=logging.INFO)

def create_document(doc):
    logging.info("Starting the create_document function")
    
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenario_AttFeedbackMC")
    doc_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs")

    logging.info(f"Data directory: {data_dir}")
    logging.info(f"Doc directory: {doc_dir}")

    try:
        logging.info("Creating MonteCarloPlotter instance")
        plotter = MonteCarloPlotter(data_dir, doc_dir=doc_dir)
        
        logging.info("Loading data")
        plotter.load_data(['attGuidMsg.sigma_BR', 'attGuidMsg.omega_BR_B'])

        logging.info("Creating plot layout")
        layout = plotter.show_plots()

        doc.add_root(layout)
        logging.info("Added plot layout to document")

    except Exception as e:
        error_message = f"An error occurred: {str(e)}"
        logging.error(error_message)
        doc.add_root(Div(text=error_message))

# This is the entry point for Bokeh server
curdoc().add_root(Div(text="Loading..."))
curdoc().add_next_tick_callback(lambda: create_document(curdoc()))

print("Script executed successfully!")