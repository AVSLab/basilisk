#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import os
import sys
import requests
import importlib.util
from tqdm import tqdm

# define the print color codes
statusColor = '\033[92m'
endColor = '\033[0m'

def download_file(url, destination_path):
    """Download a file from a URL with a progress bar."""
    try:
        print(f"Downloading file from {url}...")
        response = requests.get(url, stream=True, timeout=10)
        response.raise_for_status()  # Raise an error for bad HTTP responses

        # Get the total file size from headers
        total_size = int(response.headers.get("content-length", 0))

        os.makedirs(os.path.dirname(destination_path), exist_ok=True)  # Ensure destination folder exists

        # Download the file with tqdm progress bar
        with open(destination_path, "wb") as file, tqdm(
            desc=f"Downloading {os.path.basename(destination_path)}",
            total=total_size,
            unit="B",
            unit_scale=True,
            unit_divisor=1024,
        ) as bar:
            for chunk in response.iter_content(chunk_size=1024):
                file.write(chunk)
                bar.update(len(chunk))

        print(f"File downloaded and saved to {destination_path}.")
    except requests.exceptions.RequestException as e:
        print(f"Error downloading file: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)

def main():
    """
    Large BSK data files are downloaded directly from their web server
    and are then installed in the local Basilisk python package.
    For example, if using a python 3.11 virtual environment, the Spice ``*.bsp`` files
    will be stored in::

        .../.venv/lib/python3.11/site-packages/Basilisk/supportData/EphemerisData

    This function is useful if Basilisk is installed via a wheel which does not contain
    these large BSK data file to keep the wheel file size reasonable.  Calling this
    python file allows these files to be installed in an automated manner.

    If internet access is not available, these large BSK data files can be
    included in the above python package installation directly as well.

    """

    # Display the task message
    print(f"{statusColor}Task: Downloading large BSK data files{endColor}")

    # Step 1: Determine the file location of the Basilisk package
    try:
        spec = importlib.util.find_spec("Basilisk")
        if spec is None:
            print("Basilisk package not found. Ensure it is installed.")
            sys.exit(1)
        basilisk_path = os.path.dirname(spec.origin)
    except Exception as e:
        print(f"Error locating Basilisk package: {e}")
        sys.exit(1)

    # Step 2: Define the download URLs and destination paths
    files_to_download = {
        "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp": os.path.join(
            basilisk_path, "supportData", "EphemerisData", "de430.bsp"
        ),
        "https://naif.jpl.nasa.gov/pub/naif/HST/kernels/spk/hst_edited.bsp": os.path.join(
            basilisk_path, "supportData", "EphemerisData", "hst_edited.bsp"
        ),
        "https://naif.jpl.nasa.gov/pub/naif/pds/data/nh-j_p_ss-spice-6-v1.0/nhsp_1000/data/spk/nh_pred_od077.bsp": os.path.join(
            basilisk_path, "supportData", "EphemerisData", "nh_pred_od077.bsp"
        ),
    }

    # Step 3: Download each file
    for url, destination_path in files_to_download.items():
        download_file(url, destination_path)

    print("All files downloaded successfully.")

if __name__ == "__main__":
    main()
