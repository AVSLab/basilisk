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
import requests
import bskLargeData

# define the print color codes
statusColor = '\033[92m'
endColor = '\033[0m'

# this statement is needed to enable Windows to print ANSI codes in the Terminal
# see https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-terminal-in-python/3332860#3332860
os.system("")

# Function to download a single file
def download_file(file_url, dest_folder):
    """Download a Basilisk example file"""
    if not os.path.exists(dest_folder):
        os.makedirs(dest_folder)

    local_filename = os.path.join(dest_folder, file_url.split("/")[-1])
    with requests.get(file_url, stream=True) as response:
        response.raise_for_status()
        with open(local_filename, "wb") as file:
            for chunk in response.iter_content(chunk_size=8192):
                file.write(chunk)
    print(f"Downloaded: {local_filename}")

# Function to process a folder or file via GitHub API
def process_github_folder(api_url, dest_folder):
    """Process the BSK GitHub examples folder recursively and download all files."""
    response = requests.get(api_url)
    if response.status_code != 200:
        print(f"Failed to fetch the URL: {api_url}")
        print(f"Response: {response.text}")
        return

    items = response.json()
    for item in items:
        if item["type"] == "file":
            print(f"Downloading file: {item['name']}")
            download_file(item["download_url"], dest_folder)
        elif item["type"] == "dir":
            print(f"Entering folder: {item['name']}")
            subfolder_api_url = item["url"]
            subfolder_dest = os.path.join(dest_folder, item["name"])
            process_github_folder(subfolder_api_url, subfolder_dest)

def main():
    """
    Script to download all GitHub Basilisk examples into the local folder called ``examples``.
    This script also downloads the large BSK data files by loading ``bskLargeData.py``.
    """
    # install the large BSK data files
    bskLargeData.main()

    # Display the task message
    print(f"{statusColor}Task: Downloading BSK examples folder{endColor}")

    # GitHub API URL for the target folder
    github_api_url = "https://api.github.com/repos/AVSLab/basilisk/contents/examples"
    destination_folder = "./examples"

    process_github_folder(github_api_url, destination_folder)

# Main execution
if __name__ == "__main__":
    main()
