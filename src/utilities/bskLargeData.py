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

import logging
import sys

import pooch
from tqdm import tqdm

from Basilisk.utilities.supportDataTools.dataFetcher import (
    POOCH,
    LOCAL_SUPPORT,
)
from Basilisk.utilities.supportDataTools.registrySnippet import REGISTRY

statusColor = "\033[92m"
endColor = "\033[0m"


def quiet_fetch(rel: str):
    """
    Silences Pooch logging for a single fetch call.
    """
    pooch_logger = pooch.utils.get_logger()
    old_level = pooch_logger.level
    pooch_logger.setLevel(logging.CRITICAL)

    try:
        return POOCH.fetch(rel)
    finally:
        pooch_logger.setLevel(old_level)


def main():
    print(f"{statusColor}Task: Pre-fetching Basilisk supportData files{endColor}")

    all_relpaths = sorted(REGISTRY.keys())
    failures = []
    successes = 0

    for rel in tqdm(all_relpaths, desc="Fetching data files", unit="file"):
        try:
            if LOCAL_SUPPORT:
                local = LOCAL_SUPPORT / rel
                if local.exists():
                    successes += 1
                    continue
            # Output progress
            tqdm.write(f"Downloading {rel}")
            quiet_fetch(rel)
            successes += 1

        except Exception as e:
            failures.append((rel, str(e)))

    print(f"\n{statusColor}Completed pre-fetch.{endColor}")
    print(f"Fetched {successes} / {len(all_relpaths)} files.")

    if failures:
        print("\nSome files could not be downloaded:")
        for rel, err in failures:
            print(f"    {rel}: {err}")
        sys.exit(1)

    print("All supportData files fetched successfully!")
    sys.exit(0)


if __name__ == "__main__":
    main()
