#!/usr/bin/env bash
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

set -euo pipefail
metrics_directory="$1"
metrics_branch="${2:-usage-metrics}"
mkdir -p "${metrics_directory}"

# Only a successful lookup with no matching ref establishes a first run.
# Network/authentication failures must stop publication of shortened history.
remote_ref="$(git ls-remote --heads origin "refs/heads/${metrics_branch}")"
if [[ -z "${remote_ref}" ]]; then
    if git show-ref --verify --quiet "refs/remotes/origin/${metrics_branch}"; then
        echo "Metrics branch disappeared after checkout; refusing to discard history." >&2
        exit 1
    fi
    exit 0
fi

git fetch origin "${metrics_branch}:refs/remotes/origin/${metrics_branch}"
git show "origin/${metrics_branch}:metrics.csv" > "${metrics_directory}/metrics.csv"
git show "origin/${metrics_branch}:summary.json" > "${metrics_directory}/summary.json"
