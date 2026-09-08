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
# A failed restore must never leave a previous run's publication authorization.
rm -f "${metrics_directory}/.source-commit"

# Only a successful lookup with no matching ref establishes a first run.
# Network/authentication failures must stop publication of shortened history.
remote_ref="$(git ls-remote --heads origin "refs/heads/${metrics_branch}")"
if [[ -z "${remote_ref}" ]]; then
    if git show-ref --verify --quiet "refs/remotes/origin/${metrics_branch}"; then
        echo "Metrics branch disappeared after checkout; refusing to discard history." >&2
        exit 1
    fi
    # An empty expected revision means publication may only create the branch.
    printf '\n' > "${metrics_directory}/.source-commit"
    exit 0
fi

# The branch is deliberately replaced with a parentless snapshot after each run.
git fetch origin "+refs/heads/${metrics_branch}:refs/remotes/origin/${metrics_branch}"
source_commit="$(git rev-parse "refs/remotes/origin/${metrics_branch}")"
git show "${source_commit}:metrics.csv" > "${metrics_directory}/metrics.csv"
git show "${source_commit}:summary.json" > "${metrics_directory}/summary.json"
# Pin the exact snapshot used above, even if another process later fetches refs.
printf '%s\n' "${source_commit}" > "${metrics_directory}/.source-commit"
