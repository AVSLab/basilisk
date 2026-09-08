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

# Publish complete metric artifacts as one commit, without switching branches
# or touching the source checkout/index. All historical dates live in the CSV.
set -euo pipefail
metrics_directory="$1"
metrics_branch="${2:-usage-metrics}"
branch_ref="refs/heads/${metrics_branch}"
git check-ref-format "${branch_ref}"

if [[ ! -f "${metrics_directory}/.source-commit" ]]; then
    echo "Restore usage metrics successfully before publishing a snapshot." >&2
    exit 1
fi
expected_commit="$(< "${metrics_directory}/.source-commit")"

# Explicitly select the published files; never include the restore marker or
# files from the source branch. Missing artifacts stop before any remote write.
tree_entries=""
for filename in README.md metrics.csv summary.json usage.svg; do
    blob="$(git hash-object -w -- "${metrics_directory}/${filename}")"
    tree_entries+="$(printf '100644 blob %s\t%s' "${blob}" "${filename}")"$'\n'
done
empty_blob="$(git hash-object -w --stdin < /dev/null)"
tree_entries+="$(printf '100644 blob %s\t.nojekyll' "${empty_blob}")"$'\n'
snapshot_tree="$(printf '%s' "${tree_entries}" | git mktree)"

if [[ -n "${expected_commit}" ]]; then
    previous_tree="$(git rev-parse "${expected_commit}^{tree}")"
    parents="$(git show -s --format=%P "${expected_commit}")"
    if [[ "${snapshot_tree}" == "${previous_tree}" && -z "${parents}" ]]; then
        echo "Usage metrics did not change; retaining the existing single snapshot."
        exit 0
    fi
fi

# Omitting -p creates a parentless commit, including when migrating a branch
# with older daily commits. The lease protects concurrent updates/deletions,
# and its empty first-run value refuses to replace a newly created branch.
snapshot_commit="$(git commit-tree "${snapshot_tree}" -m "Update usage metrics")"
git push --force-with-lease="${branch_ref}:${expected_commit}" \
    origin "${snapshot_commit}:${branch_ref}"
