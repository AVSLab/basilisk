/*
 * ISC License
 * Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* Sphinx's html_search_scorer interface. Keep its standard term/object weights;
 * adjust only already-matched results. No pages are removed from the index.
 */
var Scorer = {
    objNameMatch: 11,
    objPartialMatch: 6,
    objPrio: {0: 15, 1: 5, 2: -5},
    objPrioDefault: 0,
    title: 15,
    partialTitle: 7,
    term: 5,
    partialTerm: 2,

    score(result) {
        // The documented tuple has six fields; newer Sphinx adds a result kind.
        const [docname, title, anchor, description, originalScore] = result;
        const path = docname.toLowerCase();
        const basename = path.split("/").pop();
        // Standard Sphinx search submits its query through the URL's q parameter.
        const query = (new URLSearchParams(window.location.search).get("q") || "")
            .trim().toLowerCase();
        const plainTitle = title.replace(/<[^>]*>/g, "").trim();
        const moduleTitle = plainTitle.match(/^(?:C(?:\+\+)?|Python|Rust) Module:\s*(\S+)$/i);
        const isTest = /(^|\/)(?:_unittest|tests)(\/|$)/.test(path)
            || basename.startsWith("test_");
        const asksForTests = /\b(?:tests?|testing|validation|pytest)\b|\btest_/.test(query);
        let score = originalScore;

        if (isTest && !asksForTests && query !== basename) {
            score -= 40;
        }
        if (basename === "index") {
            score -= 15;
        }
        if (/^(?:learn|build|extensions|vizard|support|examples)\//.test(path)) {
            score += 8;
        }
        if (moduleTitle && !isTest) {
            score += 12;
            if (moduleTitle[1].toLowerCase() === query && basename === query) {
                score += 100;
            }
        } else if (query && basename !== "index" && plainTitle.toLowerCase() === query) {
            // Preserve exact lookups of API symbols, scenarios, helpers, and tests.
            score += 80;
        }
        return score;
    },
};
