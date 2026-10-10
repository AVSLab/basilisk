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

/* Standard Sphinx tables need no author-supplied classes or RST containers.
 * JavaScript checks actual widths; CSS handles scrolling without scroll events.
 * Without ResizeObserver, retain the theme's ordinary scrollable tables.
 */
(() => {
    "use strict";

    function initialize() {
        if (typeof ResizeObserver === "undefined") {
            return;
        }
        const content = document.querySelector(".rst-content");
        if (!content) {
            return;
        }
        const entries = Array.from(content.querySelectorAll("table.docutils"))
            .filter((table) => table.tHead?.querySelector("th")
                && !table.matches(".field-list, .footnote, .citation")
                && !table.parentElement.closest("table, .sidebar"))
            .map((table) => ({table}));
        if (!entries.length) {
            return;
        }
        const observedWrappers = new Set();
        let pending = false;

        function update() {
            pending = false;
            // Theme scripts may wrap tables after window.load (notably the
            // catalog). Always use the current ancestors, not a startup snapshot.
            entries.forEach((entry) => {
                const wrappers = [];
                for (let parent = entry.table.parentElement; parent && parent !== content;
                        parent = parent.parentElement) {
                    if (parent.matches(".wy-table-responsive")) {
                        wrappers.push(parent);
                        if (!observedWrappers.has(parent)) {
                            observedWrappers.add(parent);
                            observer.observe(parent);
                        }
                    }
                }
                entry.wrappers = wrappers;
            });
            const wrappers = new Set(entries.flatMap((entry) => entry.wrappers));
            // Read geometry before changing classes. Account for both theme
            // wrappers when Sphinx and the theme supply nested scroll containers.
            const fits = new Map(entries.map(({table, wrappers: parents}) => {
                const available = Math.min(content.clientWidth,
                    ...parents.map((parent) => parent.clientWidth));
                const width = Math.max(table.scrollWidth, table.getBoundingClientRect().width);
                return [table, width > 0 && width <= available + 1];
            }));
            const enabledWrappers = new Map(Array.from(wrappers, (wrapper) => [wrapper,
                entries.filter((entry) => entry.wrappers.includes(wrapper))
                    .every((entry) => fits.get(entry.table))]));
            let anySticky = false;
            entries.forEach(({table, wrappers: parents}) => {
                const enabled = fits.get(table)
                    && parents.every((parent) => enabledWrappers.get(parent));
                table.classList.toggle("bsk-sticky-table", enabled);
                anySticky ||= enabled;
            });
            enabledWrappers.forEach((enabled, wrapper) => {
                wrapper.classList.toggle("bsk-sticky-table-wrap", enabled);
            });
            document.body.classList.toggle("bsk-has-sticky-tables", anySticky);
        }

        function schedule() {
            if (!pending) {
                pending = true;
                requestAnimationFrame(update);
            }
        }

        const observer = new ResizeObserver(schedule);
        observer.observe(content);
        entries.forEach(({table}) => observer.observe(table));
        const structureObserver = new MutationObserver((records) => {
            if (records.some((record) => Array.from(record.addedNodes).some((node) =>
                node.nodeType === Node.ELEMENT_NODE
                && (node.matches(".wy-table-responsive") || node.querySelector("table"))))) {
                schedule();
            }
        });
        structureObserver.observe(content, {childList: true, subtree: true});
        document.fonts?.ready.then(schedule);
        window.addEventListener("pageshow", schedule);
        schedule();
    }

    // Initial layout is ready; the observer also handles late theme wrappers.
    if (document.readyState === "complete") {
        initialize();
    } else {
        window.addEventListener("load", initialize, {once: true});
    }
})();
