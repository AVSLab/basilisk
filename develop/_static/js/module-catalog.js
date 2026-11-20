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

/* Filter the build-generated table locally, including when opened as a file.
 * No remote service, fetched index, or second module registry is needed.
 */
(() => {
    "use strict";

    function initialize() {
        const catalog = document.querySelector(".bsk-module-catalog");
        if (!catalog) {
            return;
        }
        const form = catalog.querySelector("form");
        const search = form.elements.query;
        const category = form.elements.category;
        const language = form.elements.language;
        const table = catalog.querySelector("table");
        const count = catalog.querySelector(".bsk-catalog-count");
        const empty = catalog.querySelector(".bsk-catalog-empty");
        const pagination = catalog.querySelector(".bsk-catalog-pagination");
        const previous = pagination.querySelector(".bsk-catalog-previous");
        const next = pagination.querySelector(".bsk-catalog-next");
        const pageLabel = pagination.querySelector(".bsk-catalog-page");
        const rows = Array.from(table.querySelectorAll("tbody tr"), (row) => ({
            element: row,
            text: (row.textContent + " "
                + (row.querySelector(".bsk-catalog-search-text")?.dataset.text || ""))
                .replace(/\s+/g, " ").toLowerCase(),
            category: row.querySelector(".bsk-catalog-category").textContent.trim(),
            language: row.querySelector(".bsk-catalog-language").textContent.trim(),
        }));
        const pageSize = 20;
        let page = 0;

        function rememberFilters() {
            const url = new URL(window.location.href);
            const values = {
                "module-search": search.value,
                "module-category": category.value,
                "module-language": language.value,
                "module-page": page ? String(page + 1) : "",
            };
            Object.entries(values).forEach(([key, value]) => {
                if (value) {
                    url.searchParams.set(key, value);
                } else {
                    url.searchParams.delete(key);
                }
            });
            // A copied link and browser Back both retain the current results.
            // Do not create a history entry for each keystroke.
            if (url.href !== window.location.href) {
                try {
                    window.history.replaceState(window.history.state, "", url.href);
                } catch (error) {
                    // Some browsers restrict history updates for local files.
                    // Filtering must still work without URL persistence.
                    if (error.name !== "SecurityError") {
                        throw error;
                    }
                }
            }
        }

        function render(resetPage = true) {
            if (resetPage) {
                page = 0;
            }
            const words = search.value.trim().toLowerCase().split(/\s+/).filter(Boolean);
            const matches = rows.filter((row) => (
                (!category.value || row.category === category.value
                    || row.category.startsWith(category.value + " / "))
                && (!language.value || row.language === language.value)
                && words.every((word) => row.text.includes(word))
            ));
            const pageCount = Math.max(1, Math.ceil(matches.length / pageSize));
            page = Math.min(page, pageCount - 1);
            rows.forEach((row) => { row.element.hidden = true; });
            const start = page * pageSize;
            matches.slice(start, start + pageSize).forEach((row) => {
                row.element.hidden = false;
            });
            count.textContent = matches.length
                ? `Showing ${start + 1}\u2013${Math.min(start + pageSize, matches.length)} of ${matches.length} modules`
                : "0 matching modules";
            empty.hidden = matches.length !== 0;
            table.hidden = matches.length === 0;
            previous.disabled = page === 0;
            next.disabled = page >= pageCount - 1;
            pageLabel.textContent = `Page ${page + 1} of ${pageCount}`;
            pagination.hidden = pageCount === 1;
            rememberFilters();
        }

        form.addEventListener("submit", (event) => event.preventDefault());
        search.addEventListener("input", () => render());
        category.addEventListener("change", () => render());
        language.addEventListener("change", () => render());
        form.addEventListener("reset", (event) => {
            event.preventDefault();
            search.value = "";
            category.value = "";
            language.value = "";
            render();
            search.focus();
        });
        function changePage(direction) {
            page += direction;
            render(false);
            // Present the new results at the top, without moving the sidebar.
            const firstLink = table.querySelector("tbody tr:not([hidden]) a");
            firstLink?.focus({preventScroll: true});
            catalog.scrollIntoView({block: "start"});
        }
        previous.addEventListener("click", () => changePage(-1));
        next.addEventListener("click", () => changePage(1));
        count.setAttribute("role", "status");
        count.setAttribute("aria-live", "polite");
        count.setAttribute("aria-atomic", "true");
        function restoreFilters() {
            const params = new URL(window.location.href).searchParams;
            search.value = params.get("module-search") || "";
            category.value = params.get("module-category") || "";
            language.value = params.get("module-language") || "";
            const requestedPage = Number(params.get("module-page"));
            page = Number.isSafeInteger(requestedPage) && requestedPage > 0
                ? requestedPage - 1 : 0;
            render(false);
        }
        restoreFilters();
        form.hidden = false;
        window.addEventListener("pageshow", restoreFilters);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", initialize, {once: true});
    } else {
        initialize();
    }
})();
