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

/* Keep the Read the Docs theme's tree/ARIA behavior, but not its automatic
 * scroll-to-top. Load via html_js_files, before the theme starts navigation.
 */
(() => {
    "use strict";

    const root = new URL("../../", document.currentScript.src);
    const storageKey = `bsk-sidebar:${root.href}`;

    function initialize() {
        const navigation = window.SphinxRtdTheme?.Navigation;
        const sidebar = document.querySelector(".wy-side-scroll");
        const menu = sidebar?.querySelector(".wy-menu-vertical");
        if (!menu || typeof navigation?.reset !== "function"
                || typeof navigation.toggleCurrent !== "function") {
            return; // Leave navigation usable if a future theme changes its API.
        }

        function address(href) {
            const url = new URL(href, window.location.href);
            // Sphinx uses href="#" for the current page, unlike incoming links.
            if (!url.hash) {
                url.hash = "";
            }
            return url.href;
        }

        function offset(link) {
            return link.getBoundingClientRect().top - sidebar.getBoundingClientRect().top;
        }

        function restore(link, top) {
            // Track the clicked row rather than only scrollTop: another open
            // branch above it may have collapsed. Native scroll limits apply.
            sidebar.scrollTop += offset(link) - top;
        }

        function takeBookmark() {
            try {
                const saved = JSON.parse(window.sessionStorage.getItem(storageKey));
                window.sessionStorage.removeItem(storageKey);
                return saved && saved.destination === address(window.location.href)
                    && Number.isFinite(saved.top) ? saved : null;
            } catch (_) {
                // Browsers may disable storage, especially for local HTML files.
                return null;
            }
        }

        let bookmark = takeBookmark();
        const originalReset = navigation.reset;
        navigation.reset = function (...args) {
            const scrollTop = sidebar.scrollTop;
            const pageX = window.scrollX;
            const pageY = window.scrollY;
            // The theme still selects the active item and updates ancestors.
            // Undo its scrollIntoView synchronously, before the browser paints;
            // that call can otherwise move both the sidebar and the document.
            const result = originalReset.apply(this, args);
            window.scrollTo(pageX, pageY);
            sidebar.scrollTop = scrollTop;

            const saved = bookmark;
            bookmark = null;
            if (saved && saved.destination === address(window.location.href)) {
                try {
                    // In-page hash navigation consumes the same one-use bookmark.
                    window.sessionStorage.removeItem(storageKey);
                } catch (_) {
                    // Storage is optional; the in-memory bookmark is sufficient.
                }
                const link = Array.from(menu.querySelectorAll("a[href]"))
                    .find((item) => address(item.href) === saved.destination);
                if (link) {
                    restore(link, saved.top);
                    return result;
                }
            }

            // Direct/deep links have no saved row. Reveal only the portion
            // outside the sidebar, without scrolling the document itself.
            const active = menu.querySelector("a.current");
            if (active) {
                const bounds = sidebar.getBoundingClientRect();
                const item = active.getBoundingClientRect();
                if (item.top < bounds.top) {
                    sidebar.scrollTop += item.top - bounds.top;
                } else if (item.bottom > bounds.bottom) {
                    sidebar.scrollTop += item.bottom - bounds.bottom;
                }
            }
            return result;
        };

        const originalToggle = navigation.toggleCurrent;
        navigation.toggleCurrent = function (link, ...args) {
            const element = link[0]; // The theme supplies a jQuery link object.
            const top = offset(element);
            const result = originalToggle.call(this, link, ...args);
            restore(element, top);
            return result;
        };

        // Capture the row before the theme's delegated click handler changes
        // the tree. Do not intercept links or interfere with keyboard/modifier
        // behavior, external sites, downloads, or mobile-menu closing.
        document.addEventListener("click", (event) => {
            if (event.defaultPrevented || event.button !== 0 || event.ctrlKey
                    || event.metaKey || event.shiftKey || event.altKey) {
                return;
            }
            const link = event.target.closest("a[href]");
            if (!link || !menu.contains(link) || event.target.closest(".toctree-expand")
                    || link.hasAttribute("download") || (link.target && link.target !== "_self")) {
                return;
            }
            const destination = new URL(link.href);
            if (destination.origin !== root.origin || !destination.pathname.startsWith(root.pathname)) {
                return;
            }
            bookmark = {destination: address(link.href), top: offset(link)};
            try {
                // A one-use bookmark scoped to this documentation root and tab.
                // Nothing is sent to a server or shared across tabs/versions.
                window.sessionStorage.setItem(storageKey, JSON.stringify(bookmark));
            } catch (_) {
                // In-page expansion/anchor preservation still works without storage.
            }
        }, true);
    }

    // html_js_files precede theme.js. This listener runs before the theme's
    // jQuery-ready initialization, so even its first reset uses our wrapper.
    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", initialize, {once: true});
    } else {
        initialize();
    }
})();
