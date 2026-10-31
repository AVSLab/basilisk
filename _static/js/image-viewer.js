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

/* Enhance only figures marked bsk-enlarge. Sphinx's full-size image links
 * remain usable without JavaScript or native dialog support.
 */
(() => {
    "use strict";

    function initialize() {
        const links = document.querySelectorAll(".bsk-enlarge a.image-reference");
        if (!links.length || typeof HTMLDialogElement === "undefined"
                || typeof HTMLDialogElement.prototype.showModal !== "function") {
            return;
        }

        // Native modal dialogs provide Escape handling and keep focus inside.
        const viewer = document.createElement("dialog");
        viewer.className = "bsk-image-viewer";
        const toolbar = document.createElement("div");
        toolbar.className = "bsk-image-viewer-toolbar";
        const closeButton = document.createElement("button");
        closeButton.type = "button";
        closeButton.textContent = "Close \u00d7";
        closeButton.setAttribute("aria-label", "Close enlarged image");
        closeButton.autofocus = true;
        toolbar.append(closeButton);
        const enlargedImage = document.createElement("img");
        viewer.append(toolbar, enlargedImage);
        document.body.append(viewer);
        let activeLink;

        closeButton.addEventListener("click", () => viewer.close());
        viewer.addEventListener("click", (event) => {
            const bounds = viewer.getBoundingClientRect();
            if (event.target === viewer && (event.clientX < bounds.left
                    || event.clientX > bounds.right || event.clientY < bounds.top
                    || event.clientY > bounds.bottom)) {
                viewer.close();
            }
        });
        viewer.addEventListener("close", () => {
            document.documentElement.classList.remove("bsk-image-viewer-open");
            activeLink?.focus({preventScroll: true});
        });

        links.forEach((link) => {
            const image = link.querySelector("img");
            if (!image) {
                return;
            }
            link.setAttribute("aria-haspopup", "dialog");
            link.setAttribute("aria-label", `Enlarge image: ${image.alt}`);
            link.addEventListener("click", (event) => {
                // Preserve standard modified-click/open-in-new-tab behavior.
                if (event.button !== 0 || event.ctrlKey || event.metaKey
                        || event.shiftKey || event.altKey) {
                    return;
                }
                event.preventDefault();
                activeLink = link;
                enlargedImage.src = link.href;
                enlargedImage.alt = image.alt;
                viewer.setAttribute("aria-label", image.alt || "Enlarged image");
                viewer.showModal();
                document.documentElement.classList.add("bsk-image-viewer-open");
            });
        });
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", initialize, {once: true});
    } else {
        initialize();
    }
})();
