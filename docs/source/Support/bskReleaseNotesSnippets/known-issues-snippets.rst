.. Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

- Known-issue updates now use individual RST snippets under ``docs/source/Support/bskKnownIssuesSnippets``, reducing conflicts between unrelated PRs. Release notes and known issues share a compiler that runs automatically during Sphinx builds; maintainers review and archive both collections before a tagged release.
- Full documentation builds reject nonempty snippet collections without an active include, preventing new entries from being omitted after release finalization. Update pending known-issue entries when their descriptions change, and use new snippets for follow-ups to published history.
- Known-issue entries cover issues affecting tagged releases. Issues introduced and fixed entirely within the same beta cycle need no known-issue entry if no tagged release was affected; release-note requirements apply separately.
