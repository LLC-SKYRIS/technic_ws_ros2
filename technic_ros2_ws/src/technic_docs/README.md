# technic_docs

The Technic documentation from `technic_ws` (`roswww_static/src/docs`, GitBook
Markdown) for the ROS 2 web UI. `technic_web` serves it at
`http://<drone>:8000/technic/docs/` (`docs:=false` in `web.launch.py` turns it off).

There is no GitBook build: `docs/<lang>/index.html` renders the `.md` files in
the browser with `docs/_viewer` (marked.js 4.3.0, vendored, so it works without
Internet). Pages are `docs/ru/?p=aruco.md#anchor`. The viewer supports what the
GitBook plugins did: `SUMMARY.md` contents, search, `> **Note**` boxes
(richquotes), `{#id}` heading anchors, `img.zoom`, the RU/EN switch.

To update the texts, copy `ru/`, `en/` and `assets/` from
`technic_ws/src/technic/roswww_static/src/docs` again. Keep `ru/index.html` and
`en/index.html`: they are the viewer pages.

Licenses: documentation CC BY-NC-SA 4.0 (`docs/LICENSE`), marked.js MIT,
viewer MIT.
