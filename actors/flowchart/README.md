# gmt_dos-actors_flowchart

Flowcharts of [GMT DOS actors](https://docs.rs/gmt_dos-actors) models.

A model is turned into a `PlainModel`, a collection of `PlainActor`s free of generic types and constants,
from which a `Graph` is built and written to `<name>_flowchart.html`, in the current directory or in `DATA_REPO`.

## Backends

| Feature | Backend |
|---|---|
| `graphviz` (default) | The graph is laid out with [Graphviz](https://www.graphviz.org/) (`dot`, `neato` or `fdp` must be installed) and written as SVG diagrams, one per system |
| `elk` | The graph is embedded as JSON in the page and laid out in the browser with [elkjs](https://github.com/kieler/elkjs), no external program is needed |

With ELK, an actor image is embedded in the page as a base64 data URI and drawn as the block background
(blocks with an image are at least 120×80); the image path is resolved against the working directory,
then against `CARGO_MANIFEST_DIR`, and a missing image is skipped with a warning.

In `gmt_dos-actors`, the features are forwarded as `flowchart-graphviz` (default) and `flowchart-elk`.

## Environment variables

| Variable | Description |
|---|---|
| `FLOWCHART` | `dot`, `neato` or `fdp`: Graphviz with that layout; `elk`: ELK; empty or `none`: no flowchart; unset: Graphviz `neato` if the `graphviz` feature is enabled, otherwise ELK |
| `FLOWCHART_THEME` | Graphviz theme: `screen` (dark, default) or `paper` (light) |
| `TO_DOT` | If set, the Graphviz dot files are written as well |
| `DATA_REPO` | Directory where the flowchart is written |

## Licence

This crate is MIT licensed.
The `elk` feature embeds the [elkjs](https://github.com/kieler/elkjs) 0.11.1 bundle (`src/elk/elk.bundled.js`)
into the flowchart pages; elkjs is Copyright (c) 2017 Kiel University and others
and is distributed under the [Eclipse Public License 2.0](https://www.eclipse.org/legal/epl-2.0/) (EPL-2.0).
