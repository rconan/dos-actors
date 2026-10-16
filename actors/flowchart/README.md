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

In `gmt_dos-actors`, the features are forwarded as `flowchart-graphviz` (default), `flowchart-elk` and `flowchart-replay`.

## Replay

With the `replay` feature (`flowchart-replay` in `gmt_dos-actors`), a model run is recorded and played back in the ELK flowchart.
The flowchart page is written as usual, then the actors record, at each sample step, that they have been updated and which outputs they have sent,
and, once the model has completed, the page is rewritten with the recorded steps.
In the page, `→` or space steps forward, `←` steps back, `Shift+→`/`Shift+←` jump to the next/previous step where the activity changes (e.g. a slower rate actor), a step number can be typed in the step box, `p` plays or pauses and `Esc` resets;
the actors updated at the current step are outlined and the wires carrying data are highlighted, the bootstrapped outputs are shown in a frame before the first step.
An actor inside a closed system is shown on the system block.

The replay requires the ELK backend (`FLOWCHART=elk` if the `graphviz` feature is enabled too) and only the first `FLOWCHART_STEPS` steps are recorded.

## Environment variables

| Variable | Description |
|---|---|
| `FLOWCHART` | `dot`, `neato` or `fdp`: Graphviz with that layout; `elk`: ELK; empty or `none`: no flowchart; unset: Graphviz `neato` if the `graphviz` feature is enabled, otherwise ELK |
| `FLOWCHART_THEME` | Graphviz theme: `screen` (dark, default) or `paper` (light) |
| `TO_DOT` | If set, the Graphviz dot files are written as well |
| `DATA_REPO` | Directory where the flowchart is written |
| `FLOWCHART_STEPS` | Number of sample steps recorded for the replay, 100 by default |

## Licence

This crate is MIT licensed.
The `elk` feature embeds the [elkjs](https://github.com/kieler/elkjs) 0.11.1 bundle (`src/elk/elk.bundled.js`)
into the flowchart pages; elkjs is Copyright (c) 2017 Kiel University and others
and is distributed under the [Eclipse Public License 2.0](https://www.eclipse.org/legal/epl-2.0/) (EPL-2.0).
