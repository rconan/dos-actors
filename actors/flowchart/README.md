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

In `gmt_dos-actors`, the features are forwarded as `flowchart-graphviz` (default), `flowchart-elk` and `flowchart-replay`; `scope` is only used by the `flowchart-scope` command.

## Replay

With the `replay` feature (`flowchart-replay` in `gmt_dos-actors`), a model run is recorded and played back in the ELK flowchart.
The flowchart page is written as usual, then the actors record, at each sample step, that they have been updated and which outputs they have sent,
and, once the model has completed, the page is rewritten with the recorded steps.
In the page, `→` or space steps forward, `←` steps back, `Shift+→`/`Shift+←` jump to the next/previous step where the activity changes (e.g. a slower rate actor), a step number can be typed in the step box, `p` plays or pauses and `Esc` resets;
the actors updated at the current step are outlined and the wires carrying data are highlighted, the bootstrapped outputs are shown in a frame before the first step.

With the stages on (`s` or the **stages** button), `→` and `←` go through each step stage by stage:
an actor comes after the actors it receives data from in the same step,
and the data of a bootstrapped output, sent at the previous step, breaks the feedback loops.
At each stage, the current actors and the data they send are highlighted, the earlier stages of the step are faded,
and the bootstrapped data still to be sent again at that step is drawn dotted.
An actor inside a closed system is shown on the system block.

Each actor records its steps locally, as arithmetic progressions (`[start, period, count]`) that follow from its rate,
and hands them over when its loop ends, so the recording costs a few integer operations per step and its size does not
grow with the length of the run (a few kB for the GMT integrated models).
The page is rewritten once the outermost model has completed (a system runs its actors as a nested model).

The replay requires the ELK backend (`FLOWCHART=elk` if the `graphviz` feature is enabled too).
The whole run is recorded unless `FLOWCHART_STEPS` limits the number of recorded steps.

## Scope

With the `scope` feature, the `flowchart-scope` command embeds the signals saved in parquet files by the Arrow loggers
(`gmt_dos-clients_arrow`) into the replay page of the same run, which remains a single self-contained file:

```shell
cargo run --release -p gmt_dos-actors_flowchart --features scope --bin flowchart-scope -- \
    data/model_flowchart.html data/model_data_1.parquet data/model_data_1000.parquet
```

A parquet file is matched to the logger actor with the same name and a column to the input of that logger with the same name;
the k-th row of a column is the k-th data sent on that channel, the bootstrapped data first, so it is placed at the recorded sample step.
The command is run after the model, once the loggers have written their files.

In the page, the wires of the embedded signals are marked and clicking one opens a scope at the bottom of the page:
* a time series is plotted with [Apache ECharts](https://echarts.apache.org/) against the sample steps (or the time, see `--sampling-frequency`),
  the elements given in the `elements` box (e.g. `0-6,10`) in color and the others faded, or as their envelope (minimum and maximum)
  if they are too many, with a cursor at the current step of the replay; the mouse wheel zooms, dragging pans, the slider below the plot
  selects the time range, the tooltip gives the values of the highlighted elements, and clicking in the plot moves the replay to that step,
* an image (a signal with a square number of values, at least 32², or with a shape given with `--shape`) shows the last frame at or before
  the current step, the pixels equal to 0 or NaN masked, with a color scale per frame (default) or over the whole run, linear or logarithmic,
  and the value of the pixel under the mouse; with the pixel size of the image (`--pixel`), the scope shows the field of view,
  a scale bar and the offset of the pixel under the mouse from the image center.

ECharts (a custom build of about 550 kB, `src/elk/echarts/`) is inserted into the page only if time series are embedded.

The values are embedded as float32 and the size of the embedded signals is limited (64 MB by default):
above that limit, the time series are thinned and the images are downsampled by the same factor, and the page says so.

| Option | Description |
|---|---|
| `--signals NAME,FILE:NAME,...` | Signals to embed, all by default |
| `--max-size MB` | Size limit of the embedded signals, 64 MB by default, 0 for no limit |
| `--steps START..END` | Embeds the data of the steps `START` to `END` (excluded), `START..` to the end |
| `--shape NAME=WxH` | Image shape of a signal |
| `--sampling-frequency HZ` | Sampling frequency of the steps, to show the time of the steps in the page and the time axis of the scopes |
| `--pixel NAME=SIZE[UNIT]` | Pixel size of an image, e.g. `'Frame<Host>=3.93mas'`; mas and arcsec are shown in mas or arcsec whichever reads best |
| `--output OUT.html` | Output page, the input page is rewritten by default |

## Environment variables

| Variable | Description |
|---|---|
| `FLOWCHART` | `dot`, `neato` or `fdp`: Graphviz with that layout; `elk`: ELK; empty or `none`: no flowchart; unset: Graphviz `neato` if the `graphviz` feature is enabled, otherwise ELK |
| `FLOWCHART_THEME` | Graphviz theme: `screen` (dark, default) or `paper` (light) |
| `TO_DOT` | If set, the Graphviz dot files are written as well |
| `DATA_REPO` | Directory where the flowchart is written |
| `FLOWCHART_STEPS` | Number of sample steps recorded for the replay, the whole run by default |

## Licence

This crate is MIT licensed.
The `elk` feature embeds the [elkjs](https://github.com/kieler/elkjs) 0.11.1 bundle (`src/elk/elk.bundled.js`)
into the flowchart pages; elkjs is Copyright (c) 2017 Kiel University and others
and is distributed under the [Eclipse Public License 2.0](https://www.eclipse.org/legal/epl-2.0/) (EPL-2.0).

The `scope` feature embeds a custom build of [Apache ECharts](https://echarts.apache.org/) 5.6.0 (`src/elk/echarts/echarts.custom.min.js`)
into the replay pages with time series; Apache ECharts is Copyright 2017-2024 The Apache Software Foundation and is distributed under the
[Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0); it includes ZRender 5.6.1 (Copyright (c) 2017, Baidu Inc., BSD 3-Clause License)
and tslib 2.3.0 (Copyright (c) Microsoft Corporation, 0BSD License).
The build only includes the line chart, the grid, dataZoom, tooltip, legend and markLine components and the canvas renderer (`src/elk/echarts/entry.js`);
it is rebuilt with:

```shell
npm install --ignore-scripts echarts@5.6.0 esbuild@0.24.2
NODE_PATH=node_modules node_modules/.bin/esbuild src/elk/echarts/entry.js --bundle --minify --format=iife \
    --global-name=echarts --legal-comments=inline --target=es2018 --outfile=src/elk/echarts/echarts.custom.min.js
```

and the licence header at the top of `echarts.custom.min.js` is added back.
