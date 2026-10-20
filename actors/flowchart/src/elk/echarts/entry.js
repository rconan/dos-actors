// Custom Apache ECharts 5.6.0 build for the time series scopes of the flowchart replay (feature `scope`).
// Rebuild with (in a directory where `npm install echarts@5.6.0 esbuild@0.24.2` was run, see README.md):
//   NODE_PATH=<dir>/node_modules <dir>/node_modules/.bin/esbuild entry.js --bundle --minify --format=iife \
//     --global-name=echarts --legal-comments=inline --target=es2018 --outfile=echarts.custom.min.js
import * as echarts from 'echarts/core';
import { LineChart } from 'echarts/charts';
import {
  GridComponent,
  DataZoomComponent,
  TooltipComponent,
  LegendComponent,
  MarkLineComponent,
} from 'echarts/components';
import { CanvasRenderer } from 'echarts/renderers';

echarts.use([LineChart, GridComponent, DataZoomComponent, TooltipComponent, LegendComponent, MarkLineComponent, CanvasRenderer]);

export const { init, use, getInstanceByDom, dispose, version } = echarts;
