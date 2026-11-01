# sagtest: anchored bridge-strand experiment (experimental branch)

> **Experimental, AI-assisted G-code.** The plunge lanes press the hot nozzle up
> to **0.2 mm into printed plastic**. Check the preview and stay at the printer.
> Use at your own risk (see [NOTICE](../NOTICE)).

Question: do bridge strands sag less if they are **anchored**? Here that means:
at every post the nozzle retracts, presses 0.1–0.2 mm into the post top, waits,
then rises while extruding (heat stake). The test also tries stretching the
strand (flow below 1.0). The measurements also feed the sag simulation planned
on this branch.

## Test piece
- `sagtest_posts.stl`: 5 posts, 6 mm tall, on a base plate, with gaps of
  **5 / 10 / 20 / 30 mm**. Total size 91 × 77 mm. The base sticks out further at
  the **front: that's lane 1**.
- 12 lanes, 5 mm apart. Each lane is one strand along X crossing all gaps:

| Lane | Mode | Plunge depth | Dwell | Flow | Speed |
|---|---|---|---|---|---|
| 1, 12 | lay (normal bridge, baseline ×2) | – | – | 1.0 | 50 |
| 2 / 3 | lay, stretched | – | – | 0.85 / 0.7 | 50 |
| 4 | lay, slow | – | – | 1.0 | 20 |
| 5 | touch (dwell, no plunge) | – | 0.3 s | 0.85 | 50 |
| 6 / 7 | plunge | 0.1 / 0.2 | 0.3 s | 1.0 | 50 |
| 8 | plunge, long dwell | 0.2 | 0.8 s | 1.0 | 50 |
| 9 / 10 / 11 | plunge + stretched | 0.1 / 0.2 / 0.2 | 0.3 s | 0.85 / 0.85 / 0.7 | 50 |

Strands are round "thick bridge" strands (0.4 mm), with the part fan at 100 %.

## Steps
1. Slice `sagtest_posts.stl` in Bambu Studio: PLA, **supports off**, place it
   flat and **don't rotate** it, nothing else on the plate. Export the plate
   as `.gcode.3mf`.
2. Run:
   ```
   python sagtest/sagtest.py inject sliced.gcode.3mf -o sagtest.gcode.3mf --preview lanes.png
   ```
   The tool finds the posts in the top layer and refuses to continue if the
   model is rotated, the wrong model, or on a plate with other objects.
3. Open `sagtest.gcode.3mf` in Bambu Studio and check the preview: the
   strands appear above the last layer. Then print and **watch the strand
   phase**.
4. Measure each gap with calipers (outside jaws: one under the base, the
   other on the strand):
   **sag = height on the post − height at mid-gap**.
   Rate adhesion by pulling gently: 0 = loose … 3 = won't come off.
   Write it into `sagtest_lanes.csv` and send it back.

The safety checks in the code: Z never goes more than 0.25 mm below the post
tops, X and Y stay within the posts, extrusion steps are capped, and the
retraction state is restored before Bambu's end G-code.
