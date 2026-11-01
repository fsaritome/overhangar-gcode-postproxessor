# overhangar

G-code post-processing and pre-slicing planning for FDM printing (Bambu Studio / OrcaSlicer).

The longer-term plan is a planning layer in front of the slicer (mesh analysis, critical-face intent, per-region strategy: membranes, conical layers, arc overhangs). The first tool is `bridgefix`.

## bridgefix: shortest-span bridge post-processor (Bambu Studio / OrcaSlicer)

Bambu Studio uses one bridge direction per bridge island, or one fixed
`bridge_angle`. On ring- or L-shaped bridges, such as gasket-groove ceilings,
slots and frames, some lines then run **along** the gap instead of across it.
`bridgefix` rewrites every `; FEATURE: Bridge` block so that each part of the
bridge is filled across its shortest anchor-to-anchor span.

How it works:
1. Rebuilds the bridged area from the block's extrusion moves (arcs are converted to line segments).
2. For each cell of a 0.8 mm grid, finds the direction with the shortest chord.
   Where the slicer's own direction is nearly as short, it is kept.
3. If one direction is good enough for a whole island, that island stays one
   zone. Otherwise the island is split into zones, and tiny zones are merged
   into a neighbour.
4. Refills each zone with straight lines. Line spacing, flow (E/mm), speed, fan
   and other non-move commands come from the original block.
5. Fills leftover gaps, then restores position, Z and retraction state, so the
   rest of the file stays byte-identical.

A block is only rewritten if the longest span drops by at least 30 %
(`--force` overrides this).

### Install
```
pip install -r requirements.txt
```

### Use
```
python bridgefix/bridgefix.py model.gcode.3mf -o model.bridgefix.gcode.3mf --preview preview.png
python bridgefix/bridgefix.py model.gcode -o model.fixed.gcode
python bridgefix/bridgefix.py model.gcode            # rewrites the file in place
```
For a `.gcode.3mf`, the tool rewrites `Metadata/plate_N.gcode` and updates its
`.md5`. Send the result to the printer from Bambu Studio (open it, then Print),
or copy it to the SD card.

As a slicer post-processing script (Others → Post-processing scripts), use:
`"C:\path\to\python.exe" "C:\path\to\bridgefix.py";`. The slicer passes the
G-code path, and the script edits it in place.

Options: `--grid` (zone cell size in mm), `--angle-step` (direction resolution
in degrees), `--anchor` (extra extension of line ends into anchors, in mm),
`--internal` (also rewrite `Internal Bridge` blocks), `--force`.

### Limits and assumptions
- Assumes relative E (`M83`), the Bambu/Orca default, and that the slicer
  unretracts before each feature.
- Treats the edge of the bridged area as anchored. This holds for slicer bridge
  regions.
- Rewritten bridges are emitted as G1 moves (no arcs).
- Line ends run onto the anchor, so a bridge layer uses about 5–10 % more filament.
- The print-time estimate (M73) is not recalculated.
