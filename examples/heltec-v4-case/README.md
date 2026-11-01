# Heltec V4 outdoor case: MainBody

| File | What |
|---|---|
| `MainBody.3mf` | Model (Fusion 360 export), 76 × 52 × 14 mm frame |
| `HeltecV4CaseOutdoorV2.gcode.3mf` | Original slice: Bambu Studio 2.0.2, X1C, 0.4 nozzle, PLA, 0.16 mm layers, supports off, `bridge_angle = 90` |
| `HeltecV4CaseOutdoorV2.bridgefix.gcode.3mf` | Same file after `bridgefix` |

The interesting layer is **Z 1.16 (layer 8)**, the ceiling of the bottom gasket
groove. To check it without printing the whole part, start the print and cancel
it after layer 9, then look at the groove ceiling.
