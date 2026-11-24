# Findings (generated numbers; interpretation in STUDY.md)

Accuracy on the dense block (all joints, RMS / max deg): a_old 3.22 / 17.91, b_new_stops 1.73 / 16.36, c_lut 1.38 / 10.66, b_sac 1.72 / 16.15
N10-extra held out (124 poses): a_old 4.22 / 28.88, b_new_stops 3.84 / 28.97, c_lut 3.93 / 35.98, b_sac 3.78 / 29.03
N10-extra with the maps fitted on N07-short (same calibration run): a_old 4.22 / 28.88, b_new_stops 3.61 / 29.71, c_lut 3.41 / 34.87
Robust (all joints, median / p95 |error| deg): dense: a_old 1.64/7.27, b_new_stops 0.55/3.68, c_lut 0.48/3.05, b_sac 0.56/3.71; V0: a_old 2.18/8.42, b_new_stops 1.06/6.66, c_lut 0.99/5.54, b_sac 1.06/6.34; V5: a_old 1.87/7.69, b_new_stops 1.16/5.91, c_lut 1.9/6.2, b_sac 1.14/5.88; extra: a_old 2.42/8.86, b_new_stops 2.17/7.94, c_lut 2.24/7.91, b_sac 2.12/7.85
N02-V0: a_old 3.9, b_new_stops 2.96, c_lut 2.49, b_sac 2.93
N03-V1: a_old 3.95, b_new_stops 3.06, c_lut 2.58, b_sac 3.03
N04-V2: a_old 3.92, b_new_stops 3.02, c_lut 2.52, b_sac 3.0
N05-V3: a_old 4.05, b_new_stops 3.23, c_lut 2.79, b_sac 3.21
N06-V4: a_old 3.71, b_new_stops 2.96, c_lut 3.39, b_sac 2.93
N08-V5: a_old 3.64, b_new_stops 2.88, c_lut 3.38, b_sac 2.85
N09-V6: a_old 3.79, b_new_stops 2.95, c_lut 3.22, b_sac 2.94, closed_loop_vs_wanted 3.43
N05-V3-pre: a_old 4.25, b_new_stops 3.42, c_lut 2.8, b_sac 3.39
N05-V3-post: a_old 3.15, b_new_stops 2.35, c_lut 2.78, b_sac 2.33

Hard stops changed: thumb_cmc [-45.0, 33.0]→[-46.0, 33.0], thumb_abd [-18.0, 55.0]→[-17.1, 59.7], thumb_mcp [-25.0, 100.0]→[-21.5, 102.1], thumb_dip [-15.0, 107.0]→[-11.2, 108.2], index_abd [-25.0, 30.0]→[-21.3, 27.0], index_mcp [-25.0, 100.0]→[-35.9, 103.8], index_pip [-15.0, 107.0]→[-18.8, 113.9], middle_mcp [-25.0, 100.0]→[-27.8, 103.8], ring_abd [-27.0, 27.0]→[-28.5, 26.6], ring_mcp [-25.0, 100.0]→[-22.4, 102.1], ring_pip [-15.0, 107.0]→[-16.3, 108.3], pinky_abd [-30.0, 30.0]→[-34.1, 29.6], pinky_mcp [-25.0, 100.0]→[-19.9, 106.1], pinky_pip [-15.0, 107.0]→[-14.3, 103.3]

| finger | map | distance mm | touch | force thumb | force finger |
|---|---|---|---|---|---|
| index | lut | 0.26 | yes | 0.224 | 0.0 |
| index | new | 0.76 | yes | 0.141 | 0.0 |
| index | old | 0.09 | yes | 0.374 | 0.141 |
| index | sac | 0.67 | yes | 0.3 | 0.0 |
| middle | lut | 0.23 | yes | 0.361 | 0.242 |
| middle | new | 0.31 | yes | 0.566 | 1.085 |
| middle | old | 0.42 | yes | 0.64 | 2.078 |
| middle | sac | 0.11 | yes | 0.707 | 1.336 |
| pinky | lut | 4.02 | no | 0.0 | 0.0 |
| pinky | new | 0.26 | no | 0.0 | 0.0 |
| pinky | old | 0.88 | no | 0.0 | 0.0 |
| pinky | sac | 0.24 | no | 0.0 | 0.0 |
| ring | lut | 0.18 | yes | 0.0 | 0.264 |
| ring | new | 0.12 | yes | 0.0 | 0.173 |
| ring | old | 0.49 | yes | 0.0 | 0.608 |
| ring | sac | 0.2 | yes | 0.0 | 0.173 |

Motor faults: 6 ({'wrist': 6}); poses skipped: 4
- wrist id 1 ['overload'] at 20261108-night-N04-V2/v-ref-mid (after v-stop-flex-lo: wrist -65 with all fingers at their extension stops), 40.0 °C, 999.0 mA
- wrist id 1 ['stuck'] at 20261108-night-N05-V3/v-stop-flex-lo#0, 41.0 °C, 30.0 mA
- wrist id 1 ['stuck'] at 20261108-night-N05-V3/v-stop-thumb#0, 42.0 °C, 77.2 mA
- wrist id 1 ['stuck'] at 20261108-night-N05-V3/v-stop-thumb#1, 42.0 °C, 0.6 mA
- wrist id 1 ['stuck'] at 20261108-night-N05-V3/v-ref-pinch#0, 42.0 °C, 0.6 mA
- wrist id 1 ['stuck'] at 20261108-night-N05-V3/v-ref-pinch#1, 42.0 °C, 0.2 mA
