# Standard Floodfill & CoG / Stability

## Files in this folder

| File | Purpose |
|---|---|
| `floodfill.py` | Standard Floodfill algorithm, runs on both 8×8 and 16×16 mazes (maze size read at runtime, nothing hardcoded) |
| `API.py` | Standard mms simulator communication bridge |
| `CoG_Calculation.xlsx` | Weighted-average CoG spreadsheet with live formulas |
| `Stability_Argument.md` | Written reasoning for battery/motor placement and stability |

## Running the Floodfill algorithm in mms

1. Open the mms simulator.
2. Mouse Algorithms → New Algorithm.
3. Set the **Directory** to this folder, and **Run Command** to:
   ```
   python3 floodfill.py
   ```
4. Build/load an **8×8** maze → click Run → let it reach the goal.
5. Build/load a **16×16** maze → click Run → let it reach the goal.

## How this satisfies the "standard Floodfill" requirement

- Flood values are computed with a real breadth-first flood **from the
  goal outward**, recomputed only when a wall is newly discovered.
- The mouse only acts on walls it has actually sensed (`API.wallFront/
  Left/Right`), so it behaves like it would on real hardware with no
  prior maze knowledge — required for the algorithm to count as a fair
  "dynamic mapping" benchmark rather than a pre-solved path.
- Works unmodified on both required maze sizes since dimensions are
  pulled from `API.mazeWidth()/mazeHeight()`.

## CoG Calculation and Stability Argument

- `CoG_Calculation.xlsx` — edit the blue/yellow input cells once real
  component masses and positions are known; everything else
  recalculates automatically.
- `Stability_Argument.md` — the written justification for the
  Mechanical Design PDF. References the spreadsheet's two stability
  ratios (CoG X vs. width, CoG Y vs. length).
