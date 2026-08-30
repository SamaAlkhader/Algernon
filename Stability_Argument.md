# Center of Gravity & Stability Argument

## 1. Method

The chassis CoG is computed with the standard weighted-average method,
applied independently in each horizontal axis:

```
x_CoG = Σ(m_i · x_i) / Σm_i
y_CoG = Σ(m_i · y_i) / Σm_i
```

Every major component (battery, two drive motors, main PCB, front
sensor array, wheel/gearbox assemblies, printed shell) is listed with
an assumed mass and position in `CoG_Calculation.xlsx`. All masses and
positions are currently **assumptions** (marked in blue/yellow in the
sheet) because exact components have not been purchased yet — the
sheet recalculates automatically once real datasheet values are typed
in.

## 2. Placement strategy

Two failure modes drive the layout:

- **Nose-diving under braking** — if the CoG sits too far forward (or
  too high), hard braking rotates the chassis forward about the front
  wheel contact patch.
- **Flipping / rollover in fast turns** — if the CoG sits too far to
  one side, or too high above the wheelbase, cornering forces can tip
  the chassis before the wheels lose traction.

To counter both:

- The **battery**, the single heaviest component, is placed low and
  toward the rear-center of the chassis. This pulls the CoG rearward
  and downward, which directly resists forward pitch (nose-dive) under
  braking, and lowers the roll moment during turns.
- The two **drive motors** are placed symmetrically left/right at the
  same y-position, which keeps the CoG centered on the width axis
  (x_CoG ≈ 50% of footprint width) so cornering load is shared evenly
  between the left and right wheels rather than favoring one side.
- The **sensor array** (front-mounted, required for wall detection) is
  kept as light as possible at the front, since it is the only
  necessarily-forward mass; the battery placement is chosen specifically
  to offset the pitch moment this creates.

## 3. Quantitative check

The spreadsheet converts the computed CoG into two ratios against the
chassis footprint:

- **CoG X / footprint width** — target 40–60%, i.e. centered
  left-to-right. A value outside this range means one side is
  overloaded and turning stability is asymmetric.
- **CoG Y / footprint length (measured from the rear)** — target
  ≤ 45%, i.e. biased toward the rear half. This keeps the CoG behind
  the mouse's mid-length, which resists the forward pitching moment
  generated during braking.

With the current assumed values, both ratios fall inside their target
bands (see the sheet's "Stability checks" section) — this is flagged
explicitly as based on assumed masses/positions until real components
are measured.

## 4. What would invalidate this argument

- If the real battery or motors turn out to be significantly heavier
  or lighter than assumed, both target ratios must be recomputed —
  the spreadsheet already does this automatically once new values are
  entered.
- If the final chassis footprint (from Sahar's structural layout)
  differs from the 80 mm × 150 mm assumption used here, the footprint
  cells in the sheet must be updated to match the real design before
  the percentages are trusted.
