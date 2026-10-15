# 90-degree trochoidal gear generator

This project generates an experimental right-angle roller-pinion transmission by
subtracting sampled roller positions from a cylindrical driven-gear blank. The
pinion remains on global **Z** and the driven gear
rotates about global **Y**, with its positive circular face pointing directly
toward the pinion. Both discs use the same 30 mm radius. The barrel-center circle
is inset by one barrel radius, and the pinion Y position is derived so its nearest
barrel overlaps the driven +Y face. The driven axis is lifted to Z=30 mm, placing
the barrel plane at the lower driven rim rather than the face center. At each
sample the cutter is transformed into the driven gear's local rotating frame:

`C(q) = inverse(T_output(q)) * T_input(q) * C`

This is a numerical 3D envelope, not a linear rack profile and not a conventional
bevel-gear tooth formula.

## Motivations

Inspired by Mishin Machine's video.
https://www.youtube.com/watch?v=VXcuryyRGbo&t=183s

Related Papers on linear actuator.
- https://www.mdpi.com/2076-3417/12/11/5620
- https://patents.google.com/patent/TW202100895A/en

## Generate the files

[uv](https://docs.astral.sh/uv/) installs a compatible Python and CadQuery:

```bash
uv sync
uv run python -m rollergear --preview
uv run python -m rollergear --samples 120
```

When `--output` is omitted, every run gets a new timestamped directory such as
`output/run-20261015-123456`. If two runs start during the same second, the next
directory gets `-2`, `-3`, and so on, so an earlier model is never overwritten.
Pass `--output output/final` when you intentionally want a specific directory.

The final command creates:

- `driven_gear.step` and `driven_gear.stl`
- `roller_pinion_hub.step` and `roller_pinion_hub.stl`
- `roller.step` and `roller.stl` (print `roller_count` copies)
- `roller_pinion_assembly.step` and `.stl` — pinion disc with all barrel teeth
- `right_angle_assembly.step` and `.stl` — both discs positioned at 90°
- `validation_report.json`
- `model_metadata.json`

`model_metadata.json` records the exact command-line arguments, parsed values,
fully resolved configuration (including default and derived dimensions), generation
timestamp, volume metrics, and a size plus SHA-256 checksum for every CAD file.

STEP preserves exact B-rep surfaces and is the preferred Fusion import. STL is
in millimetres; STL does not itself store a unit, so choose millimetres on import.
Open `right_angle_assembly.step` first when checking orientation. The output gear
rotates about Y; the pinion disc rotates about Z, so their axes are perpendicular.
The driven +Y face is positioned toward the pinion instead of presenting its
perimeter. Barrel teeth project from the pinion's positive-Z face.
The pinion carrier automatically reaches the barrel-center pitch circle. It does
not extend over the complete barrel footprint because that would make the
perpendicular carrier collide with the driven disc at the mesh point. Use
`--pinion-disc-radius` only when deliberately overriding this clearance.

Useful adjustments:

```bash
uv run python -m rollergear \
  --module 2 --rollers 10 --roller-coeff 0.8 \
  --roller-length 30 --pinion-face-gap 0 --pinion-disc-radius 30 \
  --gear-ratio 1 --driven-radius 30 --driven-thickness 10 --driven-hole 10 \
  --driven-hub-radius 30 --driven-hub-length 5 \
  --input-x 0 --input-y 30 --input-z 0 \
  --driven-x 0 --driven-y 0 --driven-z 30 \
  --phase 0 --direction -1 \
  --samples 240 --clearance 0.15 --cut-batch-size 32 \
  --output output/custom
```

## Command-line parameters

All dimensions are in millimetres and all angles are in degrees. A “resolved
default” depends on another parameter; the values shown in parentheses are the
resolved values with an otherwise default command.

| Option | Default | What it controls |
| --- | --- | --- |
| `-h`, `--help` | — | Prints the complete command-line help and exits without generating a model. |
| `--module` | `2.0 mm` | Sets the basic scale used to calculate barrel diameter. Barrel diameter is `module × roller coefficient`. |
| `--rollers` | `10` | Sets the number of equally spaced barrel teeth around the input pinion. It also sets the number of repeated sectors used by the experimental sector-cut mode. |
| `--roller-coeff` | `0.8` | Multiplies the module to determine barrel diameter. The defaults produce a `1.6 mm` diameter and `0.8 mm` radius. |
| `--roller-length` | Driven radius (`30 mm`) | Sets barrel length along `+Z`, starting at `--input-z`. When omitted, it follows `--driven-radius`. |
| `--pinion-disc-radius` | Driven radius (`30 mm`) | Sets the input pinion-disc radius. The current model requires it to equal `--driven-radius`; omitting it maintains that equality automatically. |
| `--pinion-face-gap` | `0.0 mm` | Sets the axial gap between the pinion disc’s positive-Z face and the start of the barrels. Increasing it moves the disc farther toward `-Z`; it does not move the barrels. |
| `--gear-ratio` | `1.0` | Sets input revolutions per driven revolution. This proof of concept requires a positive integer so the sampled motion closes. |
| `--driven-radius` | `30.0 mm` | Sets the driven-gear blank radius. It also supplies the default pinion radius, barrel length, input Y position, and driven Z position. |
| `--driven-thickness` | `10.0 mm` | Sets driven-gear thickness. The blank starts at `--driven-y` and extends this distance toward `+Y`. |
| `--driven-hole` | `10.0 mm` | Sets the driven gear’s center-hole diameter. |
| `--driven-hub-radius` | Driven radius (`30 mm`) | Sets the radius of the centered base fused to the driven gear’s negative-Y face. When omitted, it follows `--driven-radius`; an explicit value cannot exceed the gear radius. |
| `--driven-hub-length` | `5.0 mm` | Sets how far the driven mounting hub extends from the start face toward `−Y`. Use `0` to disable the hub. |
| `--input-x` | `0.0 mm` | Sets the pinion-axis X coordinate and shifts the complete pinion assembly along X. |
| `--input-y` | Pinion radius (`30 mm`) | Sets the pinion-axis Y coordinate. When omitted, it follows the resolved pinion-disc radius. |
| `--input-z` | `0.0 mm` | Sets the barrel start plane on Z. Barrels extend from this coordinate toward `+Z`; the pinion disc remains behind this plane. |
| `--driven-x` | `0.0 mm` | Sets the driven gear’s X coordinate. |
| `--driven-y` | `0.0 mm` | Sets the driven gear’s negative-Y start face. Its thickness extends from here toward `+Y`. |
| `--driven-z` | Driven radius (`30 mm`) | Sets the driven axis height on Z. When omitted, it follows `--driven-radius`, placing the barrel plane at the lower rim. |
| `--phase` | `0.0°` | Rotates the driven gear around its Y axis before the synchronized cutting motion starts. It changes initial tooth-pattern alignment without changing dimensions or ratio. |
| `--direction` | `-1` | Sets the driven rotation sign. Accepted values are `-1` for opposite signed rotation and `1` for the alternate synchronized direction. |
| `--samples` | `120` per input revolution | Sets motion-sampling resolution. More samples reduce envelope scalloping but increase precheck and Boolean time. |
| `--clearance` | `0.15 mm` | Expands virtual cutter radius and both cutter ends. It adds running clearance without enlarging the exported printable barrels. |
| `--cut-batch-size` | `32` | Sets the maximum number of explicit barrel cutters per OpenCascade Boolean operation. Use `1` for legacy sequential cutting. Failed batches automatically fall back to individual cuts. |
| `--experimental-sector-cut`, `--no-experimental-sector-cut` | On for `1:1`; off otherwise | Enables or disables cutting one overlapped angular slice and reconstructing the driven gear by rotation. Sector cutting is supported only for a 1:1 ratio, requires at least three barrels, and requires `--samples` divisible by the barrel count. |
| `--verify-clearance` | Off | Runs the optional post-cut intersection check against every retained barrel pose and aborts if sampled overlap remains. This adds generation time. |
| `--output` | New timestamped directory | Selects the output directory. When omitted, the command creates `output/run-YYYYMMDD-HHMMSS`, adding a numeric suffix if necessary. |
| `--preview` | Off | Forces a quick draft with `30` samples per input revolution, overriding the value supplied by `--samples`. |

Older commands may still use the hidden compatibility aliases
`--input-axis-x`, `--input-axis-y`, `--input-axis-z`, `--driven-center-y`, and
`--driven-center-z`. They map directly to the corresponding shorter options
above, but new commands should use `--input-*` and `--driven-*`.

`--samples` is samples per input revolution. The default 1:1 ratio uses one full
input revolution and one full driven revolution. Start with
`--preview`; increase sampling only after inspecting the layout. More samples
reduce scalloping but increase Boolean time.

For a 1:1 ratio, the default symmetry mode cuts an overlapped working slice,
trims it back to exactly `360 / rollers` degrees, and rotates copies around the
driven axis. No extra flag is required:

```bash
uv run python -m rollergear
```

Use `--no-experimental-sector-cut` to force full-circle cutting for comparison.
Sector mode is limited to a 1:1 ratio, requires at least three barrels, and
requires `--samples` to be divisible by the barrel count. Reconstructed sector
boundaries can be more sensitive to OpenCascade tolerances than full-circle
generation. Valid disconnected multi-solid results are exported and their
solid count is recorded in `validation_report.json`; inspect the resulting STEP
before manufacturing it.

During generation the CLI displays live progress for both expensive stages:
sampling the synchronized roller sweep and applying the OpenCascade Boolean
cuts. Every barrel at every sampled motion state is considered. A broad-phase
test uses the full transformed cylinder bounds, followed by an exact intersection
against the original simple blank. Only poses with positive overlap reach the
Boolean stage. Those cutters are passed directly to OpenCascade in batches of
32, reducing how often the increasingly complex gear must be rebuilt. If a
batch produces invalid topology, it automatically falls back to individual
cuts. Use `--cut-batch-size 1` to force the legacy sequential behavior. This is
safe because the finished gear is a subset of the original blank. Post-cut
clearance checking is skipped by default
to avoid repeating intersection work. Add `--verify-clearance` to check every
retained overlapping pose and abort export if sampled overlap remains. The CLI
then reports each STEP/STL export and metadata/checksum stage. When
output is redirected to a log, progress is emitted at 10% intervals instead of
printing an updating terminal bar.

```bash
uv run python -m rollergear --verify-clearance
```

## Design assumptions and limits

- Pinion and driven discs both default to a 30 mm radius. The 0.8 mm barrel
  radius puts barrel centers at a 29.2 mm pitch radius. Barrel length defaults
  to the driven radius, so the default barrels extend 30 mm from their start
  plane toward +Z. `--roller-length` overrides this resolved default.
- The driven gear center-hole diameter defaults to 10 mm and can be changed
  with `--driven-hole`.
- A concentric driven base defaults to the full driven-gear radius by 5 mm
  long. It projects from the driven start face toward −Y and retains the same
  through hole as the driven gear. `--driven-hub-radius` can make it smaller.
- The default pinion center is `(0, 30, 0)`: its Y coordinate equals the
  default 30 mm pinion radius. Its barrels start at Z=0 and extend toward +Z.
  The driven disc origin is `(0, 0, 30)`: Y=0 is its start face, thickness
  extends toward +Y, and Z equals its default 30 mm radius.
- `--input-x`, `--input-y`, and `--input-z` position the pinion axis and the
  barrel start plane.
- `--driven-x`, `--driven-y`, and `--driven-z` independently position the
  driven-gear start-face origin and circular center axis.
- `--input-z` moves only the pinion assembly. The driven gear remains at its
  independent default Z center of 30 mm; move it separately with
  `--driven-z`.
- Clearance enlarges cutter rollers, not the printable rollers.
- The proof of concept requires an integer ratio so the sampled motion closes
  after one output revolution.
- The script detects a sweep that misses the blank, negligible cutting, and a
  sweep that removes more than 85% of the blank.
- The OpenCascade cuts use bounded multi-tool batches rather than a compound.
  Every retained cutter is supplied explicitly, and invalid batch results fall
  back to individual cuts.
- `connected_solid_count` reports disconnected output components. A count above
  one is allowed, including in experimental sector mode. `mesh_watertight` is
  only true for one valid connected solid. It does **not**
  prove contact continuity, torque transfer, acceptable backlash, strength, or
  safe powered operation. Those remain explicitly unverified in the JSON report.

Run the kinematics checks with:

```bash
uv run --with pytest pytest -q
```
