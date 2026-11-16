# Wheel-slip benchmark report implementation plan

Status: implemented, 2026-11-16 (wheel_contact module 1.1). Verification and
known limits are recorded in `ws/src/benchmark/VALIDATION.md`. No simulator/plugin
changes are required. The original implementation plan follows. The subsequent shared-viewer design
in `benchmark_viewer_extensions.md` supersedes the separate-map layout below:
wheel slip now contributes overlays and plots to the original Race Performance
viewer, retaining only its overall summary as a separate card.

## Outcome and scope

Extend the existing `wheel_contact` benchmark module, retaining its CLI name,
raw exports and metric keys. Give it a custom report renderer with interactive
track-position plots and a separate overall-results card directly below them.
It must work when selected alone, without running `race_performance` first.

The report describes simulator slip coordinates. It does not estimate physical
tire load, force utilization, remaining grip or saturation. See
[the investigation conclusion](tire_utilization_assessment.md).

Use the already recorded `/autodrive/roboracer_1/wheel_contact` stream, localized
map-frame poses, lap evidence and preserved track assets. No plugin installation,
bridge restart, collection schema change, driving or new runtime dependency is
needed to implement and verify the offline report. Contact collection still
requires the existing plugin/bridge opt-in and detailed logging. Old runs can
be analyzed, but missing contact data stays unavailable.

## Report layout

Render the following module-owned section after the existing race section when
both are selected. Keep other report sections and their behavior unchanged.

```text
Wheel Slip Diagnostics
  Lap selector | wheel visibility | map color quantity/wheel
  [track map + selected position] [signed forward slip vs track progress]
                                 [signed sideways slip vs track progress]
                                 [forward slip / configured peak vs progress]
                                 [sideways slip / configured peak vs progress]
  [Overall Slip Summary — separate card, entire measurement window]
     Forward summary table | Sideways summary table
     Coverage and availability | expandable diagnostic details
```

- Four plots, each with FL, FR, RL and RR traces and consistent wheel colors.
  Raw slip is signed and remains in Unity's native slip coordinates; do not
  relabel it as conventional slip ratio, slip angle or degrees.
- Ratios are nonnegative `abs(raw slip) / runtime extremumSlip`, calculated
  separately for forward and sideways directions at each observation. Label
  the axis `Slip / configured peak slip (dimensionless)`, with a reference
  line at 1. Values above 1 stay visible; do not clamp or format as grip percent.
- Horizontal axis is distance along the saved raceline, with a synchronized
  0–100% progress readout. Explain that its origin is the first saved reference
  point, which need not coincide with the native lap-counter boundary.
- Default to the first complete measured lap, otherwise the first partial lap.
  Label incomplete/interrupted laps explicitly. Do not average laps together
  in the initial version; select and inspect them individually.
- Plot hover/click selects an actual retained contact observation, highlights
  its associated vehicle position on the map and shows all four wheels' values.
  Include lap, progress, timestamp, contact state, both raw slips, both ratios,
  runtime peak values and position-association method. Support pin/unpin and
  keyboard navigation between observations.
- The map colors recorded vehicle-position markers for the selected wheel and
  quantity. Use a zero-centered diverging scale for signed slip and a sequential
  scale for ratios, with a numeric legend. Hold the selected quantity's scale
  constant across laps/wheels for the report; do not hide outliers by clipping.
  Label markers as vehicle positions, not measured wheel-contact map locations.
- Missing data produces gaps and unavailable readouts. Break lines at lap wraps,
  resets, session/clock discontinuities, invalid observations and excessive time
  gaps; preserve acquisition order rather than sorting points by progress.
- On narrow screens, stack map and plots. Reuse existing offline Plotly support,
  with no CDN or network requirement. The summary remains readable without JS.

## Observation and position association

Implement one reusable preparation path so plots, exports and summaries use the
same wheel-level values and validity decisions.

1. Apply existing measurement-window selection (`measurement_samples`), excluding
   warm-up and stopping phases according to recorded measurement boundaries.
2. Preserve every raw observation in `contacts.jsonl`. For derived metrics,
   deduplicate by `(session_id, reset_generation, sample_sequence)` within the
   measurement window. Keep the earliest eligible arrival as its representative;
   do not place retransmissions of one cached state at multiple vehicle positions.
   Count duplicates and detect conflicting payloads for the same source identity.
3. Require a grounded, valid wheel and finite raw slip. Use existing curve
   validation for a positive finite runtime peak denominator. A bad curve makes
   its ratio unavailable without erasing otherwise valid raw slip. Ungrounded
   observations are unavailable, never zero. Retain negative raw slip.
4. Associate a unique contact observation with `/localized_pose` using its
   bridge-arrival stamp in the verified transport domain. Prefer an exact pose
   timestamp; otherwise permit bracketed pose interpolation using the existing
   `aligned_pose` policy and `--max-gap-s` (currently 0.1 s). Never extrapolate,
   bridge duplicate/ambiguous timestamps, reset boundaries or mismatched domains.
   Check segmentation compatibility across signals rather than assuming equal
   segment numbers alone establish continuity. Never interpolate slip values.
5. Store association evidence: exact/interpolated/unavailable, bracketing stamps,
   bracket span and reason for rejection. Clearly label interpolated position as
   an estimate. This is transport-time association, not physics-time alignment.
   Keep the existing Race Performance viewer's exact-match contract unchanged.
6. Project associated poses using `project_reference` and the preserved
   `raceline.csv`. Reject ambiguous projection; retain valid map positions even
   if progress is unavailable. Use the preserved map as an optional background.
   Do not depend on mutable files in `ws/tracks/` or on simulator ground truth.
7. Apply the same recorded lap-boundary convention as the race report, including
   partial laps and reset/clock splits. Factor only the shared boundary helper
   if necessary, with regressions preserving existing race-view behavior.

Missing localized poses disables spatial plots while retaining slip summaries.
Missing raceline disables progress plots; valid localized XY can still be shown.
Missing map pixels removes only the background. Missing contact disables both
plots and summaries, with a clear reason rather than a misleading empty chart.

## Separate overall-results card

The card always summarizes **all unique eligible observations in the measured
window**, independently of the selected lap or map quantity. State that scope
in its subtitle; switching laps must not silently change overall statistics.
Include partial-lap observations, but show complete/partial lap counts and any
recorded collision/reset contamination alongside the scope.

Use one compact four-wheel table per direction with these columns:

| Field | Meaning |
|---|---|
| Valid raw / ratio counts | Separate denominators; missing data is excluded explicitly |
| Mean absolute slip | Typical magnitude without signed cancellation |
| P95 absolute slip / maximum absolute slip | Sampled magnitude tails |
| Mean / P95 / maximum peak-slip ratio | Sample statistics of the per-observation ratios |
| Samples at or above configured peak | Count and fraction with ratio >= 1, using valid ratio count |

Display the last field as `Samples at/above configured peak-slip coordinate`,
never `time saturated` or `grip used`. These are sample-weighted observations,
not elapsed-time or track-distance fractions. Do not pool four wheels into an
unexplained overall percentage. Summaries use all eligible contact data,
including samples without a spatial association; coverage distinguishes them.

Show grounded/invalid counts, duplicate observations, spatial match coverage,
exact/interpolated match counts, source gaps and unverified sampling phase.
Put existing raw-force metrics and build/configuration evidence in an expandable
diagnostic table so the new card focuses on slip. Preserve existing JSON keys.
Configuration revisions can change due to numerical noise: report their count,
but do not describe every revision as a deliberate settings change. Highlight
actual peak-denominator changes and compute ratios with each sample's own curve.

## Code and artifacts

1. `analysis/core/wheel_contact.py`: add pure observation/summary helpers, or a
   neighboring `wheel_slip.py` if that keeps decoding separate from preparation.
2. `analysis/modules/wheel_contact.py`: consume the prepared observations, retain
   existing exports/metrics, add magnitude/coverage metrics and viewer payload;
   bump module version to 1.1 for additive reporting changes.
3. Add module-owned `wheel_contact_view.py` and `wheel_contact_view.js`, registering
   the renderer through `Module.render_report`. The generic report renderer
   already supports this hook; avoid module-specific dispatch there.
4. Reuse geometry and lap conventions without depending on another module's
   `data.shared` side effects. Use the existing report selection bus only with
   timestamp/lap semantics that identify matching observations; sample indexes
   from different viewers must not be treated as interchangeable.
5. Write `slip_samples.jsonl` with source identity, raw/ratio values, validity,
   associated pose/progress/lap and alignment evidence. Write compact viewer data
   for the HTML. Compute statistics from the full unique dataset, independent
   of rendering decimation; preserve gap boundaries and extrema if decimating.
6. All artifacts go inside the new analysis directory. Keep strict JSON,
   safe HTML/script escaping and sealed raw-run immutability. Update benchmark
   README, VALIDATION.md and telemetry documentation with labels and examples.

## Verification and acceptance

First implement pure preparation and metrics, then spatial/lap association,
then rendering and exports. Use synthetic fixtures and existing sealed runs;
do not start a controller or simulator for offline verification.

- Calculation tests: signs, per-wheel ordering, changing runtime peaks, ratios
  above 1, invalid/nonfinite/ungrounded cases, signed cancellation, separate
  denominators, duplicate/restart/reset identities and conflicting duplicates.
- Position tests: exact match, bounded interpolation, yaw wrap, stale/missing
  poses, duplicate stamps, incompatible domains/segments, reset boundaries,
  ambiguous projection, track wrap and complete/partial lap assignment.
- Report tests: module selected alone and with `all`, old bag availability,
  summaries present without pose/map/raceline, strict JSON, escaping, raw hashes
  unchanged, correct card placement and stable overall results across lap selection.
- Browser checks using the existing browser-test approach: four plots/four wheels,
  ratio reference line, map/cursor synchronization, wheel/quantity/lap controls,
  pin/keyboard behavior, visible gaps, responsive layout and no console errors.
  Report missing browser dependencies as unperformed checks, not passes.
- Reanalyze a contact-enabled sealed collection into a new analysis ID if one is
  available. Otherwise export a synthetic report and clearly label it synthetic;
  request a future normal gated collection for real track-position review.

Acceptance: a reader can locate each wheel's observed slips and peak-slip ratios
on a lap, inspect their values at a position, and read the entire measurement's
summary in a separate card below. Missing coverage and interpolation remain
visible, existing race reports remain compatible, and no grip/utilization claim
is introduced. Physics-load validation is not a prerequisite for this narrowly
labeled diagnostic report.
