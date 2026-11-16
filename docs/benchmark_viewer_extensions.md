# Extending the benchmark lap viewer

Race Performance owns the track SVG, preserved occupancy background, lap controls,
selection/pinning, cursor, keyboard navigation, zoom synchronization, and plot
layout. Modules contribute data rather than another map or another selection bus.
The original raceline/localized pose/ground-truth overlay remains the default.

Register a module with `Module(..., report_extensions=extension)`. Its hook receives
the module result and returns:

```python
{
    'host': 'race-performance',
    'fallback': {'viewer': shared_viewer_payload},
    'viewer': {
        'id': 'my_module',
        'title': 'My diagnostics',
        'samples': observations,
        'signals': {
            'Value|units|My diagnostics|Panel title': values,
        },
        'colors': {'Value|units|My diagnostics|Panel title': '#2563eb'},
        'overlays': [{
            'id': 'value', 'title': 'My diagnostics · value',
            'signal': 'Value|units|My diagnostics|Panel title',
            'range': [0, 2], 'palette': 'sequential', 'units': 'units',
        }],
        'reference_lines': {'Panel title': [1]},
        'notes': 'Explain units, availability and alignment here.',
    },
}
```

`signals` arrays align one-to-one with this extension's observations. The four
pipe-separated key fields are trace name, units, group (matching `title`), and
panel title. Trace names should be unique within the contribution. Each group
appears as a collapsed dropdown section on the plot side. Legend visibility is
preserved across lap changes; x zoom and the spatial cursor use the shared viewer.
No JavaScript or changes to the generic renderer are needed for another module
using these plot/overlay types. Module hooks are trusted installed Python code;
all data labels are escaped or assigned through DOM `textContent`.

An observation supplies:

```python
{
    'stamp_ns': '1000000000',  # absolute transport source stamp; string avoids JS integer loss
    'segment': 0, 'domain': 'transport',
    'recorded': 1234567890,   # same measurement-boundary basis as host lap start/end
    'xy': [1.0, 2.0],        # map-frame vehicle position, or None
    'heading': 0.1, 'd': 4.2,  # saved-reference progress in metres, or None
    'continuous': False,      # false breaks the preceding connection
    'details': [('Evidence label', 'Recorded evidence value')],
}
```

Keep acquisition order. Retain unavailable values as `None`; never fill gaps with
zero. The host additionally breaks lines at reference wraps. Color ranges must be
computed from the entire contribution, across all laps and wheels, before any
rendering reduction; use `diverging` with symmetric bounds for signed quantities,
or `sequential` for nonnegative quantities. The shared numeric legend identifies
gray/unavailable observations. Markers locate vehicles, not physical contacts.

Extension observations have their own namespace (`extension`, `observationIndex`,
`observationStamp`) in `window.benchmarkReport` selection state. Base pose indexes
remain base pose indexes. Plot hover/click and map hover/click select actual
observations from the active source. Arrow navigation steps that source while it
is selected; lap changes restore base pose selection. Shared cursors indicate
geometric position, but cross-source telemetry labels and truth markers require
an exact unique absolute stamp, compatible segment and domain. There is no
nearest-sample substitution. `details` can explain estimated positions without
changing that contract. Preparation owns validity and interpolation policy.

The generic report renderer collects contributions through the module registry,
then attaches them to the module registered with
`report_host='race-performance'`. If it wasn't selected, it renders that same host
once using the contribution's fallback viewer. `race_view.preserved_payload`
builds this fallback directly from preserved assets and signals without running
Race Performance analysis or depending on `data.shared`. The contribution's own
`render_report` can still render a static overall card; wheel contact uses this
for the full-window slip summary. Host rendering precedes contribution cards,
including when CLI module selection is reversed.

See `wheel_contact_view.extension` for the four-wheel implementation and
`test_report_extensions.py` / `test_wheel_slip_browser.py` for composition and
selection regressions. Reanalyze into a new analysis ID to update existing HTML;
saved reports and sealed runs are never rewritten.


The wheel-contact contribution uses one **Slip** overlay: its per-observation
value is the maximum valid forward/sideways peak-slip ratio across all four
wheels. The winning wheel/direction remains in observation details; the four
plots still show individual wheels. Overlays may provide a `values` array instead
of a `signal` key when the displayed quantity is a reduction rather than a plotted
trace. Generic `color_stops` (numeric `value` plus RGB `color`), `above_color`, and
`legend` entries support anchored thresholds: wheel slip uses green at 0, yellow
at 1 and red above 1, independent of the observed maximum. Module `notes` are
optional; wheel slip omits its explanatory paragraph from the plot group.

Open/closed state is retained across laps for both groups and individual plot
panels. Keys use module ID, group and panel title; keep those stable across laps.
