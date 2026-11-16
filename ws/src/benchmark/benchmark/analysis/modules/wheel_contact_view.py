"""Wheel-slip contributions to the shared race viewer and overall summary."""
import html
import json

from ..core.wheel_contact import WHEEL_NAMES


def extension(result):
    """Declarative viewer contract: own observations, signals, overlays and evidence."""
    viewer = result.get('viewer', {})
    samples = viewer.get('samples', [])
    colors = ['#2563eb', '#d97706', '#16a34a', '#9333ea']
    labels = ['FL', 'FR', 'RL', 'RR']
    signals, signal_colors, overlays = {}, {}, []
    panels = [('forward', 'raw', 'Signed forward slip'), ('sideways', 'raw', 'Signed sideways slip'),
              ('forward', 'ratio', 'Forward slip / configured peak'),
              ('sideways', 'ratio', 'Sideways slip / configured peak')]
    for direction, field, panel in panels:
        units = 'Unity slip coordinate' if field == 'raw' else 'Slip / configured peak slip<br>(dimensionless)'
        keys = []
        for name, label, color in zip(WHEEL_NAMES, labels, colors):
            key = f'{label} {direction} {field}|{units}|Wheel slip|{panel}'
            signals[key] = [s['wheels'][name][direction][field] for s in samples]
            signal_colors[key] = color
            keys.append(key)
    # At each observation choose the largest valid ratio across both axes and
    # all four wheels. Missing coordinates never contribute a fabricated zero.
    maximum_ratios, dominant = [], []
    for sample in samples:
        candidates = [(sample['wheels'][name][direction]['ratio'], label, direction)
                      for name, label in zip(WHEEL_NAMES, labels)
                      for direction in ('forward', 'sideways')
                      if sample['wheels'][name][direction]['ratio'] is not None]
        chosen = max(candidates, key=lambda item: item[0]) if candidates else None
        maximum_ratios.append(chosen[0] if chosen else None)
        dominant.append(f'{chosen[1]} {chosen[2]}' if chosen else 'Unavailable')
    maximum = max((v for v in maximum_ratios if v is not None), default=1)
    overlays = [{'id': 'slip', 'title': 'Slip', 'values': maximum_ratios,
                 'range': [0, max(1, maximum)], 'units': 'dimensionless',
                 'color_stops': [{'value': 0, 'color': [22, 163, 74]},
                                 {'value': 1, 'color': [250, 204, 21]}],
                 'above_color': [220, 38, 38],
                 'legend': [{'value': 0, 'label': '0'}, {'value': 1, 'label': '1'},
                            {'value': 2, 'label': f'>1 (max {maximum:.4g})' if maximum > 1 else '>1'}]}]
    observations = []
    for index, s in enumerate(samples):
        details = [('Maximum slip ratio', f'{maximum_ratios[index]} · {dominant[index]}'), ('Bridge arrival timestamp (ns)', s['stamp_ns'] or 'Unavailable'), ('Position association', s['alignment']['method'] + (' (estimate)' if s['alignment']['method'] == 'interpolated' else '')),
                   ('Alignment evidence', json.dumps(s['alignment'])), ('Source identity', json.dumps(s['source_identity']))]
        for name, label in zip(WHEEL_NAMES, labels):
            wheel = s['wheels'][name]
            details.append((label+' contact', f"grounded={wheel['grounded']}, valid={wheel['valid']}"))
            for direction in ('forward', 'sideways'):
                v = wheel[direction]
                def fmt(x):
                    return 'Unavailable' if x is None else f'{x:.5g}'
                details.append((label+' '+direction, f"raw {fmt(v['raw'])} · ratio {fmt(v['ratio'])} · runtime peak {fmt(v['peak'])}"))
        observations.append({'stamp_ns': s['stamp_ns'], 'segment': s['segment'], 'domain': s['domain'], 'recorded': int(s['boundary_ns']),
                             'xy': s['xy'], 'd': s['d'], 'heading': s['pose'][2] if s['pose'] else None,
                             'continuous': s['continuous'], 'details': details})
    return {'host': 'race-performance', 'viewer': {'id': 'wheel_contact', 'title': 'Wheel slip',
            'samples': observations, 'signals': signals, 'colors': signal_colors, 'overlays': overlays,
            'reference_lines': {p[2]: [1] for p in panels if p[1] == 'ratio'},
            'notes': ''},
            'fallback': {'viewer': result.get('race_viewer', {'laps': [], 'reason': 'Localized poses and preserved raceline required for spatial inspection'})}}


def render_report(result):
    esc = lambda x: html.escape(str(x))
    def fmt(x):
        return 'Unavailable' if x is None else f'{x:.5g}' if isinstance(x, float) else str(x)
    tables = []
    for direction, wheels in result.get('slip_summary', {}).items():
        rows = []
        for name, value in wheels.items():
            a, r = value['absolute_slip'], value['ratio']
            cells = [name, str(a['count'])+' / '+str(r['count']), a['mean'], a['p95'], a['max'],
                     r['mean'], r['p95'], r['max'], str(value['at_or_above_peak_count'])+' / '+fmt(value['at_or_above_peak_fraction'])]
            rows.append('<tr>'+''.join('<td>'+esc(fmt(x))+'</td>' for x in cells)+'</tr>')
        tables.append('<h4>'+esc(direction.title())+'</h4><div style="overflow:auto"><table><thead><tr>'+''.join('<th>'+x+'</th>' for x in ['Wheel', 'Valid raw / ratio counts', 'Mean absolute slip', 'P95 absolute slip', 'Maximum absolute slip', 'Mean ratio', 'P95 ratio', 'Maximum ratio', 'Samples at/above configured peak-slip coordinate (count / fraction)'])+'</tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>')
    metrics = result.get('metrics', {})
    diagnostic = '<table>'+''.join('<tr><td>'+esc(k)+'</td><td>'+esc(json.dumps(v, allow_nan=False))+'</td></tr>' for k, v in metrics.items())+'</table>'
    coverage = ' · '.join(esc(k.replace('_', ' '))+': '+esc(fmt(v['value'])) for k, v in metrics.items() if any(t in k for t in ('samples', 'gaps', 'laps', 'spatial_match', 'revisions')))
    return ('<section id="wheel-slip"><div class="slip-summary"><h3>Overall Slip Summary</h3><p>Entire measured window: all unique eligible source observations, including partial laps and observations without spatial matches. Sample weighted; fractions are neither elapsed-time nor distance fractions. Lap and map controls do not change these tables.</p>'+''.join(tables)+'<p>'+coverage+'</p><details><summary>Diagnostic details: raw forces, configuration and contamination</summary><p>Build hashes and runtime snapshots: <a href="wheel_contact/contacts.jsonl">raw contact envelopes</a>. Alignment evidence: <a href="wheel_contact/slip_samples.jsonl">unique slip observations</a>. Configuration revisions can include numerical noise.</p>'+diagnostic+'<pre>'+esc(json.dumps(result.get('contamination', {}), indent=2))+'</pre></details></div></section>')
