"""Default renderer for module results and safe static figure artifacts."""
import html
import json
from pathlib import Path


def render(output, summary):
    output = Path(output)
    esc = lambda value: html.escape(str(value))
    sections = ['<!doctype html><html><head><meta charset="utf-8"><title>Race benchmark</title>',
                '<style>body{font:16px system-ui;max-width:1100px;margin:2em auto;padding:1em}'
                'table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:.5em;text-align:left}'
                'pre{white-space:pre-wrap}img{max-width:100%}.warning{color:#854000}</style></head><body>',
                '<h1>Race benchmark</h1>', '<h2>Experiment and analysis</h2><pre>',
                esc(json.dumps({k: v for k, v in summary.items() if k != 'modules'}, indent=2)), '</pre>']
    for result in summary['modules']:
        sections.append(f'<h2>{esc(result["name"])} · {esc(result["status"])}</h2>')
        for warning in result.get('warnings', []):
            sections.append(f'<p class="warning">{esc(warning)}</p>')
        sections.append('<table><tr><th>Metric</th><th>Value</th><th>Units</th><th>Evidence / availability</th></tr>')
        for name, metric in result.get('metrics', {}).items():
            reason = metric.get('reason', '') if metric.get('value') is None else ''
            reason += ('; v2: '+metric['v2_requirement']) if metric.get('v2_requirement') else ''
            sections.append('<tr>'+''.join(f'<td>{esc(v)}</td>' for v in (
                name, json.dumps(metric['value'], allow_nan=False), metric.get('units') or '',
                reason or metric.get('status', '')) )+'</tr>')
        sections.append('</table>')
        for figure in result.get('figures', []):
            relative = Path(figure['path'])
            target = (output / relative).resolve()
            if relative.is_absolute() or not target.is_relative_to(output.resolve()) or not target.is_file():
                sections.append('<p class="warning">Unavailable or unsafe figure artifact</p>')
                continue
            if target.suffix.lower() not in ('.png', '.jpg', '.jpeg'):
                sections.append('<p class="warning">Unsupported static figure format</p>')
                continue
            sections.append(f'<figure><img src="{esc(relative.as_posix())}"><figcaption>'
                            f'{esc(figure.get("caption", ""))}</figcaption></figure>')
    sections.append('</body></html>')
    (output / 'report.html').write_text(''.join(sections))
