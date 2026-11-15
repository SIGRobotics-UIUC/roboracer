"""Default renderer for module results and safe static figure artifacts."""
import html
import json
from pathlib import Path


def render(output, summary):
    output = Path(output)
    esc = lambda value: html.escape(str(value))
    sections = ['<!doctype html><html><head><meta charset="utf-8"><title>Benchmark Report</title>',
                '<style>body{font:16px system-ui;max-width:1100px;margin:2em auto;padding:1em}'
                'table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:.5em;text-align:left}'
                'pre{white-space:pre-wrap}img{max-width:100%}.warning{color:#854000}</style></head><body>',
                '<h1>Benchmark Report</h1>', core(summary)]
    sections.append(SHARED)
    from ..analysis.core.results import REGISTRY
    for result in summary['modules']:
        if result['name'] == 'computation_timing':
            continue
        module = REGISTRY.get(result['name'])
        if module and module.render_report:
            sections.append(module.render_report(result))
            continue
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


SHARED = """<script>
document.addEventListener('click',async event=>{
 const button=event.target.closest('.copy-commit');if(!button)return;
 try {await navigator.clipboard.writeText(button.dataset.commit);button.nextElementSibling.textContent='Copied';}
 catch {const input=document.createElement('textarea');input.value=button.dataset.commit;document.body.append(input);input.select();const ok=document.execCommand('copy');input.remove();button.nextElementSibling.textContent=ok?'Copied':'Copy unavailable';}
});
window.benchmarkReport = (() => {
 let selection = Object.freeze({lap:null,timestamp:null}), listeners = new Set();
 return Object.freeze({getSelection:()=>selection,
 subscribe:fn=>{listeners.add(fn);return ()=>listeners.delete(fn)},
 requestSelection:change=>{selection=Object.freeze({...selection,...change});
 listeners.forEach(fn=>fn(selection));}});
})();
</script><style>
body{background:#f5f7fa;color:#172438;max-width:1280px}
h1{display:inline-block;margin-right:1rem}small{color:#687483}
section{background:white;border:0;border-radius:10px;padding:20px;margin:20px 0}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:16px}
.cards dt{color:#687483;font-size:13px}.cards dd{margin:4px 0;font-weight:600}
.commit{display:flex;align-items:center;gap:10px;margin:16px 0}.commit code{background:#edf1f6;border-radius:5px;padding:6px}.copy-commit{border:0;background:#edf1f6;border-radius:5px}details{margin:12px 0}summary,button{cursor:pointer}button,select{padding:8px 13px;border:1px solid #d7dfeb;border-radius:7px;background:#fff;color:#263b56;font:inherit;font-size:13px;transition:background .15s,border-color .15s}button:hover:not(:disabled){background:#eef4ff;border-color:#96b5e3}button:active:not(:disabled){background:#ddeaff}button:disabled{opacity:.4;cursor:default}button:focus-visible,select:focus-visible{outline:2px solid #0671ee;outline-offset:2px}td,th{border-color:#e3e7ed}
</style>"""


def core(summary):
    """Collection facts remain available independently of analysis modules."""
    esc = lambda x: html.escape(str(x))
    meta = summary.get('experiment', {})
    config = meta.get('config', {})
    exp = config.get('experiment', {})
    components = config.get('components', {})
    measured = meta.get('measurement', {})
    from ..collector.metadata import git_metadata, initial_pose_metadata
    initial = meta.get('initial_pose_metadata') or initial_pose_metadata(config)
    initial_label = ('Explicit pose supplied; ' if initial['explicitly_supplied'] else 'No explicit pose supplied; ')
    initial_label += ('resolved default: '+str(initial['resolved_pose']) if initial.get('resolution') == 'resolved'
                      else 'simulator default unresolved')
    def component(name):
        value = components.get(name)
        if isinstance(value, dict):
            if value.get('enabled') is False or value.get('type') in ('none', 'disabled'):
                return 'None'
            return value.get('type', 'Unavailable')
        return value if value is not None else 'Unavailable'
    facts = {'Map': exp.get('map', 'Unavailable'),
             'Localization': component('localization'),
             'Online planner': component('online_planner'),
             'Controller': component('controller'),
             'Requested laps': exp.get('laps', 'Unavailable'),
             'Completed laps': measured.get('completed_laps', 'Unavailable'),
             'Initial pose (x [m], y [m], yaw [rad])': initial_label}
    def readable(value):
        if value is None:
            return 'Not recorded'
        if isinstance(value, bool):
            return 'Enabled' if value else 'Disabled'
        if isinstance(value, float):
            return f'{value:,.3f}'.rstrip('0').rstrip('.')
        if isinstance(value, dict):
            return '; '.join(k.replace('_', ' ')+': '+readable(v) for k,v in value.items()) or 'None'
        if isinstance(value, list):
            return ', '.join(map(readable, value)) or 'None'
        return str(value).replace('_', ' ')

    def fields(value, prefix=''):
        rows = []
        for key, item in value.items():
            label = (prefix+' / ' if prefix else '')+key.replace('_', ' ')
            if isinstance(item, dict):
                rows.extend(fields(item, label))
            else:
                rows.append((label, readable(item)))
        return rows

    def table(rows):
        return '<table>'+''.join('<tr><td>'+esc(k)+'</td><td>'+esc(v)+'</td></tr>' for k,v in rows)+'</table>'

    for role, component_config in components.items():
        if isinstance(component_config, dict):
            for name,value in component_config.get('params', {}).items():
                facts[role.replace('_', ' ')+' · '+name.replace('_', ' ')] = value
    source = git_metadata(meta.get('git'))
    commit = source.get('commit') or 'Unavailable'
    branch = source.get('branch') or {'detached': 'Detached HEAD', 'unavailable': 'Git unavailable'}.get(source['branch_state'], 'Branch not recorded')
    dirty_label = 'dirty' if source['dirty'] is True else 'clean' if source['dirty'] is False else 'dirty state not recorded'
    cards = ''.join('<div><dt>'+esc(k)+'</dt><dd>'+esc(readable(v))+'</dd></div>' for k,v in facts.items())
    level = config.get('logging', {}).get('level', 'unknown')
    recorder = meta.get('recorder', {})
    counts = recorder.get('counts', {})
    from ..collector.topics import BASE, EXTRA, DETAILED, profile
    duration = meta.get('recording_duration_s', recorder.get('duration_s'))
    inventory = []
    expected, essential = profile({'logging': {'level':level, 'require_ground_truth':
        config.get('logging', {}).get('require_ground_truth', False)}})
    for category, topics in [('Basic', {**BASE, **EXTRA}), ('Detailed', DETAILED)]:
        for topic in topics:
            count = counts.get(topic, 0)
            state = (f'{count:,} messages recorded' if count else
                     'Not collected at this logging level' if level == 'off' or topic not in expected
                     else 'No messages recorded')
            inventory.append((category+' · '+topic, state))
    missing = sorted(t for t in essential if not counts.get(t)) if level != 'off' else []
    issues = []
    if recorder.get('error'):
        issues.append('Recorder error: '+str(recorder['error']))
    if level != 'off' and not recorder.get('closed'):
        issues.append('Recording was not confirmed closed')
    if missing:
        issues.append('Missing required inputs: '+', '.join(missing))
    issues.extend(w for w in summary.get('warnings', []) if any(term in w.lower() for term in ('samples invalid', 'invalid (', 'alignment unavailable', 'alignment split', 'unverified', 'no finalized readable bag')))
    if meta.get('recorder_integrity', {}).get('consistent') is False:
        issues.append('Recorded message counts do not match recorder acknowledgements')
    validity = ('Telemetry disabled by logging configuration' if level == 'off' else
                'Collection evidence needs attention' if issues else 'Recording available')
    status = 'Closed' if recorder.get('closed') else 'Not confirmed closed'
    if level == 'off':
        status = 'Disabled by configuration'
    provenance = table(fields({k:v for k,v in summary.items() if k not in ('modules','experiment','signal_map')}))
    provenance += '<h4>Collection provenance</h4>'+table(fields(meta))
    return ('<small>Run ID: '+esc(summary.get('run_id', 'Unavailable'))+' | Analysis ID: '+
            esc(summary.get('analysis_id', 'Unavailable'))+'</small><section><h2>Experiment Config</h2>'+
            '<dl class="cards">'+cards+'</dl><div class="commit"><span>'+esc(branch)+'</span> <small>'+esc(dirty_label)+'</small> <code>'+esc(commit[:7] if commit != 'Unavailable' else commit)+
            '</code> <button class="copy-commit" data-commit="'+esc(commit)+'" aria-label="Copy full commit" title="Copy full commit">⧉</button><small class="copy-result" aria-live="polite"></small></div>'+
            '<details><summary>All experiment settings</summary>'+table(fields(config))+'</details></section>'+
            '<section><h2>Collected Data</h2><strong>'+esc(validity)+'</strong><dl class="cards">'+
            ''.join('<div><dt>'+esc(k)+'</dt><dd>'+esc(v)+'</dd></div>' for k,v in (
                ('Recording status',status),('Duration', f'{duration:,.2f} s' if duration is not None else 'Not recorded'),
                ('Logging level',level),('Collection outcome',readable(meta.get('status', 'Unavailable')))))+'</dl>'+
            ''.join('<p class="warning">'+esc(issue)+'</p>' for issue in issues)+
            ('<p>Collection failure: '+esc(meta['failure'])+'</p>' if meta.get('failure') else '')+
            '<details><summary>Provenance and saved evidence</summary>'+provenance+'</details>'+
            '<details><summary>Full recording inventory</summary>'+table(inventory)+'</details></section>')
