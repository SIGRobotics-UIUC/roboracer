"""Race-owned sample payload and offline Plotly report renderer."""
import base64
import io
import html
import json
from dataclasses import replace
from pathlib import Path
import numpy as np
from ..core.signals import measurement_samples, aligned_pose, project_reference, wrap


def payload(data, rows, truth, config):
    pose = measurement_samples(data, 'pose')
    if rows is None or not pose:
        return {'laps': [], 'reason': 'Requires localized map-frame poses and a preserved raceline'}
    gap = config.get('max_gap_s', .1)
    projection, length = project_reference([s.value[:3] for s in pose], rows)
    origin = pose[0].source_ns
    samples = []
    measured = data.metadata.get('measurement', {})
    use_receipt = measured.get('start_monotonic_ns') is not None
    def boundary_time(sample):
        return sample.received_monotonic_ns if use_receipt else sample.recorded_ns
    # The viewer requires exact recorded timestamps; metric analysis retains its
    # separate bounded-interpolation policy.
    def scalar(name, column):
        source = [replace(s, value=[s.value[column]]*3) for s in measurement_samples(data, name)]
        return aligned_pose(source, pose, gap, interpolate=False)[:, 0]
    aligned_truth = aligned_pose(truth, pose, gap, interpolate=False)
    controller = data.metadata.get('config', {}).get('components', {}).get('controller', {})
    params = controller.get('params', {})
    target = projection[:, 3]*params.get('speed_scale', .5)
    if params.get('max_speed', 0) > 0:
        target = np.minimum(target, params['max_speed'])
    values = {'Wheel odometry speed|m/s|Motion and commands|Speed': scalar('odom', 3),
              'Controller request speed|m/s|Motion and commands|Speed': scalar('requested_drive', 0),
              'Final gate speed|m/s|Motion and commands|Speed': scalar('drive', 0),
              'Controller request steering|rad|Motion and commands|Steering': scalar('requested_drive', 1),
              'Final gate steering|rad|Motion and commands|Steering': scalar('drive', 1),
              'Localized pose heading|rad|Motion and commands|Heading': [s.value[2] for s in pose],
              'Raceline heading|rad|Motion and commands|Heading': wrap(np.array([s.value[2] for s in pose])-projection[:, 2]),
              'Estimated lateral|m|Tracking|Signed lateral error': projection[:, 1],
              'Estimated absolute lateral|m|Tracking|Absolute lateral error': abs(projection[:, 1]),
              'Estimated heading|rad|Tracking|Signed heading error': projection[:, 2],
              'Estimated absolute heading|rad|Tracking|Absolute heading error': abs(projection[:, 2])}
    if controller.get('managed', True):
        values['Effective spatial raceline target|m/s|Motion and commands|Speed'] = target
    if truth:
        values['Ground truth speed|m/s|Motion and commands|Speed'] = aligned_pose(
            [replace(s, value=[s.value[3]]*3) for s in truth], pose, gap, interpolate=False)[:, 0]
        values['Ground truth heading|rad|Motion and commands|Heading'] = aligned_truth[:, 2]
        errors = np.array([s.value[:3] for s in pose])-aligned_truth
        values['Localization position error|m|Localization|Position error'] = np.linalg.norm(errors[:, :2], axis=1)
        values['Localization heading error|rad|Localization|Heading error'] = wrap(errors[:, 2])
        tp = np.full_like(projection, np.nan)
        ok = np.isfinite(aligned_truth).all(axis=1)
        tp[ok], _ = project_reference(aligned_truth[ok], rows)
        tp[tp[:, 4] != 1] = np.nan
        values['True lateral|m|Tracking|Signed lateral error'] = tp[:, 1]
        values['True heading|rad|Tracking|Signed heading error'] = tp[:, 2]
        values['True absolute lateral|m|Tracking|Absolute lateral error'] = abs(tp[:, 1])
        values['True absolute heading|rad|Tracking|Absolute heading error'] = abs(tp[:, 2])
    map_data = None
    if 'map.yaml' in data.assets:
        from planning.centerline import OccupancyMap
        from scipy.ndimage import distance_transform_edt
        grid = OccupancyMap(str(data.assets['map.yaml']))
        pixels = np.rint(grid.to_pixel([s.value[:2] for s in pose])).astype(int)
        dist = distance_transform_edt(np.pad(grid.free, 1))[1:-1, 1:-1]*grid.res-.5*grid.res
        clearance = np.full(len(pose), np.nan)
        ok = (pixels[:, 0]>=0)&(pixels[:, 0]<grid.h)&(pixels[:, 1]>=0)&(pixels[:, 1]<grid.w)
        clearance[ok] = dist[pixels[ok, 0], pixels[ok, 1]]
        values['Vehicle-center distance to nonfree pixels|m|Clearance|Vehicle-center clearance'] = clearance
        from PIL import Image
        import yaml
        spec = yaml.safe_load(data.assets['map.yaml'].read_text())
        image_path = Path(spec['image'])
        if not image_path.is_absolute():
            image_path = data.assets['map.yaml'].parent/image_path
        buffer = io.BytesIO()
        Image.open(image_path).convert('RGB').save(buffer, format='PNG')
        map_data = {'image': 'data:image/png;base64,'+base64.b64encode(buffer.getvalue()).decode(),
                    'width':grid.w, 'height':grid.h, 'resolution':grid.res,
                    'origin':list(map(float, grid.origin))}
    counters = sorted(measurement_samples(data, 'lap_count'), key=lambda s:s.order)
    boundaries = []
    for a,b in zip(counters, counters[1:]):
        if b.segment == a.segment and b.value == a.value+1:
            boundaries.append(boundary_time(b))
    recorded_laps = data.metadata.get('measurement', {}).get('laps', [])
    laps = []
    start = measured.get('start_monotonic_ns' if use_receipt else 'start_recorded_ns', boundary_time(pose[0]))
    for i,end in enumerate(boundaries):
        info = dict(recorded_laps[i]) if i < len(recorded_laps) else {}
        laps.append({**info, 'start': start, 'end':end, 'complete':True})
        start=end
    if not laps or boundary_time(pose[-1]) > start:
        laps.append({'start':start, 'end':boundary_time(pose[-1]), 'complete':False,
                     'start_kind':'interrupted / partial', 'time_s':None})
    for key in values:
        values[key] = np.asarray(values[key], dtype=float)
        values[key][~projection[:, 4].astype(bool)] = np.nan
    resets = measurement_samples(data, 'reset')
    for lap in laps:
        lap['reset_events'] = sum(lap['start'] <= boundary_time(s) <= lap['end'] for s in resets)
        if lap['reset_events']:
            lap['start_kind'] = str(lap.get('start_kind', 'unclassified'))+'; recorded reset / interruption'
    for i,s in enumerate(pose):
        valid = bool(projection[i, 4])
        continuous = i == 0 or (s.segment == pose[i-1].segment and
            0 < s.source_ns-pose[i-1].source_ns <= gap*1e9)
        samples.append({'stamp_ns':str(s.source_ns), 'segment':s.segment, 'domain':s.domain, 't':(s.source_ns-origin)/1e9, 'recorded':boundary_time(s),
                        'd':float(projection[i, 0]) if valid else None,
                        'xy':s.value[:2], 'heading':float(s.value[2]), 'truth':aligned_truth[i,:2].tolist(), 'continuous':continuous})
    # Preserve recorded heading values; the renderer breaks lines at wrap crossings.
    values = {key: np.asarray(array, dtype=float).tolist() for key, array in values.items()}
    result = {'samples':samples, 'signals':values, 'laps':laps, 'reference':rows[:,1:3].tolist(),
              'map':map_data, 'length':length, 'source':data.metadata.get('config',{}).get('components',{}).get('localization',{}).get('type','unknown'),
              'value_alignment':'exact_recorded_timestamp', 'time_origin_ns':str(origin)}
    def clean(x):
        if isinstance(x, float) and not np.isfinite(x): return None
        if isinstance(x, list): return [clean(v) for v in x]
        if isinstance(x, dict): return {k:clean(v) for k,v in x.items()}
        return x
    return clean(result)


def preserved_payload(data, config):
    """Build the shared viewer without executing any analysis module."""
    from dataclasses import replace
    from ..core.signals import transform_world
    import yaml
    rows = None
    if 'raceline.csv' in data.assets:
        rows = np.loadtxt(data.assets['raceline.csv'], delimiter=';', comments='#', ndmin=2)
    truth = measurement_samples(data, 'truth')
    truth_map = []
    if truth and 'map_to_world.yaml' in data.assets:
        origin = yaml.safe_load(data.assets['map_to_world.yaml'].read_text())
        values = transform_world([s.value[:3] for s in truth], origin)
        truth_map = [replace(s, value=list(v)+s.value[3:], frame='map') for s, v in zip(truth, values)]
    return payload(data, rows, truth_map, config)


def render_report(result):
    from plotly.offline import get_plotlyjs
    esc = lambda value: html.escape(str(value))
    def format_value(value):
        if value is None:
            return 'Unavailable'
        if isinstance(value, float):
            return f'{value:.4g}'
        if isinstance(value, dict):
            return ' · '.join(k.replace('_', ' ')+': '+format_value(v) for k,v in value.items())
        return str(value)
    metrics = ''.join('<tr><td>'+esc(k.replace('_', ' '))+'</td><td>'+esc(format_value(v.get('value')))+'</td><td>'+esc(v.get('units') or '')+'</td><td>'+esc(v.get('reason',''))+'</td></tr>' for k,v in result.get('metrics',{}).items())
    viewer = {**result.get('viewer', {'laps':[]}), 'extensions': result.get('extensions', [])}
    data = json.dumps(viewer, allow_nan=False).replace('<', r'\u003c').replace('\u2028', r'\u2028').replace('\u2029', r'\u2029')
    script = Path(__file__).with_name('race_view.js').read_text()
    outcome = esc(result.get('metrics', {}).get('failure_classification', {}).get('value', 'Unavailable'))
    return ('<section id="race-performance"><style>'+
            '#race-performance .race-inspect{display:grid;grid-template-columns:minmax(300px,42%) minmax(0,1fr);gap:20px;align-items:start}'
            '#race-performance .race-map{position:sticky;top:16px;height:calc(90vh - 80px);min-height:500px;display:flex;flex-direction:column;gap:8px}'
            '#race-performance .race-controls{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:6px;align-items:center}'
            '#race-performance .race-controls button,#race-performance .race-controls select{min-width:0;height:36px;box-sizing:border-box;padding:8px 4px;font-size:12px;white-space:nowrap}'
            '#race-performance .race-controls .race-select{grid-column:span 2}'
            '#race-performance .race-controls .race-prev{grid-column:span 2}'
            '#race-performance .race-controls .race-next{grid-column:span 2}'
            '#race-performance .race-controls .race-unpin{grid-column:span 4}'
            '#race-performance .race-map svg{width:100%;flex:1;min-height:0;touch-action:none}'
            '#race-performance .race-readout{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));grid-template-rows:1fr;gap:8px;min-height:0;background:#f4f6fa;border-radius:8px;padding:14px;margin:12px 0}'
            '#race-performance .race-readout>div{min-width:0;overflow:hidden}'
            '#race-performance .race-readout strong{display:block;height:32px;line-height:16px;overflow:hidden;text-overflow:ellipsis;white-space:pre;font-variant-numeric:tabular-nums;font-size:13px}'
            '#race-performance .race-label{margin:0;min-height:24px}'
            '#race-performance .race-readout span{display:block;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:#687483;font-size:12px}'
            '#race-performance .race-panel{position:relative}'
            '#race-performance .race-cursor{position:absolute;top:20px;bottom:135px;border-left:1px dashed #334155;pointer-events:none;display:none}'
            '#race-performance .race-map-legend{display:flex;align-items:center;flex-wrap:wrap;gap:10px 16px;padding:10px 0;border-top:1px solid #dce2e9;font-size:12px}'
            '#race-performance .race-map-legend strong{color:#526175;text-transform:uppercase;font-size:10px;letter-spacing:.06em}'
            '#race-performance .race-map-legend span{display:inline-flex;align-items:center;gap:6px}'
            '#race-performance .race-map-legend i{display:inline-block;width:24px}'
            '#race-performance .race-subpanel{margin-left:16px;padding-left:12px;border-left:2px solid #e8edf3}'
            '#race-performance .race-subpanel>summary{font-size:14px;color:#526175}'
            '#race-performance .race-panels>details>summary{font-weight:650}'
            '#race-performance .race-laps tr[tabindex]{cursor:pointer;transition:background .15s}'
            '#race-performance .race-laps tr[tabindex]:hover{background:#eef4ff!important}'
            '#race-performance .race-laps tr[tabindex]:focus-visible{outline:2px solid #0671ee;outline-offset:-2px}'
            '#race-performance .race-values{position:absolute;inset:0;pointer-events:none;overflow:hidden}'
            '#race-performance .race-intersection{position:absolute;width:7px;height:7px;border:1px solid white;border-radius:50%;transform:translate(-50%,-50%)}'
            '#race-performance .race-value{box-sizing:border-box;max-width:210px;padding:3px 6px;background:rgba(255,255,255,.96);border-left:3px solid;font-size:10px;line-height:16px;border-radius:3px;white-space:nowrap;text-overflow:ellipsis;overflow:hidden;box-shadow:0 1px 4px #17243818}'
            '#race-performance .race-value-list{position:absolute;max-width:210px;overflow:auto;pointer-events:auto;background:white;box-shadow:0 2px 8px #17243822;border-radius:5px}'
            '@media(max-width:850px){#race-performance .race-inspect{display:block}#race-performance .race-map{position:relative;height:800px;max-height:90vh}}'
            '</style><h2>Race Performance</h2><p>Race outcome: '+outcome+'</p><details><summary>Race summary and metrics</summary><table>'+metrics+
            '</table></details><div class="race-laps"></div><h3>Lap Data</h3>'
            '<details><summary>Time basis and alignment</summary><p>Selected time: localized-pose header timestamp, seconds from '+esc(result.get('viewer',{}).get('time_origin_ns','unavailable'))+
            ' ns. Sensor stamps represent bridge arrival. /clock measures host-monotonic elapsed time, not simulator physics time. Selectable positions are recorded localized poses. Telemetry values require an exact timestamp match in the same clock segment; unmatched values are unavailable. Reference curves and tracking errors are geometric calculations from those samples. Lines break at timestamp gaps, unavailable values, and wraparound; dots show isolated samples. Pointer selection snaps to the nearest recorded sample; arrow navigation steps between recorded samples. Reference progress is measured from the first saved raceline point (0% / 100%), not the simulator lap-counter boundary. The lap-start marker shows the first recorded pose in the selected lap; sampling and counter delivery can place it away from that reference origin. Ground truth is evaluation only. Odometry heading requires validated map-frame TF alignment.</p></details>'
            '<div class="race-inspect"><div class="race-map"><div class="race-controls"><button class="race-prev">Previous lap</button><select class="race-select" aria-label="Select lap"></select><button class="race-next">Next lap</button><button class="race-back" aria-label="Previous telemetry sample">←</button><button class="race-forward" aria-label="Next telemetry sample">→</button><button class="race-unpin">Unpin</button></div><label>Track overlay <select class="race-overlay" aria-label="Track overlay"><option value="">Raceline + localized position + ground truth</option></select></label><p class="race-label"></p><div class="race-readout" aria-live="polite"></div><details class="race-extension-readout" hidden style="max-height:240px;overflow:auto;font-size:12px"><summary>Selected observation details</summary><div class="race-extension-values"></div></details><div class="race-track" style="display:flex;flex:1;min-height:0;flex-direction:column"></div></div><div class="race-panels"></div></div><script>'+get_plotlyjs()+'</script><script>\n(()=>{const D='+data+';\n'+script+'\n})();</script></section>')
