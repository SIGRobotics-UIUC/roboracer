#!/usr/bin/env python3
"""Read-only extraction of WheelCollider settings from an AutoDRIVE Unity scene.

Requires UnityPy installed separately. Pass the installed simulator's Data/level0.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('level0', type=Path, help='Installed simulator Data/level0 scene')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        import UnityPy
    except ImportError as exc:
        parser.error(f'UnityPy is needed to parse the asset: {exc}')
    env = UnityPy.load(str(args.level0))
    script_asset = args.level0.parent / 'globalgamemanagers.assets'
    script_env = UnityPy.load(str(script_asset))
    script_names = {obj.path_id: obj.read_typetree().get('m_ClassName', '')
                    for obj in script_env.objects if obj.type.name == 'MonoScript'}
    objects = {obj.path_id: obj for obj in env.objects}
    game_names = {pid: obj.read_typetree().get('m_Name', '')
                  for pid, obj in objects.items() if obj.type.name == 'GameObject'}
    wheels = []
    for pid, obj in objects.items():
        if obj.type.name != 'WheelCollider':
            continue
        data = obj.read_typetree()
        game_id = data['m_GameObject']['m_PathID']
        forward = data['m_ForwardFriction']
        sideways = data['m_SidewaysFriction']
        def curve(raw):
            return dict(extremum_slip=raw['m_ExtremumSlip'],
                        extremum_value=raw['m_ExtremumValue'],
                        asymptote_slip=raw['m_AsymptoteSlip'],
                        asymptote_value=raw['m_AsymptoteValue'],
                        stiffness=raw['m_Stiffness'],
                        effective_extremum_value=raw['m_ExtremumValue'] * raw['m_Stiffness'],
                        effective_asymptote_value=raw['m_AsymptoteValue'] * raw['m_Stiffness'])
        wheels.append(dict(path_id=pid, game_object_id=game_id,
                           name=game_names.get(game_id), enabled=data['m_Enabled'],
                           radius_m=data['m_Radius'], forward=curve(forward),
                           sideways=curve(sideways)))
    wheels.sort(key=lambda wheel: wheel['name'] or '')
    controller_refs = []
    for pid, obj in objects.items():
        if obj.type.name != 'MonoBehaviour':
            continue
        raw = obj.get_raw_data()
        if len(raw) < 28:
            continue
        game_id = struct.unpack_from('<q', raw, 4)[0]
        if game_names.get(game_id) != 'Roboracer':
            continue
        script_id = struct.unpack_from('<q', raw, 20)[0]
        refs = [wheel['path_id'] for wheel in wheels
                if struct.pack('<q', wheel['path_id']) in raw]
        if len(refs) == len(wheels):
            controller_refs.append(dict(component_path_id=pid,
                                        script=script_names.get(script_id), wheel_path_ids=refs))
    report = dict(scene_file=args.level0.name, scene_sha256=sha256(args.level0),
                  wheels=wheels, roboracer_components_referencing_all_wheels=controller_refs,
                  limitation='Serialized scene defaults; runtime code may change friction settings.')
    rendered = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end='')


if __name__ == '__main__':
    main()
