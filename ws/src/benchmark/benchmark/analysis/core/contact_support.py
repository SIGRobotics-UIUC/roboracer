"""Conditional static-support diagnostics, not validated tire-load estimates."""
import math
from .wheel_contact import finite


def vector(value):
    if not isinstance(value, dict) or not all(finite(value.get(k)) for k in ('x', 'y', 'z')):
        return None
    return tuple(value[k] for k in ('x', 'y', 'z'))


def static_support_observation(contact):
    """Retain ambiguity: absent/stale collision data never means zero support.

    A candidate impulse/time-step projection requires the same *observed* Unity
    fixed time, matching step duration, and native collider/body identity. This
    is not proof of solver alignment, impulse sign, or WheelHit.force == Fz.
    """
    result = {'status': 'conditional_diagnostic_only', 'raw_force_sum': None,
              'conditional_mg_N': None, 'conditional_missing_support_N': None,
              'candidate_nonwheel_impulse_per_step_N': None,
              'conditional_residual_N': None, 'events': [], 'reasons': [],
              'physical_load_interpretation_accepted': False}
    snapshot = contact.get('runtime_snapshot', {})
    configuration = snapshot.get('configuration', {})
    gravity = vector(configuration.get('gravity'))
    mass = configuration.get('mass')
    up, mg = None, None
    if gravity is not None and finite(mass) and mass > 0:
        g = math.hypot(*gravity)
        if g > 0 and math.isfinite(g):
            up = tuple(-x/g for x in gravity)
            mg = mass*g
            if math.isfinite(mg):
                result['conditional_mg_N'] = mg
    wheels = contact.get('wheels', [])
    if (len(wheels) == 4 and all(w.get('grounded') is True and w.get('valid') is True
            and isinstance(w.get('wheel_hit'), dict) and finite(w['wheel_hit'].get('force')) for w in wheels)):
        total = sum(w['wheel_hit']['force'] for w in wheels)
        if math.isfinite(total):
            result['raw_force_sum'] = total
    if result['raw_force_sum'] is not None and result['conditional_mg_N'] is not None:
        result['conditional_missing_support_N'] = result['conditional_mg_N']-result['raw_force_sum']
    diagnostics = snapshot.get('collision_diagnostics')
    if not isinstance(diagnostics, dict):
        result['reasons'].append('collision diagnostics unavailable')
        return result
    if snapshot.get('cosimulation_component', {}).get('active') is True:
        result['reasons'].append('co-simulation component active; effects unaccounted')
    if contact.get('runtime_snapshot_valid') is not True:
        result['reasons'].append('runtime snapshot not valid')
    sleep_state = diagnostics.get('sleep_state_runtime_invoke', {})
    preferred_sleep = (sleep_state['value'] if isinstance(sleep_state, dict)
                       and sleep_state.get('available') is True and type(sleep_state.get('value')) is bool
                       else diagnostics.get('body_is_sleeping'))
    byte_state = diagnostics.get('sleep_state_icall_byte', {})
    if isinstance(byte_state, dict) and byte_state.get('available') is True:
        raw = byte_state.get('raw_byte')
        if (type(raw) is int and raw in (0, 1) and type(byte_state.get('value')) is bool
                and byte_state['value'] == bool(raw)):
            preferred_sleep = byte_state['value']
        else:
            preferred_sleep = None
            result['reasons'].append('inconsistent native sleep byte')
    result['sleep_state_used'] = preferred_sleep
    result['sleep_readings_disagree'] = (type(preferred_sleep) is bool
        and type(diagnostics.get('body_is_sleeping')) is bool
        and preferred_sleep != diagnostics['body_is_sleeping'])
    if preferred_sleep is not False:
        result['reasons'].append('body sleeping or sleep state unavailable')
    if diagnostics.get('observer_enabled') is not True or diagnostics.get('last_sampling_error'):
        result['reasons'].append('collision observer unavailable/error')
    if diagnostics.get('inventory_truncated') is not False:
        result['reasons'].append('collider inventory incomplete')
    body_id = diagnostics.get('body_id')
    colliders = {c.get('instance_id'): c for c in diagnostics.get('colliders', []) if isinstance(c, dict)}
    wheel_ids = {w.get('collider_id') for w in wheels}
    source = contact.get('source', {})
    step, duration = source.get('observed_fixed_time'), source.get('fixedDeltaTime')
    if (not finite(step) or not finite(duration) or duration <= 0
            or diagnostics.get('observed_fixed_time') != step or type(body_id) is not int):
        result['reasons'].append('observed step/body identity unavailable or inconsistent')
    events = diagnostics.get('events', [])
    if not isinstance(events, list) or not events:
        result['reasons'].append('no collision events observed; no zero-support inference')
        return result
    ids = [e.get('other_collider_id') for e in events if isinstance(e, dict)]
    if len(ids) != len(set(ids)):
        result['reasons'].append('duplicate collision aggregate identity')
    candidates = []
    for event in events:
        reasons = []
        if not isinstance(event, dict):
            result['events'].append({'eligible': False, 'reasons': ['malformed event']})
            continue
        if event.get('phase') not in ('OnCollisionEnter', 'OnCollisionStay'):
            reasons.append('not a continuing/entering contact event')
        if not finite(step) or event.get('observed_fixed_time') != step:
            reasons.append('stale/different observed fixed time')
        if not finite(duration) or duration <= 0 or event.get('fixedDeltaTime') != duration:
            reasons.append('time step unavailable/mismatched')
        contacts = event.get('contacts', [])
        if (not isinstance(contacts, list) or not contacts or event.get('contacts_truncated') is not False
                or event.get('contact_count') != len(contacts)):
            reasons.append('contact inventory missing/truncated/inconsistent')
        else:
            for point in contacts:
                collider = colliders.get(point.get('this_collider_id'))
                if (collider is None or collider.get('body_id') != body_id
                        or collider.get('enabled') is not True or collider.get('active') is not True
                        or collider.get('isTrigger') is not False):
                    reasons.append('contact collider identity/body/state unverified')
                elif (collider.get('instance_id') in wheel_ids
                        or collider.get('type') == 'UnityEngine.WheelCollider'
                        or not isinstance(collider.get('type'), str)):
                    reasons.append('wheel/unknown collider; possible double counting')
                if point.get('other_collider_id') != event.get('other_collider_id'):
                    reasons.append('other-collider identity mismatch')
        impulse = vector(event.get('impulse_world'))
        contribution = None
        if impulse is None or up is None:
            reasons.append('impulse/gravity unavailable')
        if not reasons and not result['reasons']:
            contribution = sum(x*y for x, y in zip(up, impulse))/duration
            if not math.isfinite(contribution):
                reasons.append('nonfinite impulse projection')
                contribution = None
            else:
                candidates.append(contribution)
        result['events'].append({'other_collider_id': event.get('other_collider_id'),
                                  'eligible': contribution is not None,
                                  'candidate_vertical_impulse_per_step_N': contribution,
                                  'reasons': sorted(set(reasons))})
    if not result['reasons'] and len(candidates) == len(events):
        total = sum(candidates)
        if math.isfinite(total):
            result['candidate_nonwheel_impulse_per_step_N'] = total
            if result['conditional_missing_support_N'] is not None:
                result['conditional_residual_N'] = result['conditional_missing_support_N']-total
    return result
