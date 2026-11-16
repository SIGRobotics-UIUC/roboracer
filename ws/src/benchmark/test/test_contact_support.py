from copy import deepcopy
import pytest
from benchmark.analysis.core.contact_support import static_support_observation


@pytest.fixture
def contact():
    return {'runtime_snapshot_valid': True, 'source': {'observed_fixed_time': 1., 'fixedDeltaTime': .001},
            'runtime_snapshot': {'configuration': {'mass': 3.4, 'gravity': {'x': 0., 'y': -10., 'z': 0.}},
                'collision_diagnostics': {'body_id': 1, 'observer_enabled': True, 'body_is_sleeping': False,
                    'observed_fixed_time': 1., 'inventory_truncated': False, 'last_sampling_error': None,
                    'colliders': [{'instance_id': 10, 'body_id': 1, 'enabled': True, 'active': True,
                                   'isTrigger': False, 'type': 'UnityEngine.MeshCollider'}],
                    'events': [{'other_collider_id': 20, 'phase': 'OnCollisionStay', 'observed_fixed_time': 1.,
                                'fixedDeltaTime': .001, 'contact_count': 1, 'contacts_truncated': False,
                                'contacts': [{'this_collider_id': 10, 'other_collider_id': 20}],
                                'impulse_world': {'x': 0., 'y': .004, 'z': 0.}}]}},
            'wheels': [{'collider_id': i+100, 'grounded': True, 'valid': True, 'wheel_hit': {'force': 7.5}} for i in range(4)]}


def test_synthetic_impulse_can_close_conditional_gap_without_validating_load(contact):
    result = static_support_observation(contact)
    assert result['raw_force_sum'] == 30
    assert result['conditional_mg_N'] == 34
    assert result['candidate_nonwheel_impulse_per_step_N'] == 4
    assert result['conditional_residual_N'] == 0
    assert not result['physical_load_interpretation_accepted']


@pytest.mark.parametrize('change', ['absent', 'sleep', 'stale', 'step', 'truncated', 'wheel', 'body',
                                  'other', 'nan', 'duplicate', 'no_events', 'exit', 'disabled', 'snapshot', 'cosim'])
def test_ambiguous_collision_data_never_becomes_zero_support(contact, change):
    d=contact['runtime_snapshot']['collision_diagnostics'];e=d['events'][0]
    if change == 'absent': del contact['runtime_snapshot']['collision_diagnostics']
    if change == 'sleep': d['body_is_sleeping'] = True
    if change == 'stale': e['observed_fixed_time'] = .999
    if change == 'step': e['fixedDeltaTime'] = .002
    if change == 'truncated': e['contacts_truncated'] = True
    if change == 'wheel': d['colliders'][0]['type'] = 'UnityEngine.WheelCollider'
    if change == 'body': d['colliders'][0]['body_id'] = 2
    if change == 'other': e['contacts'][0]['other_collider_id'] = 21
    if change == 'nan': e['impulse_world']['y'] = {'nonfinite': 'NaN'}
    if change == 'duplicate': d['events'].append(deepcopy(e))
    if change == 'no_events': d['events'] = []
    if change == 'exit': e['phase'] = 'OnCollisionExit'
    if change == 'disabled': d['observer_enabled'] = False
    if change == 'snapshot': contact['runtime_snapshot_valid'] = False
    if change == 'cosim': contact['runtime_snapshot']['cosimulation_component'] = {'active': True}
    result = static_support_observation(contact)
    assert result['candidate_nonwheel_impulse_per_step_N'] is None
    assert result['conditional_residual_N'] is None
    assert result['conditional_missing_support_N'] == 4


def test_negative_impulse_preserved_not_clipped(contact):
    contact['runtime_snapshot']['collision_diagnostics']['events'][0]['impulse_world']['y']=-.004
    assert static_support_observation(contact)['candidate_nonwheel_impulse_per_step_N'] == -4


def test_independent_runtime_boolean_is_retained_and_preferred_for_candidate_only(contact):
    d=contact['runtime_snapshot']['collision_diagnostics']
    d['body_is_sleeping']=True
    d['sleep_state_runtime_invoke']={'available':True,'value':False}
    result=static_support_observation(contact)
    assert result['sleep_readings_disagree']
    assert result['candidate_nonwheel_impulse_per_step_N']==4
    assert not result['physical_load_interpretation_accepted']
    d['body_is_sleeping']=False
    d['sleep_state_runtime_invoke']['value']=True
    assert static_support_observation(contact)['candidate_nonwheel_impulse_per_step_N'] is None


def test_explicit_native_sleep_byte_overrides_known_unreliable_generated_bool(contact):
    d=contact['runtime_snapshot']['collision_diagnostics']
    d['body_is_sleeping']=True
    d['sleep_state_icall_byte']={'available':True,'value':False,'raw_byte':0}
    result=static_support_observation(contact)
    assert result['sleep_state_used'] is False
    assert result['sleep_readings_disagree']
    assert result['candidate_nonwheel_impulse_per_step_N']==4
    assert not result['physical_load_interpretation_accepted']


@pytest.mark.parametrize('raw,value',[(2,True),(True,True),(1,False),(0,True)])
def test_inconsistent_sleep_byte_does_not_create_candidate_support(contact,raw,value):
    contact['runtime_snapshot']['collision_diagnostics']['sleep_state_icall_byte']={'available':True,'value':value,'raw_byte':raw}
    result=static_support_observation(contact)
    assert result['candidate_nonwheel_impulse_per_step_N'] is None
    assert 'inconsistent native sleep byte' in result['reasons']
