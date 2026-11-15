"""Explicit provenance states, including compatibility with legacy collections."""


def git_metadata(source):
    result = dict(source or {})
    result.setdefault('commit', None)
    result.setdefault('dirty', None)
    branch = result.get('branch')
    if branch == 'HEAD':
        result.update(branch=None, branch_state='detached')
    elif branch:
        result.setdefault('branch_state', 'attached')
    else:
        result.setdefault('branch', None)
        result.setdefault('branch_state', 'unrecorded' if result['commit'] else 'unavailable')
    return result


def initial_pose_metadata(config):
    """Native reset exposes no numeric spawn transform; never infer one from telemetry."""
    requested = config.get('experiment', {}).get('initial_pose')
    return {'explicitly_supplied': requested is not None, 'requested_pose': requested,
            'resolution': 'unresolved', 'resolved_pose': None, 'frame': None,
            'source': 'native simulator reset interface',
            'reason': 'Reset requests carry only a Boolean. The simulator scene spawn/reset transform '
                      'is not exported in collection metadata. Localizer initialization and observed '
                      'poses do not establish the configured simulator spawn pose.'}
