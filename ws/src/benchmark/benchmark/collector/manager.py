"""Lightweight benchmark manager; separate processes own driving and bag writes."""
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import time
import yaml

from .artifacts import finalize, git_snapshot, unique_directory, utc, write_json
from .adapters import controller_parameters, managed_commands
from .lifecycle import Health, RestConfirmation
from .process import Process
from .topics import BASE, NS, REQUESTED, profile, stamp_ns, subscription_qos, valid


class Abort(RuntimeError):
    pass


def run(config, args, *, on_finalized=None):
    from .progress import say, lap_message, result_message
    from .metadata import initial_pose_metadata
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from ament_index_python.packages import get_package_share_directory

    args.log_root = args.log_root.resolve()
    args.tracks_dir = args.tracks_dir.resolve()
    if args.log_root == args.tracks_dir or args.tracks_dir in args.log_root.parents:
        raise ValueError('Collection output must be outside tracks')
    args.log_root.mkdir(parents=True, exist_ok=True)
    # Cross-process lock lives outside immutable raw runs, on this ROS domain.
    singleton = (args.log_root/f'.benchmark-domain-{os.environ.get("ROS_DOMAIN_ID", "0")}.lock').open('a+')
    try:
        fcntl.flock(singleton, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        singleton.close()
        raise ValueError('A manager is already active on this domain') from None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('benchmark_manager', parameter_overrides=[Parameter('use_sim_time', value=True)])
    topics, essential = profile(config)
    limits = config['watchdogs']
    unknown_limits = set(limits['signals'])-essential
    if unknown_limits:
        node.destroy_node(); rclpy.try_shutdown(); singleton.close()
        raise ValueError('Unknown essential-signal timeout keys: '+','.join(sorted(unknown_limits)))
    health = Health(essential, limits['loss_s'], limits['signals'])
    latest, stamps, capture_seen, capture_counts, subs = {}, {}, {}, {}, []
    state = {'status': 'preflight', 'stop_requested': False}
    metadata = {'schema_version': 1, 'created_utc': utc(), 'config': config,
                'clock_domain': 'transport', 'recorder_clock_domain': 'transport',
                'receipt_monotonic_domain': 'collector_container_monotonic',
                'clock_provenance': 'fast_bridge SteadyEpoch: host monotonic + fixed epoch; NOT simulator physics time',
                'initial_pose_metadata': initial_pose_metadata(config),
                'requested_topic': REQUESTED, 'initial_pose': 'native simulator start; no numerical pose supplied',
                'measurement': {'laps': [], 'completed_laps': 0, 'started': False},
                'ground_truth_review_debt': 'Existing localization_gate uses ground truth/map origin; '
                    'bridge automatic respawn events use restricted telemetry. No new ground-truth feedback is added.',
                'simulator_metadata': config['metadata'], 'commands': [], 'process_results': {}}
    run_path = None
    finalized = False
    processes = {}
    collection_lock = None
    measuring = False
    stop_started = False
    event_stream = None
    pending, lap0 = None, None
    warmup = config['experiment']['warmup_laps']
    enable = node.create_publisher(Bool, '/benchmark/enable', 10)
    reset = node.create_publisher(Bool, '/autodrive/reset_command', 10)
    events = node.create_publisher(String, '/benchmark/events', 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_pub = node.create_publisher(String, '/benchmark/status', latched)

    def event(kind, **fields):
        messages = {
            'reset_requested': 'Requesting simulator reset; waiting for its reset event.',
            'measurement_start': 'Warm-up finished. Measurement lap 1 is starting; warm-up laps are excluded.',
            'motion_inhibit': 'Stopping: command gate inhibited; sending stops and checking wheel odometry.',
            'stop_procedure_complete': 'Stop procedure finished: '+str(fields.get('confirmation', 'unknown')),
            'abort': 'Run interrupted: '+str(fields.get('reason', 'unknown')),
        }
        if kind in messages:
            say(messages[kind])
        record = {'event': kind, 'utc': utc(), 'monotonic_ns': time.monotonic_ns(),
                  'transport_ns': node.get_clock().now().nanoseconds, **fields}
        events.publish(String(data=json.dumps(record, allow_nan=False)))
        if event_stream:
            try:
                event_stream.write(json.dumps(record, allow_nan=False)+'\n')
                event_stream.flush()
            except OSError as exc:
                state['fault'] = 'lifecycle_event_write_failure'
                metadata['event_write_error'] = str(exc)

    def stop_service(request, response):
        state['stop_requested'] = True
        response.success, response.message = True, 'Stop requested; gate inhibition precedes collection finalization'
        return response
    service = node.create_service(Trigger, '/benchmark/stop', stop_service)

    def signal_stop(signum, frame):
        state['stop_requested'] = True
    previous_signals = {sig: signal.signal(sig, signal_stop) for sig in (signal.SIGINT, signal.SIGTERM)}

    def observed(topic, message):
        nonlocal measuring
        now = time.monotonic()
        if not valid(topic, message):
            return
        stamp = stamp_ns(message)
        if stamp is not None:
            clock_now = node.get_clock().now().nanoseconds
            if stamp <= stamps.get(topic, 0):
                if measuring and stamp < stamps.get(topic, 0)-100_000_000:
                    state['fault'] = 'clock_or_source_discontinuity:'+topic
                return
            if topic != '/clock' and not -100_000_000 <= clock_now-stamp <= limits['signals'].get(topic, limits['loss_s'])*1e9:
                return
            stamps[topic] = stamp
        previous = latest.get(topic)
        latest[topic] = (now, message)
        health.observe(topic, now)
        if measuring and topic == NS+'sim_reset':
            state['fault'] = 'unexpected_sim_reset'
        if measuring and topic in (NS+'lap_count', NS+'collision_count') and previous:
            if message.data < previous[1].data:
                state['fault'] = 'counter_regression:'+topic

    def receive_status(kind, message):
        try:
            value = json.loads(message.data)
            latest[kind] = (time.monotonic(), value)
            if kind == 'recorder':
                if value.get('error'):
                    state['fault'] = 'recording_failure:'+str(value['error'])
                for topic, count in value['counts'].items():
                    if count > capture_counts.get(topic, 0):
                        capture_seen[topic] = time.monotonic()
                    capture_counts[topic] = count
        except (ValueError, KeyError):
            state['fault'] = 'invalid_'+kind+'_status'

    for topic, kind in {**topics, NS+'sim_reset': 'std_msgs/msg/String'}.items():
        # Avoid recursively subscribing to our own full status/events payloads.
        if topic in ('/benchmark/status', '/benchmark/events') or topic not in essential | {NS+'sim_reset'}:
            continue
        subs.append(node.create_subscription(get_message(kind), topic,
                    lambda msg, name=topic: observed(name, msg), subscription_qos(node, topic)))
    subs += [node.create_subscription(String, '/benchmark/'+kind+'_status',
             lambda msg, name=kind: receive_status(name, msg), 10) for kind in ('gate', 'recorder')]

    last_status = 0.0
    def spin(seconds=0.05, enabled=False):
        nonlocal last_status
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if 'gate' in processes:
                enable.publish(Bool(data=enabled and not state['stop_requested'] and not state.get('fault')))
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0., deadline-time.monotonic())))
            if time.monotonic()-last_status > 0.2:
                status_pub.publish(String(data=json.dumps({**state, 'run_id': run_path.name if run_path else None,
                                                          'utc': utc()})))
                last_status = time.monotonic()

    def launch(name, command):
        say(f'Starting {name}; process log: {run_path / (name+".log")}')
        env = {**os.environ, 'TRACKS_DIR': str(args.tracks_dir)}
        processes[name] = Process(command, run_path/(name+'.log'), env)
        metadata['commands'].append({'role': name, 'argv': command})
        return processes[name]

    def check(enabled=False):
        if state['stop_requested']:
            raise Abort('operator_stop')
        if state.get('fault'):
            raise Abort(state['fault'])
        for name, process in processes.items():
            if not process.alive():
                raise Abort('process_exit:'+name)
        gate = latest.get('gate')
        if gate and gate[1]['fault']:
            raise Abort('command_gate:'+gate[1]['fault'])
        if config['logging']['level'] != 'off' and 'recorder' in processes:
            recorder = latest.get('recorder')
            if recorder and recorder[1]['error']:
                raise Abort('recording_failure:'+recorder[1]['error'])
            if enabled and (not recorder or time.monotonic()-recorder[0] > limits['heartbeat_s']):
                raise Abort('recorder_heartbeat_lost')
        if shutil.disk_usage(run_path).free < limits['disk_reserve_mb']*1024**2:
            raise Abort('disk_reserve_exhausted')
        if enabled:
            missing = health.missing(time.monotonic())
            if missing:
                raise Abort('essential_signal_loss:'+','.join(missing))
            if not gate or time.monotonic()-gate[0] > limits['heartbeat_s']:
                raise Abort('gate_heartbeat_lost')
            if config['logging']['level'] != 'off':
                missing = [topic for topic in essential if time.monotonic()-capture_seen.get(topic, -math.inf)
                           > limits['signals'].get(topic, limits['loss_s'])]
                if missing:
                    raise Abort('essential_capture_loss:'+','.join(sorted(missing)))
        owners = node.get_publishers_info_by_topic('/drive')
        unknown = any(owner.node_name == '_NODE_NAME_UNKNOWN_' for owner in owners)
        # DDS may discover endpoints before their participant identity. Keep
        # startup inhibited until names resolve; never enable on unknown owners.
        if (enabled or not unknown) and (any(owner.node_name != 'benchmark_gate' for owner in owners) or len(owners) > 1):
            raise Abort('competing_drive_publisher:'+','.join(owner.node_name for owner in owners))
        if len(node.get_publishers_info_by_topic(REQUESTED)) > 1:
            raise Abort('multiple_controller_request_publishers')

    def wait(predicate, label):
        say(f'Waiting for {label.replace("_", " ") } (up to {limits["startup_s"]:g} s).')
        deadline = time.monotonic()+limits['startup_s']
        next_update = time.monotonic()+5
        while time.monotonic() < deadline:
            spin()
            check()
            if predicate():
                say(f'Ready: {label.replace("_", " ")}.')
                return
            if time.monotonic() >= next_update:
                missing = sorted(health.missing(time.monotonic()))
                say('Still waiting for '+label.replace('_', ' ')+
                    (('; missing inputs: '+', '.join(missing)) if missing else '.'))
                next_update = time.monotonic()+5
        raise Abort('readiness_timeout:'+label)

    def finish_pending_lap():
        nonlocal pending
        if pending is None or time.monotonic()-pending['arrival'] < 0.25:
            return
        index = pending['counter']-lap0
        lap_time = latest[NS+'last_lap_time']
        finite_time = math.isfinite(lap_time[1].data)
        duration = float(lap_time[1].data) if finite_time else None
        valid_lap = (pending['arrival']-0.1 <= lap_time[0] <= pending['arrival']+0.35
                     and finite_time and lap_time[1].data > 0)
        if index > warmup:
            metadata['measurement']['laps'].append({'index': index-warmup, 'time_s': duration,
                'start_kind': 'initial_partial_or_standing' if index == 1 else 'flying',
                'valid': valid_lap, 'association': 'native counter/time associated after 0.25 s; no common frame ID'})
        say(lap_message(index, warmup, config['experiment']['laps'], duration, valid_lap))
        event('lap_observed', native_count=pending['counter'], time_s=duration, valid=valid_lap)
        pending = None

    def stop_vehicle():
        nonlocal stop_started, measuring
        stop_started = True
        measuring = False
        state['status'] = 'stopping'
        metadata['measurement'].setdefault('end_recorded_ns', node.get_clock().now().nanoseconds)
        metadata['measurement'].setdefault('end_monotonic_ns', time.monotonic_ns())
        if NS+'collision_count' in latest:
            metadata['measurement']['end_collision_count'] = latest[NS+'collision_count'][1].data
        event('motion_inhibit')
        enable.publish(Bool(data=False))
        for name in ('controller', 'localization'):
            if name in processes:
                processes[name].signal()
        confirm = RestConfirmation(limits['rest_speed_mps'], limits['rest_dwell_s'],
                                   limits['stop_hold_s'], limits['stop_timeout_s'])
        began = time.monotonic()
        while True:
            spin()
            finish_pending_lap()
            now = time.monotonic()
            odom = latest.get('/odom')
            result = confirm.update(now-began, now, odom[1].twist.twist.linear.x if odom else None,
                                    odom[0] if odom else None, limits['signals'].get('/odom', limits['loss_s']))
            gate = latest.get('gate')
            gate_confirmed = gate and now-gate[0] < limits['heartbeat_s'] and not gate[1]['enabled']
            owners = node.get_publishers_info_by_topic('/drive')
            sole = len(owners) == 1 and owners[0].node_name == 'benchmark_gate'
            if result == 'confirmed' and gate_confirmed and sole:
                metadata['stop_confirmation'] = 'wheel_odom_rest_and_gate_inhibited'
                break
            if now-began >= limits['stop_timeout_s']:
                metadata['stop_confirmation'] = 'unconfirmed'
                break
        for name in ('controller', 'localization'):
            if name in processes:
                metadata['process_results'][name] = processes[name].close(spin)
                if metadata['process_results'][name]['group_alive']:
                    metadata['stop_confirmation'] = 'unconfirmed'
        event('stop_procedure_complete', confirmation=metadata['stop_confirmation'])
        metadata['stop_mode'] = latest.get('gate', (None, {}))[1].get('stop_mode', 'unknown')

    try:
        say(f'Setting up race on {config["experiment"]["map"]}: '
            f'{warmup} warm-up laps (excluded), {config["experiment"]["laps"]} measured laps.')
        say('Preflight: checking existing publishers, bridge clock and saved track assets.')
        # Allow discovery before touching reset or command ownership.
        spin(1.0)
        if node.get_publishers_info_by_topic('/drive'):
            raise Abort('preflight_existing_drive_publisher')
        names = node.get_node_names()
        blocked = {'cartographer_node', 'localization_gate', 'mcl', 'pure_pursuit_node', 'map_node',
                   'benchmark_gate', 'benchmark_recorder'}
        if config['experiment']['mode'] == 'managed' and blocked.intersection(names):
            raise Abort('preflight_existing_algorithm_nodes:'+','.join(sorted(blocked.intersection(names))))
        if config['experiment']['mode'] == 'managed' and any(node.get_publishers_info_by_topic(topic)
                                                            for topic in ('/localized_pose', '/tracked_pose')):
            raise Abort('preflight_existing_localization_publisher')
        if names.count('benchmark_manager') > 1:
            raise Abort('duplicate_manager')
        # Verify the known transport-clock source rather than trusting the topic name.
        client = node.create_client(GetParameters, '/fast_bridge/get_parameters')
        if not client.wait_for_service(timeout_sec=3.0):
            raise Abort('fast_bridge_parameter_service_unavailable')
        future = client.call_async(GetParameters.Request(names=['publish_clock', 'use_sim_time', 'publish_restricted_topics']))
        deadline = time.monotonic()+3.0
        while not future.done() and time.monotonic() < deadline:
            spin()
        if (not future.done() or len(future.result().values) != 3
                or not all(v.type == 1 and v.bool_value for v in future.result().values)):
            raise Abort('requires_native_transport_clock_and_restricted_lap_telemetry')
        clock_owners = node.get_publishers_info_by_topic('/clock')
        if len(clock_owners) != 1 or clock_owners[0].node_name != 'fast_bridge':
            raise Abort('unverified_clock_owner')
        track = args.tracks_dir/config['experiment']['map']
        required = ['raceline.csv', 'map.yaml']
        localizer = config['components'].get('localization', {}).get('type')
        if localizer == 'cartographer':
            required.append('map.pbstream')
        if any(not (track/name).is_file() for name in required):
            raise Abort('missing_track_assets:'+','.join(required))
        run_path = unique_directory(args.log_root/'runs', timestamp_prefix=args.run_id)
        collection_lock = (run_path/'collection.lock').open('x')
        fcntl.flock(collection_lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        event_stream = (run_path/'events.jsonl').open('x')
        say(f'Run ID: {run_path.name}')
        say(f'Raw collection directory (sealed on completion): {run_path}')
        say(f'Lifecycle event log: {run_path / "events.jsonl"}')
        say('Saving source and track/configuration snapshots before starting motion.')
        metadata['run_id'] = run_path.name
        metadata['git'] = git_snapshot(Path(os.environ.get('BENCHMARK_REPO_ROOT', Path.cwd())), run_path/'source')
        metadata['reproducibility_warnings'] = ([] if metadata['git'].get('commit') else
            ['Git/source snapshot unavailable; set BENCHMARK_REPO_ROOT to a readable checkout'])
        metadata['requested_overrides'] = args.set
        shutil.copyfile(args.config, run_path/'requested.yaml')
        assets = run_path/'assets'
        assets.mkdir()
        for name in ('raceline.csv', 'centerline.csv', 'map_to_world.yaml', 'map.pbstream'):
            if (track/name).is_file():
                shutil.copyfile(track/name, assets/name)
        original_map = yaml.safe_load((track/'map.yaml').read_text())
        shutil.copyfile(track/'map.yaml', assets/'source_map.yaml')
        image = Path(original_map['image'])
        if not image.is_absolute():
            image = track/image
        shutil.copyfile(image, assets/'map.pgm')
        original_map['image'] = 'map.pgm'
        (assets/'map.yaml').write_text(yaml.safe_dump(original_map))
        control_share = Path(get_package_share_directory('control'))
        for package in ('control', 'perception', 'autodrive_bridge', 'planning'):
            source = Path(get_package_share_directory(package))/'config'
            if source.is_dir():
                shutil.copytree(source, assets/package)
        shutil.copyfile(control_share/'config'/'vehicle.yaml', assets/'vehicle.yaml')
        vehicle = yaml.safe_load((assets/'vehicle.yaml').read_text())['/**']['ros__parameters']
        metadata['ros_environment'] = {key: os.environ.get(key) for key in
                                      ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'AMENT_PREFIX_PATH')}
        write_json(run_path/'metadata.json', metadata)
        gate_params = {'use_sim_time': True, 'run_path': str(run_path), 'heartbeat_s': limits['heartbeat_s'], 'loss_s': limits['loss_s'],
                       'signal_limits': json.dumps(limits['signals']), 'steer_limit': vehicle['max_steering_angle']}
        gate_file = run_path/'gate.yaml'
        gate_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': gate_params}}))
        launch('gate', ['ros2', 'run', 'benchmark', 'benchmark_gate', '--ros-args', '--params-file', str(gate_file)])
        wait(lambda: 'gate' in latest and bool(node.get_publishers_info_by_topic('/drive')), 'command_gate')
        if config['logging']['level'] != 'off':
            recorder_file = run_path/'recorder.yaml'
            recorder_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
                'use_sim_time': True, 'run_path': str(run_path), 'topics': json.dumps(topics),
                'disk_reserve_mb': limits['disk_reserve_mb'], 'storage': config['logging']['storage']}}}))
            launch('recorder', ['ros2', 'run', 'benchmark', 'benchmark_recorder', '--ros-args', '--params-file', str(recorder_file)])
            wait(lambda: latest.get('recorder', (None, {}))[1].get('counts', {}).get('/drive', 0) > 0,
                 'recorder_first_stop_write')
        event('reset_requested')
        for _ in range(3):
            reset.publish(Bool(data=True)); spin(0.1)
        wait(lambda: NS+'sim_reset' in latest, 'native_reset_event')
        state['status'] = 'starting_algorithms'
        if config['experiment']['mode'] == 'managed':
            params = controller_parameters(config, vehicle, assets/'raceline.csv')
            controller_file = run_path/'controller.yaml'
            controller_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            local_command, controller_command = managed_commands(config, controller_file)
            launch('localization', local_command)
            launch('controller', controller_command)
        else:
            event('manual_readiness_wait', required_output=REQUESTED)
        def ready():
            owners = node.get_publishers_info_by_topic('/drive')
            if len(owners) != 1 or owners[0].node_name != 'benchmark_gate':
                return False
            if health.missing(time.monotonic()):
                return False
            if config['logging']['level'] != 'off' and any(capture_counts.get(topic, 0) == 0 for topic in essential):
                return False
            return True
        wait(ready, 'essential_signals_and_capture')
        # Preserve actual endpoint contracts and resolved node parameters before motion.
        metadata['topic_endpoints'] = {}
        for topic in topics:
            metadata['topic_endpoints'][topic] = [{'node': e.node_name, 'namespace': e.node_namespace,
                'type': e.topic_type, 'reliability': str(e.qos_profile.reliability),
                'durability': str(e.qos_profile.durability)} for e in node.get_publishers_info_by_topic(topic)]
        metadata['resolved_parameters'] = {}
        for name in ('fast_bridge', 'adapter', 'wheel_odom', 'localization_gate' if localizer == 'cartographer' else 'mcl',
                     'map_node' if config['components'].get('controller', {}).get('type') == 'map' else 'pure_pursuit_node'):
            import subprocess
            if name not in node.get_node_names():
                metadata['resolved_parameters'][name] = {'unavailable': 'node not present'}
                continue
            try:
                # Keep command-gate heartbeat flowing while the read-only CLI queries DDS.
                proc = subprocess.Popen(['ros2', 'param', 'dump', '/'+name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                began = time.monotonic()
                while proc.poll() is None and time.monotonic()-began < 3.0:
                    spin()
                if proc.poll() is None:
                    proc.kill()
                out, err = proc.communicate()
                metadata['resolved_parameters'][name] = yaml.safe_load(out) if proc.returncode == 0 else {'unavailable': err.decode()}
            except Exception as exc:
                metadata['resolved_parameters'][name] = {'unavailable': str(exc)}
        # Refresh readiness after parameter collection; no cached readiness can authorize motion.
        wait(ready, 'post_snapshot_readiness')
        lap0 = latest[NS+'lap_count'][1].data
        collision0 = latest[NS+'collision_count'][1].data
        warmup = config['experiment']['warmup_laps']
        total = warmup+config['experiment']['laps']
        seen_lap, pending = lap0, None
        measuring = True
        state['status'] = 'warmup' if warmup else 'measuring'
        metadata['motion_enabled_utc'] = utc()
        if not warmup:
            metadata['measurement']['started'] = True
            metadata['measurement']['start_recorded_ns'] = node.get_clock().now().nanoseconds
            metadata['measurement']['start_monotonic_ns'] = time.monotonic_ns()
            metadata['measurement']['collision_baseline'] = collision0
        say('Motion enabled. '+(f'Warm-up lap 1/{warmup} starting.' if warmup else
            'Measurement lap 1 starting (initial lap may be partial or standing).'))
        event('motion_enable', warmup_laps=warmup)
        event('experiment_start')
        started = time.monotonic()
        while time.monotonic()-started < config['experiment']['timeout_s']:
            spin(enabled=True)
            check(enabled=True)
            if config['experiment']['collision_policy'] == 'abort' and latest[NS+'collision_count'][1].data != collision0:
                raise Abort('collision')
            lap = latest[NS+'lap_count'][1].data
            if lap > seen_lap:
                if lap != seen_lap+1 or pending is not None:
                    raise Abort('ambiguous_lap_counter_jump')
                pending = {'counter': lap, 'arrival': time.monotonic(), 'observed_ns': node.get_clock().now().nanoseconds,
                           'arrival_monotonic_ns': time.monotonic_ns()}
                seen_lap = lap
                index = lap-lap0
                if index == warmup:
                    state['status'] = 'measuring'
                    metadata['measurement']['started'] = True
                    metadata['measurement']['start_recorded_ns'] = pending['observed_ns']
                    metadata['measurement']['start_monotonic_ns'] = pending['arrival_monotonic_ns']
                    metadata['measurement']['collision_baseline'] = latest[NS+'collision_count'][1].data
                    event('measurement_start')
                if index > warmup:
                    metadata['measurement']['completed_laps'] += 1
                if index >= total:
                    metadata['measurement']['end_recorded_ns'] = pending['observed_ns']
                    metadata['measurement']['end_monotonic_ns'] = pending['arrival_monotonic_ns']
                    metadata['status'] = 'completed'
                    break
            finish_pending_lap()
        else:
            raise Abort('experiment_timeout')
    except (Exception, KeyboardInterrupt) as exc:
        metadata['status'] = 'stopped' if str(exc) == 'operator_stop' or isinstance(exc, KeyboardInterrupt) else 'aborted'
        metadata['failure'] = str(exc) or 'interrupted'
        if run_path:
            event('abort', reason=metadata['failure'])
        else:
            print(json.dumps({'status': metadata['status'], 'reason': metadata['failure']}))
    finally:
        try:
            if 'gate' in processes:
                stop_vehicle()
            if run_path:
                if metadata.get('stop_confirmation') == 'unconfirmed' and metadata.get('status') == 'completed':
                    metadata.update(status='aborted', failure='stop_unconfirmed')
                state['status'] = 'finalizing'
                event('experiment_end', status=metadata.get('status'), failure=metadata.get('failure'))
                spin(0.3)
                if 'recorder' in processes:
                    metadata['process_results']['recorder'] = processes['recorder'].close(spin)
                    final = run_path/'recorder_final.json'
                    metadata['recorder'] = json.loads(final.read_text()) if final.exists() else {'closed': False}
                    result = metadata['process_results']['recorder']
                    if not metadata['recorder'].get('closed') or metadata['recorder'].get('error') or result['exit_code'] != 0:
                        metadata.update(status='aborted', failure=metadata.get('failure') or 'recorder_finalization_failure')
                    if metadata['recorder'].get('closed'):
                        try:
                            bag_meta = yaml.safe_load((run_path/'bag'/'metadata.yaml').read_text())['rosbag2_bagfile_information']
                            actual = {item['topic_metadata']['name']: item['message_count']
                                      for item in bag_meta['topics_with_message_count'] if item['message_count']}
                            expected = metadata['recorder']['counts']
                            metadata['recorder_integrity'] = {'bag_counts': actual, 'accepted_counts': expected,
                                                               'consistent': actual == expected}
                            if actual != expected:
                                metadata.update(status='aborted', failure=metadata.get('failure') or 'bag_count_mismatch')
                        except Exception as exc:
                            metadata.update(status='aborted', failure=metadata.get('failure') or 'bag_metadata_verification_failed')
                            metadata['bag_verification_error'] = str(exc)
                if 'gate' in processes:
                    metadata['process_results']['gate'] = processes['gate'].close(spin)
                metadata['gaps'] = health.gaps
                metadata['observed_counts'] = capture_counts
                metadata['missing_optional_topics'] = sorted(set(topics)-set(capture_counts)) if config['logging']['level'] != 'off' else []
                if event_stream:
                    event_stream.close()
                if any(process.group_alive() for process in processes.values()):
                    raise RuntimeError('Owned process still alive; preserving incomplete run without sealing')
                finalize(run_path, metadata)
                finalized = True
                if collection_lock:
                    collection_lock.close()
                state['status'] = metadata['status']
                status_pub.publish(String(data=json.dumps({**state, 'run_id': run_path.name})))
                say(result_message(metadata['status'], metadata['measurement']['completed_laps'],
                    config['experiment']['laps'], metadata.get('failure'), metadata.get('stop_confirmation')))
                say(f'Saved raw collection: {run_path}')
                say(f'Failure/configuration details: {run_path / "metadata.json"}')
                print(json.dumps({'run_id': run_path.name, 'status': metadata['status'],
                                  'reason': metadata.get('failure'),
                                  'stop_confirmation': metadata.get('stop_confirmation'), 'path': str(run_path)}))
        finally:
            for process in processes.values():
                if process.group_alive():
                    process.close(spin)
            if event_stream and not event_stream.closed:
                event_stream.close()
            if collection_lock and not collection_lock.closed:
                collection_lock.close()
            for sig, previous in previous_signals.items():
                signal.signal(sig, previous)
            node.destroy_node()
            rclpy.try_shutdown()
            singleton.close()
    if finalized and on_finalized is not None:
        on_finalized(run_path.name)
    return 0 if metadata.get('status') == 'completed' else 1


def remote(command):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node('benchmark_cli')
    try:
        if command == 'stop':
            client = node.create_client(Trigger, '/benchmark/stop')
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('No active benchmark manager')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if not future.done():
                raise RuntimeError('Manager stop request timed out')
            print(future.result().message)
        else:
            received = []
            node.create_subscription(String, '/benchmark/status', lambda msg: received.append(msg.data),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            deadline = time.monotonic()+3
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            print(received[-1] if received else json.dumps({'status': 'no_active_manager'}))
    finally:
        node.destroy_node(); rclpy.try_shutdown()
    return 0
