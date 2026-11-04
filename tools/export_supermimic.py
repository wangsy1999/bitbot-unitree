#!/usr/bin/env python3
"""Export a SuperMimic ONNX policy and a portable CSV motion manifest.

Run with the Uni-SuperMimic Python environment. No simulator is started.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import shutil
import sys
import types


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    motions = parser.add_mutually_exclusive_group(required=True)
    motions.add_argument('--motion', type=Path, nargs='+', action='extend',
                         help='One or more motion CSVs with a common source FPS')
    motions.add_argument('--manifest', type=Path,
                         help='Existing manifest preserving per-motion FPS, names and crop ranges')
    parser.add_argument('--source-fps', type=float, default=30.0)
    parser.add_argument('--output', type=Path, default=Path('CheckPoints/supermimic'))
    args = parser.parse_args()
    source = args.source.resolve()
    sys.path.insert(0, str(source / 'src'))
    import numpy as np
    import onnxruntime as ort
    import torch
    from unisupermimic.rl.algos.supermimic import networks

    # Import task data modules without loading the training backend registry.
    package = types.ModuleType('_supermimic_export')
    package.__path__ = [str(source / 'src/unisupermimic/tasks/motion_tracking/g1/supermimic')]
    sys.modules[package.__name__] = package
    robot = importlib.import_module('_supermimic_export.robot')
    motion_module = importlib.import_module('_supermimic_export.motion')
    dt = motion_module.CONTROL_DT
    if args.manifest:
        manifest = json.loads(args.manifest.read_text())
        if manifest.get('SchemaVersion') != 1 or abs(manifest.get('ControlDt', dt) - dt) > 1e-9:
            raise ValueError('Unsupported motion manifest schema/control period')
        entries = [(dict(entry), (args.manifest.parent / entry['Path']).resolve())
                   for entry in manifest['Motions']]
    else:
        entries = [(dict(Name=path.stem, SourceFPS=args.source_fps), path.resolve())
                   for path in args.motion]
    names = [entry['Name'] for entry, _ in entries]
    if not names or len(set(names)) != len(names):
        raise ValueError('Motion names must be unique and nonempty')
    for entry, path in entries:
        if not entry['Name'] or not path.is_file() or not np.isfinite(entry['SourceFPS']) or entry['SourceFPS'] <= 0:
            raise ValueError(f'Invalid motion entry: {entry}')

    torch.set_num_threads(1)
    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=True)
    cfg, weights = checkpoint['args'], checkpoint['pi']
    obs_dim, goal_dim = weights['obs_norm.mean'].numel(), weights['goal_norm.mean'].numel()
    hist, goal_frames = int(cfg['hist']), int(cfg['goal_frames'])
    if obs_dim != 99 or goal_dim != 73 * goal_frames or hist <= 0 or goal_frames <= 0:
        raise ValueError('Unsupported SuperMimic policy contract')
    hidden = cfg.get('test_hidden_dims') or cfg['policy_hidden']
    networks.AMP_DTYPE = None
    policy = networks.Policy(obs_dim, goal_dim, hist=hist, z_dim=int(cfg['z_dim']),
                             num_joints=len(robot.G1_JOINT_NAMES), hidden=hidden).eval()
    policy.load_state_dict(weights, strict=True)
    args.output.mkdir(parents=True, exist_ok=True)
    obs, goal = torch.zeros(1, hist, obs_dim), torch.zeros(1, goal_dim)
    model_path = args.output / 'policy.onnx'
    torch.onnx.export(policy, (obs, goal), str(model_path), input_names=['obs_hist', 'goal'],
                      output_names=['actions'], opset_version=17, dynamo=False)
    session = ort.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
    generator = torch.Generator().manual_seed(0)
    for sample in [obs, torch.randn(obs.shape, generator=generator) * 0.1]:
        with torch.no_grad():
            expected = policy(sample, goal).numpy()
        actual = session.run(None, {'obs_hist': sample.numpy(), 'goal': goal.numpy()})[0]
        np.testing.assert_allclose(actual, expected, rtol=3e-4, atol=2e-5)

    output = args.output.resolve()
    csv_entries, local_paths = [], []
    for number, (entry, path) in enumerate(entries, 1):
        if path.is_relative_to(output / 'trajectories'):
            dest = path
        else:
            dest = output / 'trajectories' / 'exported' / f'{number:03d}_{path.name}'
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, dest)
        entry['Path'] = dest.relative_to(output / 'trajectories').as_posix()
        entry.setdefault('OriginalPath', str(path))
        entry['SHA256'] = hashlib.sha256(dest.read_bytes()).hexdigest()
        csv_entries.append(entry)
        local_paths.append(dest.relative_to(output).as_posix())
    manifest = dict(SchemaVersion=1, ControlDt=dt, PathBase='manifest directory', Motions=csv_entries)
    (output / 'trajectories/manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    names = robot.G1_JOINT_NAMES
    worker = {
        'SchemaVersion': 1,
        'Inference': {'WarmUpModel': False, 'IntraNumberThreads': 1},
        'Network': {'ModelPath': 'policy.onnx', 'InputNodeNames': ['obs_hist', 'goal'],
                    'OutputNodeNames': ['actions'], 'Cycle_time': dt},
        'Preprocess': {'ReferencePath': 'trajectories/manifest.json', 'HistoryLength': hist,
                       'GoalFrames': goal_frames, 'JointIds': list(range(len(names)))},
        'Postprocess': {'DefaultPosition': robot.default_joint_pos_list(),
                        'action_scale': [robot.action_scale(name) for name in names]},
        'Motor': {'JointNames': names,
                  'Stiffness': [robot._kp(robot.motor_of(name)[2]) for name in names],
                  'Damping': [robot._kd(robot.motor_of(name)[2]) for name in names]},
        'Source': {'Checkpoint': str(args.checkpoint.resolve()), 'Iteration': checkpoint.get('iteration'),
                   'Motions': local_paths, 'MotionManifest': 'trajectories/manifest.json',
                   'MotionSourceFPS': {entry['Name']: entry['SourceFPS'] for entry in csv_entries}},
    }
    (output / 'worker.json').write_text(json.dumps(worker, indent=2, allow_nan=False) + '\n')
    print(f'Exported {model_path}: [1,{hist},{obs_dim}] + [1,{goal_dim}] -> [1,{len(names)}]')
    print(f'CSV manifest: {len(csv_entries)} motions; ONNX/PyTorch comparison passed')


if __name__ == '__main__':
    main()
