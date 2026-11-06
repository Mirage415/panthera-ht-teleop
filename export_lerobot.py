"""Convert synchronized RGB/state/command samples using the official LeRobot writer."""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import time

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ROOT = Path(__file__).resolve().parent
JOINTS = [f'joint_{i}' for i in range(1, 7)] + ['gripper']
FPS = 30


def read_episode(folder):
    states, images, actions = [], [], []
    decoder = DecoderFactory()
    with (folder / 'data.mcap').open('rb') as handle:
        for schema, channel, message in make_reader(handle).iter_messages(
                topics=['/follower/joint_states', '/camera/color/image']):
            if channel.topic == '/follower/joint_states':
                row = json.loads(message.data)
                stamp = row['timestamp_ns']
                states.append((stamp, row['position']))
                if row.get('action_source') == 'follower_command_target':
                    actions.append((stamp, row['action']))
            else:
                image = decoder.decoder_for(channel.message_encoding, schema)(message.data)
                images.append((message.log_time, bytes(image.data)))
    source = 'follower_command_target'
    if not actions:
        recovered = folder / 'actions-recovered.jsonl'
        if not recovered.exists():
            # Old recordings had targets in the matching host-time teleop log.
            # Preserve the selected records before teleop-latest is rotated.
            selected = []
            for path in sorted((ROOT / '.runtime').glob('teleop*.jsonl')):
                with path.open() as handle:
                    for line in handle:
                        row = json.loads(line)
                        stamp = round(row['wall_time'] * 1e9)
                        if states and states[0][0] - 100_000_000 <= stamp <= states[-1][0] + 100_000_000:
                            selected.append(dict(timestamp_ns=stamp, action=row['target'], source=path.name))
            if selected:
                recovered.write_text(''.join(json.dumps(row) + '\n' for row in selected))
        if recovered.exists():
            for line in recovered.read_text().splitlines():
                row = json.loads(line)
                actions.append((row['timestamp_ns'], row['action']))
        source = 'legacy_teleop_log_target_approximately_20Hz_zero_order_hold'
    if not states or not images or not actions:
        raise ValueError(f'{folder.name}: 缺少 RGB、实测状态或真实目标 action，不导出训练数据')
    for values in (states, images, actions):
        values.sort(key=lambda pair: pair[0])
    return states, images, actions, source


def aligned_frames(folder):
    states, images, actions, source = read_episode(folder)
    streams = [states, images, actions]
    times = [[pair[0] for pair in stream] for stream in streams]
    begin = max(t[0] for t in times)
    end = min(t[-1] for t in times)
    count = int((end - begin) * FPS // 1_000_000_000) + 1
    if count < 2:
        raise ValueError('episode 太短')
    max_age = [0, 0, 0]
    def frames():
        for index in range(count):
            stamp = begin + round(index * 1_000_000_000 / FPS)
            picks = []
            for stream_index, stamps in enumerate(times):
                idx = bisect_right(stamps, stamp) - 1
                if stream_index == 1 and idx + 1 < len(stamps) and abs(stamps[idx + 1] - stamp) < abs(stamps[idx] - stamp):
                    idx += 1
                age = abs(stamps[idx] - stamp)
                limit = [50_000_000, 35_000_000, 80_000_000][stream_index]
                if age > limit:
                    raise ValueError(f'{folder.name}: 数据断流，无法在第 {index} 帧对齐（stream={stream_index}, delta_ms={age / 1e6:.1f}）')
                max_age[stream_index] = max(max_age[stream_index], age)
                picks.append(streams[stream_index][idx])
            state, rgb, action = picks
            image = cv2.imdecode(np.frombuffer(rgb[1], dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('JPEG 解码失败')
            yield {
                'observation.state': np.asarray(state[1], dtype=np.float32),
                'action': np.asarray(action[1], dtype=np.float32),
                'observation.images.realsense': cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
                'source.state_timestamp_ns': np.array([state[0]], dtype=np.int64),
                'source.action_timestamp_ns': np.array([action[0]], dtype=np.int64),
                'source.rgb_timestamp_ns': np.array([rgb[0]], dtype=np.int64),
            }
    report = dict(source_episode=folder.name, action_source=source, frames=count,
                  start_time_ns=begin, max_alignment_delta_ns=max_age)
    return frames(), report


def export_episode(folder, destination, repo_id, task):
    source_meta = json.loads((folder / 'metadata.json').read_text())
    task_info = source_meta.get('task', {})
    task = task_info.get('prompt') or task
    destination = Path(destination)
    manifest_path = destination / 'source_episodes.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {'repo_id': repo_id, 'episodes': []}
    if manifest['repo_id'] != repo_id:
        raise ValueError('导出目录属于另一个 Hugging Face 仓库')
    if folder.name in [row['source_episode'] for row in manifest['episodes']]:
        print(json.dumps(manifest['episodes'][-1]));return
    frames, report = aligned_frames(folder)
    report['task'] = dict(task_info, prompt=task)
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (7,), 'names': JOINTS},
        'action': {'dtype': 'float32', 'shape': (7,), 'names': JOINTS},
        'observation.images.realsense': {'dtype': 'video', 'shape': (480, 640, 3), 'names': ['height', 'width', 'channels']},
    }
    for key in ('state', 'action', 'rgb'):
        features[f'source.{key}_timestamp_ns'] = {'dtype': 'int64', 'shape': (1,), 'names': ['unix_ns']}
    # Work on a staging copy so a failed conversion cannot corrupt prior episodes.
    import shutil
    staging = destination.with_name(destination.name + '.staging')
    if staging.exists():
        shutil.rmtree(staging)
    if (destination / 'meta/info.json').exists():
        shutil.copytree(destination, staging)
        dataset = LeRobotDataset.resume(repo_id, root=staging, video_backend='pyav', encoder_threads=2)
    else:
        dataset = LeRobotDataset.create(repo_id=repo_id, fps=FPS, features=features,
            root=staging, robot_type='panthera_ht', use_videos=True, video_backend='pyav', encoder_threads=2)
    report['episode_index'] = dataset.meta.total_episodes
    try:
        for frame in frames:
            frame['task'] = task
            dataset.add_frame(frame)
        dataset.save_episode()
    finally:
        dataset.finalize()
    # Decode data and video with the official reader before making it uploadable.
    verified = LeRobotDataset(repo_id, root=staging, video_backend='pyav')
    assert verified.num_episodes == len(manifest['episodes']) + 1
    last = verified[len(verified) - 1]
    assert tuple(last['observation.state'].shape) == (7,)
    assert tuple(last['action'].shape) == (7,)
    assert tuple(last['observation.images.realsense'].shape) == (3, 480, 640)
    manifest['episodes'].append(report)
    (staging / 'source_episodes.json').write_text(json.dumps(manifest, indent=2))
    (staging / 'README.md').write_text('''---
tags:
- lerobot
- robotics
- panthera
---
# NYUSH Panthera teleoperation

LeRobot dataset generated with the official dataset writer at 30 FPS.

- `observation.images.realsense`: RGB, 640 x 480, encoded video.
- `observation.state`: six follower joint angles and gripper motor angle, radians.
- `action`: actual follower command target angles after position/speed limits, radians.
- New episodes use the operator-selected exact task prompt; legacy episodes retain their original generic label.
- Host-time synchronization: nearest RGB (<=35 ms), latest state (<=50 ms), latest command (<=80 ms).
- Source timestamps are retained as int64 nanoseconds. No hardware synchronization.
- Legacy episode actions recovered from matching approximately 20 Hz teleop logs use zero-order hold.
  Per-episode provenance and measured alignment deltas are in `source_episodes.json`.
- Raw MCAP and CSV remain on the capture computer. This repository contains the training export.
''')
    backup = destination.with_name(destination.name + '.previous')
    if backup.exists():
        shutil.rmtree(backup)
    if destination.exists():
        destination.rename(backup)
    staging.rename(destination)
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--repo', required=True)
    parser.add_argument('--task', default='Panthera teleoperation')
    args = parser.parse_args()
    export_episode(args.episode, args.destination, args.repo, args.task)
