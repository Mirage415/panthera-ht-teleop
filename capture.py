#!/usr/bin/env python3
"""RealSense -> Foxglove + MCAP; follower telemetry -> MCAP + CSV."""
import argparse
from collections import deque
from contextlib import ExitStack
import json
from pathlib import Path
import signal
import socket
import time

import cv2
import foxglove
from foxglove.channels import CompressedImageChannel, RawImageChannel
from foxglove.messages import CompressedImage, RawImage, Timestamp
from foxglove.websocket import Capability

from recording import Recorder, RecordingControl
from teleop_session import TeleopSession
from hub_upload import HubUploader
from collection_workflow import CollectionWorkflow
import numpy as np
import pyrealsense2 as rs

from telemetry import NAMES, PORT

ROOT = Path(__file__).resolve().parent
CSV_FIELDS = ['timestamp_ns', 'monotonic_ns', 'session', 'sequence'] + [
    f'{name}_{field}' for field in ('rad', 'rad_s', 'nm', 'feedback_time_s') for name in NAMES]
CSV_FIELDS += [f'{name}_action_rad' for name in NAMES]

# Explicit numeric types let Foxglove discover and plot array elements before
# the first telemetry packet arrives (including when connecting while idle).
JOINT_SCHEMA = {
    'type': 'object',
    'properties': {
        **{key: {'type': 'array', 'items': {'type': 'number'},
                 'minItems': 7, 'maxItems': 7}
           for key in ('position', 'velocity', 'effort', 'feedback_time_s', 'action', 'leader_position')},
        'name': {'type': 'array', 'items': {'type': 'string'}},
        **{key: {'type': 'integer'} for key in
           ('timestamp_ns', 'monotonic_ns', 'sequence', 'supervisor_pid')},
        **{key: {'type': 'string'} for key in
           ('session', 'position_unit', 'action_unit', 'action_source')},
    },
}


def joint_channel(context=None):
    return foxglove.Channel('/follower/joint_states', message_encoding='json',
        schema=foxglove.Schema(name='panthera.JointState', encoding='jsonschema',
                              data=json.dumps(JOINT_SCHEMA).encode()), context=context)


def validate_joint(row):
    if row['name'] != NAMES or row['position_unit'] != 'rad':
        raise ValueError('unexpected joints or units')
    for key in ('position', 'velocity', 'effort', 'feedback_time_s'):
        values = np.asarray(row[key], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(f'invalid {key}')
    for key in ('timestamp_ns', 'monotonic_ns', 'sequence'):
        if type(row[key]) is not int or row[key] < 0:
            raise ValueError(f'invalid {key}')
    if not isinstance(row['session'], str) or not row['session']:
        raise ValueError('invalid session')
    if 'action' in row:
        values = np.asarray(row['action'], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all() or row.get('action_unit') != 'rad':
            raise ValueError('invalid action')
    if 'leader_position' in row:
        values = np.asarray(row['leader_position'], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError('invalid leader_position')
    return row


def csv_row(row):
    return [row[k] for k in CSV_FIELDS[:4]] + sum(
        [row[k] for k in ('position', 'velocity', 'effort', 'feedback_time_s')], []) + row.get('action', [''] * 7)


def timestamp(ns):
    return Timestamp(sec=ns // 1_000_000_000, nsec=ns % 1_000_000_000)


def camera_latency_ms(frame_time_ms, domain, received_ns):
    # Only global_time is a device capture timestamp mapped onto the host clock.
    # system_time may be assigned on arrival, so cannot measure capture latency.
    if domain != rs.timestamp_domain.global_time:
        return None
    latency = received_ns / 1e6 - frame_time_ms
    return round(latency, 2) if np.isfinite(latency) and 0 <= latency <= 5000 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', action='store_true', help='启动时只预览，可在 Foxglove 页面开始/停止采集')
    parser.add_argument('--depth', action='store_true', help='同时采集原始 uint16 深度')
    parser.add_argument('--joints-only', action='store_true', help='图像只预览，仅保存关节及状态数据')
    parser.add_argument('--duration', type=float, help='自动结束秒数')
    parser.add_argument('--serial', help='指定 RealSense 序列号')
    parser.add_argument('--output', type=Path, default=ROOT / 'recordings')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址，默认允许通过采集机 IP 连接')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be positive')
    stop = False
    def request_stop(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    stats = dict(rgb_frames=0, depth_frames=0, joint_samples=0, sequence_gaps=0,
                 invalid_packets=0, stale_packets=0, out_of_order_packets=0)
    metadata = dict(start_time_ns=time.time_ns(), arguments=vars(args).copy(),
                    joint_names=NAMES, position_unit='rad',
                    timestamp_policy='Unix host receive/snapshot time; device timestamps retained separately; no hardware synchronization',
                    complete=False)
    metadata['arguments']['output'] = str(args.output)
    with ExitStack() as stack:
        receiver = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        receiver.bind(('127.0.0.1', PORT))
        receiver.setblocking(False)
        camera = rs.pipeline()
        config = rs.config()
        if args.serial:
            config.enable_device(args.serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if args.depth:
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = camera.start(config)
        stack.callback(camera.stop)
        device = profile.get_device()
        for sensor in device.query_sensors():
            if sensor.supports(rs.option.global_time_enabled):
                sensor.set_option(rs.option.global_time_enabled, 1)
        metadata['camera'] = dict(name=device.get_info(rs.camera_info.name),
            serial=device.get_info(rs.camera_info.serial_number),
            firmware=device.get_info(rs.camera_info.firmware_version))
        metadata['intrinsics'] = {}
        for stream in profile.get_streams():
            intr = stream.as_video_stream_profile().get_intrinsics()
            metadata['intrinsics'][str(stream.stream_type())] = dict(
                width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy,
                ppx=intr.ppx, ppy=intr.ppy, model=str(intr.model), coeffs=intr.coeffs)
        if args.depth:
            metadata['depth_scale_m'] = device.first_depth_sensor().get_depth_scale()
        rgb_channel = CompressedImageChannel('/camera/color/image')
        depth_channel = RawImageChannel('/camera/depth/image') if args.depth else None
        camera_info = foxglove.Channel('/camera/frame_info', message_encoding='json')
        joints = joint_channel()
        status = foxglove.Channel('/capture/status', message_encoding='json')
        recorder = Recorder(args.output, metadata, CSV_FIELDS, joints_only=args.joints_only)
        teleop_session = TeleopSession()
        uploader = HubUploader(args.output)
        workflow = CollectionWorkflow(args.output / '.collection-workflow.json')
        control = RecordingControl(teleop=teleop_session, uploader=uploader, workflow=workflow)
        server = foxglove.start_server(name='Panthera capture', host=args.host, port=args.port,
            capabilities=[Capability.Services], supported_encodings=['json'], services=control.services())
        stack.callback(server.stop)
        stack.callback(control.close)
        print(f"相机：{metadata['camera']['name']}；Foxglove 监听：{args.host}:{args.port}", flush=True)
        print('Foxglove 页面服务：/capture/start、/capture/stop、/capture/status；请求填 {}。', flush=True)
        print('页面点击开始将自动启动 Teleop 并采集；点击停止将结束遥操作和保存。Ctrl+C 退出服务。', flush=True)
        started = time.monotonic()
        last_frame = started
        have_frame = False
        last_joint = None
        follower_supervisor_pid = None
        last_report = 0
        frame_arrivals = deque(maxlen=60)
        camera_latency = None
        camera_domain = ''
        sequences = {}
        last_numbers = {}
        failure = ''
        try:
            if not args.preview:
                recorder.start(stats)
            while not stop and (args.duration is None or time.monotonic() - started < args.duration):
                for _ in range(200):
                    try:
                        packet = receiver.recv(65535)
                    except BlockingIOError:
                        break
                    try:
                        row = validate_joint(json.loads(packet))
                    except (ValueError, KeyError, TypeError):
                        stats['invalid_packets'] += 1
                        continue
                    age = (time.time_ns() - row['timestamp_ns']) / 1e9
                    if not -0.05 <= age <= 1:
                        stats['stale_packets'] += 1
                        continue
                    previous = sequences.get(row['session'])
                    if previous is not None:
                        if row['sequence'] <= previous:
                            stats['out_of_order_packets'] += 1
                            continue
                        stats['sequence_gaps'] += row['sequence'] - previous - 1
                    sequences[row['session']] = row['sequence']
                    joints.log(row, log_time=row['timestamp_ns'])
                    recorder.capture_start_pose(row)
                    recorder.write_joint(csv_row(row))
                    stats['joint_samples'] += 1
                    last_joint = time.monotonic()
                    follower_supervisor_pid = row.get('supervisor_pid')
                frames = camera.poll_for_frames()
                if frames:
                    now_ns = time.time_ns()
                    for label, frame in [('color', frames.get_color_frame()), ('depth', frames.get_depth_frame())]:
                        if not frame or (label == 'depth' and not args.depth):
                            continue
                        number = frame.get_frame_number()
                        if last_numbers.get(label) == number:
                            continue
                        last_numbers[label] = number
                        array = np.asanyarray(frame.get_data())
                        if label == 'color':
                            ok, encoded = cv2.imencode('.jpg', array, [cv2.IMWRITE_JPEG_QUALITY, 85])
                            if not ok:
                                raise RuntimeError('JPEG encoding failed')
                            rgb_channel.log(CompressedImage(timestamp=timestamp(now_ns), frame_id='camera_color_optical_frame',
                                format='jpeg', data=encoded.tobytes()), log_time=now_ns)
                            stats['rgb_frames'] += 1
                            last_frame = time.monotonic()
                            frame_arrivals.append(last_frame)
                            camera_domain = str(frame.get_frame_timestamp_domain())
                            camera_latency = camera_latency_ms(frame.get_timestamp(),
                                frame.get_frame_timestamp_domain(), now_ns)
                            have_frame = True
                        else:
                            depth_channel.log(RawImage(timestamp=timestamp(now_ns), frame_id='camera_depth_optical_frame',
                                width=frame.get_width(), height=frame.get_height(), encoding='16UC1',
                                step=frame.get_width() * 2, data=array.astype('<u2', copy=False).tobytes()), log_time=now_ns)
                            stats['depth_frames'] += 1
                        camera_info.log(dict(stream=label, timestamp_ns=now_ns, frame_number=number,
                            device_timestamp_ms=frame.get_timestamp(), timestamp_domain=str(frame.get_frame_timestamp_domain())), log_time=now_ns)
                now = time.monotonic()
                if now - last_frame > 5:
                    raise RuntimeError('相机超过 5 秒没有 RGB 帧')
                live = dict(camera_live=have_frame and now - last_frame < 0.5,
                            camera_age_ms=round((now - last_frame) * 1000, 2) if have_frame else None,
                            camera_latency_ms=camera_latency if have_frame and now - last_frame < 0.5 else None,
                            camera_timestamp_domain=camera_domain,
                            camera_fps=round((len(frame_arrivals) - 1) / (frame_arrivals[-1] - frame_arrivals[0]), 1)
                                if len(frame_arrivals) > 1 and now - last_frame < 0.5 else 0,
                            follower_supervisor_pid=follower_supervisor_pid,
                            follower_live=last_joint is not None and now - last_joint < 0.5,
                            follower_age_s=None if last_joint is None else now - last_joint)
                control.drain(recorder, stats, live)
                teleop_session.tick(recorder, stats, live)
                workflow.tick(teleop_session, recorder)
                try:
                    uploader.tick()
                except Exception as exc:
                    uploader.snapshot.update(upload_state='error', upload_error=f'上传队列错误：{type(exc).__name__}')
                if now - last_report >= 0.2:
                    state = dict(**stats, **live, **recorder.state(stats), **teleop_session.state(), **uploader.state(),
                                 **workflow.state(recorder, teleop_session))
                    status.log(state)
                    if int(now) != int(last_report):
                        print(f"RGB {stats['rgb_frames']} | joints {stats['joint_samples']} | {state['recording_state']} | follower {'在线' if state['follower_live'] else '等待/断流'}", flush=True)
                    recorder.flush()
                    last_report = now
                time.sleep(0.002)
        except Exception as exc:
            failure = str(exc)
            raise
        finally:
            control.close()
            teleop_session.close(recorder, stats, error=failure)
            try:
                uploader.tick(force=True)
            except Exception as exc:
                print(f'上传排队失败，本地文件保留：{type(exc).__name__}', flush=True)
            print(json.dumps(stats), flush=True)
            if recorder.last_path:
                print(f'最后保存目录：{recorder.last_path}', flush=True)
    if recorder.last_result and not recorder.last_result['complete']:
        print('最后一次采集不完整，请检查 metadata.json。', flush=True)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
