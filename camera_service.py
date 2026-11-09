#!/usr/bin/env python3
"""Read-only RealSense RGB/depth viewer, served on the NUC's LAN interface."""
import argparse
from collections import deque
import io
import json
import uuid
import zipfile
import logging
import signal
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response, jsonify, request
from werkzeug.serving import make_server
from camera_settings import apply_color_settings

app = Flask(__name__)
condition = threading.Condition()
stop = threading.Event()
state = {"connected": False, "frames": 0, "fps": 0, "error": "相机启动中", "updated": 0}
images = {}
frame_history = deque(maxlen=60)
camera_session = uuid.uuid4().hex
depth_png = None
cv2.setNumThreads(1)

PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NUC · RealSense 实时画面</title>
<style>
:root{color-scheme:dark;font-family:system-ui,-apple-system,sans-serif;background:#10151d;color:#e9eff9}
*{box-sizing:border-box}body{max-width:1440px;margin:auto;padding:32px 24px}header{display:flex;justify-content:space-between;gap:16px;align-items:center;margin-bottom:28px}
h1{font-size:26px;margin:6px 0}p{color:#9caec4;line-height:1.7;margin:6px 0}.tag{font-size:12px;color:#76dcd0;letter-spacing:.15em}
#status{padding:10px 16px;border-radius:24px;background:#293342;font-size:14px;white-space:nowrap}.grid{display:grid;grid-template-columns:1fr 1fr;gap:20px}
article{background:#19222e;border:1px solid #2b394a;border-radius:16px;overflow:hidden}article header{margin:0;padding:18px 20px;font-size:16px}.image{aspect-ratio:4/3;background:#060a10;display:flex;align-items:center;justify-content:center}.image img{display:block;width:100%;height:100%;object-fit:contain}.stale img{opacity:.22}
.info{padding:16px 20px;color:#a8bad0;font-size:13px}.stats{display:flex;gap:32px;flex-wrap:wrap;margin:24px 0;padding:22px;background:#19222e;border-radius:14px}.stat label{display:block;color:#91a5bf;font-size:12px;margin-bottom:6px}.stat b{font-size:20px;font-weight:550}
#error{color:#ffb997}a{color:#80d8d0;text-decoration:none}footer{font-size:13px;color:#9caec4;line-height:1.8}button{font:inherit;border:1px solid #41536b;border-radius:8px;background:#233347;color:#e9eff9;padding:7px 14px;cursor:pointer}
@media(max-width:760px){body{padding:20px 14px}.grid{grid-template-columns:1fr}header{align-items:flex-start}h1{font-size:21px}.stats{gap:22px}}
</style>
<header><div><div class="tag">NYUSH NUC / CAMERA LIVE</div><h1>RealSense 实时画面</h1><p>彩色与深度 · D435i</p></div><span id="status">正在连接</span></header>
<div class="grid"><article><header>彩色画面 <button onclick="snapshot('color')">保存快照</button></header><div class="image"><img id="color" src="/stream/color" alt="等待彩色画面"></div><div class="info" id="colorInfo">RGB</div></article>
<article><header>深度画面 <button onclick="snapshot('depth')">保存快照</button></header><div class="image"><img id="depth" src="/stream/depth" alt="等待深度画面"></div><div class="info">距离色标固定为 0–4 米；无效深度显示为黑色。两个画面未做像素对齐。</div></article></div>
<div class="stats"><div class="stat"><label>实际帧率</label><b id="fps">—</b></div><div class="stat"><label>深度中心点</label><b id="distance">—</b></div><div class="stat"><label>USB 连接</label><b id="usb">—</b></div><div class="stat"><label>已接收帧数</label><b id="frames">—</b></div><div class="stat"><label>最后一帧</label><b id="age">—</b></div></div>
<p id="error"></p><footer>视频仅实时转发，不在服务器录制。<a href="/api/status" target="_blank">状态 JSON</a> · <a href="/depth.png" target="_blank">原始 16 位深度 PNG</a><br>深度 PNG 像素值乘以状态接口中的 depth_scale 即为米。只读相机页面，不含机械臂控制。</footer>
<script>
function snapshot(kind){window.open('/snapshot/'+kind+'.jpg?t='+Date.now(),'_blank')}
let wasLive=false;
async function update(){try{const r=await fetch('/api/status',{cache:'no-store'});const s=await r.json();const live=s.connected&&s.age_seconds!==null&&s.age_seconds<3;if(live&&!wasLive){for(const kind of ['color','depth'])document.querySelector('#'+kind).src='/stream/'+kind+'?t='+Date.now()}wasLive=live;document.querySelector('#status').textContent=live?'● 实时连接':'○ 等待相机';document.querySelector('#status').style.color=live?'#76dcd0':'#ffb997';document.querySelector('.grid').classList.toggle('stale',!live);document.querySelector('#fps').textContent=s.fps.toFixed(1)+' fps';document.querySelector('#distance').textContent=live&&s.center_distance_m? s.center_distance_m.toFixed(2)+' m':'—';document.querySelector('#usb').textContent=s.usb||'—';document.querySelector('#frames').textContent=s.frames;document.querySelector('#age').textContent=s.age_seconds===null?'—':s.age_seconds.toFixed(1)+' 秒前';document.querySelector('#error').textContent=s.error||'';document.querySelector('#colorInfo').textContent=s.profile||'RGB';}catch(e){wasLive=false;document.querySelector('#status').textContent='网页服务连接中断';document.querySelector('.grid').classList.add('stale')} }
setInterval(update,1000);update();
</script></html>"""


def camera(serial, fps, low_bandwidth=False):
    global depth_png
    while not stop.is_set():
        pipeline = rs.pipeline()
        started = False
        try:
            config = rs.config()
            config.enable_device(serial)
            cw, ch = (424, 240) if low_bandwidth else (640, 480)
            dw, dh = (480, 270) if low_bandwidth else (640, 480)
            config.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
            config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
            profile = pipeline.start(config)
            started = True
            device = profile.get_device()
            color_options = apply_color_settings(device)
            scale = device.first_depth_sensor().get_depth_scale()
            intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            def intr_dict(stream):
                v=profile.get_stream(stream).as_video_stream_profile().get_intrinsics()
                return dict(width=v.width,height=v.height,fx=v.fx,fy=v.fy,ppx=v.ppx,ppy=v.ppy,model=str(v.model),coeffs=v.coeffs)
            extr=profile.get_stream(rs.stream.depth).get_extrinsics_to(profile.get_stream(rs.stream.color))
            for sensor in device.query_sensors():
                if sensor.supports(rs.option.global_time_enabled):sensor.set_option(rs.option.global_time_enabled,1)
            with condition:
                state.update(color_options=color_options,color_intrinsics=intr_dict(rs.stream.color),
                    depth_to_color=dict(rotation=extr.rotation,translation=extr.translation),session=camera_session)
            with condition:
                state.update(name=device.get_info(rs.camera_info.name), serial=serial,
                             usb=device.get_info(rs.camera_info.usb_type_descriptor), depth_scale=scale,
                             profile=f"RGB {cw} × {ch} / 深度 {dw} × {dh} · 配置 {fps} fps", depth_intrinsics={
                                 "width": intrinsics.width, "height": intrinsics.height,
                                 "fx": intrinsics.fx, "fy": intrinsics.fy, "ppx": intrinsics.ppx,
                                 "ppy": intrinsics.ppy, "model": str(intrinsics.model), "coeffs": intrinsics.coeffs})
            count, period_start = 0, time.monotonic()
            while not stop.is_set():
                frames = pipeline.wait_for_frames(10000)
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if not color or not depth:
                    continue
                received_ns=time.time_ns()
                rgb = np.asanyarray(color.get_data())
                raw = np.asanyarray(depth.get_data())
                colored = cv2.applyColorMap(cv2.convertScaleAbs(raw, alpha=scale*255/4), cv2.COLORMAP_TURBO)
                colored[raw == 0] = 0
                encoded = {}
                for name, frame in (("color", rgb), ("depth", colored)):
                    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ok:
                        raise RuntimeError("JPEG 编码失败")
                    encoded[name] = jpg.tobytes()
                ok, png = cv2.imencode(".png", raw, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if not ok:
                    raise RuntimeError("Depth PNG 编码失败")
                count += 1
                elapsed = time.monotonic()-period_start
                with condition:
                    images.update(encoded)
                    depth_png = png.tobytes()
                    state.update(connected=True, error=None, updated=time.monotonic(), frames=state["frames"]+1,
                                 center_distance_m=depth.get_distance(raw.shape[1]//2,raw.shape[0]//2))
                    if elapsed >= 1:
                        state["fps"] = round(count/elapsed, 2)
                        count, period_start = 0, time.monotonic()
                    domain=str(color.get_frame_timestamp_domain()).split('.')[-1]
                    latency=received_ns/1e6-color.get_timestamp()
                    info=dict(sequence=state['frames'],timestamp_ns=received_ns,session=camera_session,
                        color_device_timestamp_ms=color.get_timestamp(),depth_device_timestamp_ms=depth.get_timestamp(),
                        timestamp_domain=domain,color_frame_number=color.get_frame_number(),depth_frame_number=depth.get_frame_number())
                    state.update(timestamp_ns=received_ns,camera_latency_ms=round(latency,2) if domain=='global_time' and 0<=latency<=5000 else None)
                    frame_history.append((info,encoded['color'],depth_png))
                    condition.notify_all()
        except Exception as exc:
            logging.exception("Camera capture failed")
            with condition:
                state.update(connected=False, error=str(exc), fps=0)
                images.clear()
                depth_png = None
                condition.notify_all()
        finally:
            if started:
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
        stop.wait(3)


@app.after_request
def no_cache(response):
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    response.headers["X-Content-Type-Options"] = "nosniff"
    return response


@app.get("/")
def index():
    return PAGE


@app.get("/api/status")
def status():
    with condition:
        result = dict(state)
    updated = result.pop("updated")
    result["age_seconds"] = round(time.monotonic()-updated, 3) if updated else None
    return jsonify(result)


@app.get('/api/frames')
def frame_batch():
    # One consistent RGB/depth frameset and timestamp, independently of previews.
    try:after=int(request.args.get('after','-1'))
    except ValueError:return 'invalid sequence',400
    with condition:
        condition.wait_for(lambda: stop.is_set() or (state['connected'] and state['frames']>after),timeout=.3)
        if not state['connected'] or time.monotonic()-state['updated']>.5:return 'Camera unavailable',503
        rows=[r for r in frame_history if r[0]['sequence']>after]
        if after<0:rows=rows[-1:]
        rows=rows[:30]
        meta=dict(state);meta.pop('updated',None)
    output=io.BytesIO()
    with zipfile.ZipFile(output,'w',compression=zipfile.ZIP_STORED) as z:
        z.writestr('metadata.json',json.dumps(meta))
        z.writestr('frames.json',json.dumps([r[0] for r in rows]))
        for info,rgb,depth in rows:
            name=str(info['sequence']);z.writestr(name+'.jpg',rgb)
            if request.args.get('depth')=='1':z.writestr(name+'.png',depth)
    return Response(output.getvalue(),mimetype='application/zip')


@app.get("/stream/<kind>")
def stream(kind):
    if kind not in ("color", "depth"):
        return "Unknown stream", 404

    def generate():
        previous = -1
        while not stop.is_set():
            with condition:
                condition.wait_for(lambda: stop.is_set() or (state["frames"] != previous and kind in images
                                   and time.monotonic()-state["updated"]<3), timeout=15)
                if stop.is_set():
                    return
                if kind not in images or state["frames"] == previous or time.monotonic()-state["updated"]>3:
                    continue
                frame = images[kind]
                previous = state["frames"]
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/snapshot/<kind>.jpg")
def snapshot(kind):
    with condition:
        image = images.get(kind)
        age = time.monotonic()-state["updated"]
    return Response(image, mimetype="image/jpeg") if image and age<3 else ("Camera unavailable", 503)


@app.get("/depth.png")
def raw_depth():
    with condition:
        image = depth_png
        age = time.monotonic()-state["updated"]
    return Response(image, mimetype="image/png") if image and age<3 else ("Camera unavailable", 503)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="10.209.66.60")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--serial", default="261822072968")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--low-bandwidth", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    worker = threading.Thread(target=camera, args=(args.serial,args.fps,args.low_bandwidth), daemon=True)
    worker.start()
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    server = make_server(args.host,args.port,app,threaded=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        worker.join(timeout=12)
        server.server_close()
