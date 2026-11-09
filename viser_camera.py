"""Camera page uses the full Viser scene; it never reopens RealSense USB."""
import io
import json
import threading
import time
from urllib.request import urlopen
import numpy as np
from PIL import Image

class CameraBridge:
    def __init__(self,app,base='http://10.209.66.60:8080'):
        self.app=app;self.base=base;self.status={};self.last_error='';self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def get(self,path):
        with urlopen(self.base+path,timeout=.7) as response:return response.read()
    def run(self):
        last_status=0.
        while not self.stop.is_set():
            try:
                if time.monotonic()-last_status>.5:
                    self.status=json.loads(self.get('/api/status'));last_status=time.monotonic()
                s=self.status
                live=bool(s.get('connected')) and s.get('age_seconds') is not None and s['age_seconds']<2
                self.app.camera_status.content=(f'**{"在线" if live else "未就绪"}** · USB {s.get("usb","—")} · {s.get("fps",0):.1f} fps'
                    f'\n\n{s.get("profile","")} · 帧龄 {s.get("age_seconds") if live else "—"} 秒\n\n{s.get("error") or ""}')
                if self.app.page_name=='相机':
                    frame=None
                    if live and self.app.camera_show.value and self.app.server.get_clients():
                        name='color' if self.app.camera_view.value=='彩色' else 'depth'
                        frame=np.asarray(Image.open(io.BytesIO(self.get('/snapshot/'+name+'.jpg'))).convert('RGB'))
                    if frame is not None:
                        h,w=frame.shape[:2];aspect=self.app.camera_aspect
                        if w/h<aspect:
                            extra=max(0,round(h*aspect)-w);frame=np.pad(frame,((0,0),(extra//2,extra-extra//2),(0,0)))
                        else:
                            extra=max(0,round(w/aspect)-h);frame=np.pad(frame,((extra//2,extra-extra//2),(0,0),(0,0)))
                    with self.app.view_lock:
                        if self.app.page_name=='相机':self.app.server.scene.set_background_image(frame,format='jpeg',jpeg_quality=80)
                self.last_error=''
            except Exception as exc:
                self.last_error=str(exc);self.app.camera_status.content='**相机连接中断**\n\n'+str(exc)
                with self.app.view_lock:
                    if self.app.page_name=='相机':self.app.server.scene.set_background_image(None)
            self.stop.wait(.1)
