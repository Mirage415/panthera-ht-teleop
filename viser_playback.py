"""Read-only MCAP seeking. Playback never talks to the motor controller."""
import io
import json
from pathlib import Path
import queue
import threading
from mcap.reader import make_reader
from PIL import Image
import numpy as np
from mcap_io import decode_image

class Playback:
    def __init__(self, output):
        self.output=Path(output);self.requests=queue.Queue(maxsize=1);self.closed=threading.Event()
        self.snapshot={};self.thread=threading.Thread(target=self.run,daemon=True,name='episode-reader');self.thread.start()
    def seek(self, name, fraction):
        if not name or Path(name).name!=name:raise ValueError('请选择录制')
        if not 0<=fraction<=1:raise ValueError('无效回放位置')
        try:self.requests.get_nowait()
        except queue.Empty:pass
        self.requests.put_nowait((name,fraction))
    def read(self, name, fraction):
        folder=self.output/name
        if folder.is_symlink() or (folder/'data.mcap').is_symlink():raise ValueError('不读取链接文件')
        meta=json.loads((folder/'metadata.json').read_text())
        if meta.get('recording'):raise ValueError('请先结束录制')
        with (folder/'data.mcap').open('rb') as handle:
            reader=make_reader(handle);summary=reader.get_summary()
            if not summary or not summary.statistics:raise ValueError('文件没有完整索引')
            stats=summary.statistics;start=stats.message_start_time;end=stats.message_end_time
            stamp=start+round((end-start)*fraction);image=None;rows={}
            for schema,channel,msg in reader.iter_messages(start_time=max(start,stamp-200_000_000),
                    end_time=min(end+1,stamp+200_000_000),topics=['/camera/color/image','/leader/joint_states','/follower/joint_states']):
                if channel.topic not in rows or abs(msg.log_time-stamp)<rows[channel.topic][0]:
                    if channel.topic=='/camera/color/image':value=decode_image(schema,channel,msg)
                    else:value=json.loads(msg.data)
                    rows[channel.topic]=(abs(msg.log_time-stamp),value,msg.log_time)
            if '/camera/color/image' in rows:image=np.asarray(Image.open(io.BytesIO(rows['/camera/color/image'][1])).convert('RGB'))
            return dict(name=name,seconds=(stamp-start)/1e9,duration=(end-start)/1e9,image=image,
                joints={key:value[1] for key,value in rows.items() if 'joint_states' in key},metadata=meta)
    def run(self):
        while not self.closed.is_set():
            try:name,fraction=self.requests.get(timeout=.2)
            except queue.Empty:continue
            try:self.snapshot=self.read(name,fraction)
            except Exception as exc:self.snapshot=dict(error=str(exc))
