"""Persistent fault/event history and expiring, cancellable recovery requests."""
import json
from pathlib import Path
import time


class EventLog:
    def __init__(self,path):
        self.path=Path(path);self.path.mkdir(parents=True,exist_ok=True)
        self.events=[];self.last_fault=None
        try:
            self.last_fault=json.loads((self.path/'last-fault.json').read_text())
        except (OSError,ValueError):pass
        try:
            self.events=[json.loads(s) for s in (self.path/'events.jsonl').read_text().splitlines()[-100:]]
        except (OSError,ValueError):pass

    def add(self,message,kind='info',detail=None):
        event=dict(time=time.time(),kind=kind,message=message,detail=detail or {})
        self.events.append(event);self.events=self.events[-100:]
        with (self.path/'events.jsonl').open('a') as f:f.write(json.dumps(event,ensure_ascii=False)+'\n')
        return event

    def fault(self,fault):
        self.last_fault=fault
        p=self.path/'last-fault.tmp';p.write_text(json.dumps(fault,ensure_ascii=False));p.replace(self.path/'last-fault.json')
        self.add(fault['message'],'fault',fault.get('detail'))


class RecoveryRequest:
    def __init__(self,clock=time.monotonic):self.clock=clock;self.pending=None
    def request(self,client):self.pending=dict(client=client,deadline=self.clock()+25.)
    def cancel(self):self.pending=None
    def take(self,clients):
        p=self.pending;self.pending=None
        if p is None:return None
        if self.clock()>p['deadline'] or p['client'] not in clients:return None
        return p['client']
    def expired(self):return self.pending is not None and self.clock()>self.pending['deadline']
