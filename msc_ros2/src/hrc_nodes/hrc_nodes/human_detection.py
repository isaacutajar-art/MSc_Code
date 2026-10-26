import math
import threading
import time
import numpy as np
import torch
from hrc_interfaces.msg import Measurement, SensorStatus, Detection, Velocity
from .common import Base, QOS, run
from .detection_core import RuntimeModel, load_reference_map, HumanDetector, HumanClusterTracker, RTDEStateBuffer
class HumanDetection(Base):
    def __init__(self):
        super().__init__('human_detection')
        paths=[self.p(k,'') for k in ('model_file','scaler_x_file','scaler_distance_file','reference_csv')]
        device=torch.device(self.p('device','cpu'))
        self.model=RuntimeModel(*paths[:3],device)
        p95,usable=load_reference_map(paths[3])
        self.detector=HumanDetector(p95,usable,self.p('distance_threshold',3.),self.p('cluster_size',3),
            self.p('on_window',2),self.p('on_required',1),self.p('off_required',5))
        self.tracker=HumanClusterTracker(max_missed_frames=5)
        self.stop=threading.Event()
        self.rtde=RTDEStateBuffer(self.p('robot_ip','192.168.1.102'),self.p('rtde_hz',500.),self.stop)
        self.rtde.start()
        self.match_s=self.p('max_robot_match_ms',50.)/1000.
        self.pub=self.create_publisher(Detection,'/detection_1',QOS)
        self.vpub=self.create_publisher(Velocity,'/velocity_1',QOS)
        self.pending={}; self.last=None; self.last_seq=None
        self.create_subscription(Measurement,'/measurement',lambda m:self.receive(m,0),QOS)
        self.create_subscription(SensorStatus,'/status',lambda m:self.receive(m,1),QOS)
    def receive(self,msg,kind):
        entry=self.pending.setdefault(msg.packet_seq,[None,None,time.perf_counter()])
        entry[kind]=msg
        if len(self.pending)>10:
            self.pending.pop(next(iter(self.pending)))
        if entry[0] is None or entry[1] is None: return
        self.pending.pop(msg.packet_seq,None)
        if self.last_seq is not None and not 0<((msg.packet_seq-self.last_seq)&0xffffffff)<0x80000000: return
        self.last_seq=msg.packet_seq
        m,s,received=entry
        out=Detection(); out.header=m.header; out.packet_seq=m.packet_seq
        out.distance_mm=out.velocity_mps=float('nan'); out.track_id=-1
        try:
            if not 0<=self.age(m.header)<=0.15: raise ValueError('Stale sensor frame')
            state=self.rtde.nearest(received)
            if state is None or abs(state.sample_perf_s-received)>self.match_s:
                raise ValueError('No matched robot pose')
            distances=np.asarray(m.distance_mm,dtype=np.float32)
            statuses=np.asarray(s.zone_status,dtype=np.int16)
            predicted,ms=self.model.predict(state)
            result,debug=self.detector.detect(distances,statuses,predicted,ms,time.perf_counter())
            current,_,_=self.tracker.update(debug['qualifying_clusters'],distances,received)
            closest=min(current,key=lambda t:t.min_distance_mm) if current else None
            out.valid=True; out.human_detected=result.state=='HUMAN'
            if not out.human_detected: self.last=None
            elif closest is not None:
                v=float(closest.velocity_filtered_mps)
                self.last=(closest.min_distance_mm,v,closest.sensor_id,closest.track_id)
            if out.human_detected:
                out.temporally_held=closest is None
                if self.last is not None:
                    d,v,sid,tid=self.last
                    out.distance_mm=float(d); out.velocity_mps=max(0.,v) if out.temporally_held and math.isfinite(v) else v
                    out.sensor_id=sid; out.track_id=tid
                else: out.valid=False
        except Exception as exc:
            self.get_logger().error(str(exc))
        self.pub.publish(out)
        v=Velocity(); v.header=out.header; v.packet_seq=out.packet_seq
        v.closing_mps=out.velocity_mps; v.valid=out.valid and math.isfinite(v.closing_mps) and not out.temporally_held
        self.vpub.publish(v)
    def destroy_node(self):
        self.stop.set(); self.rtde.join(timeout=2.)
        return super().destroy_node()
def main(): run(HumanDetection)
