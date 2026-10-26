import math
import time
from hrc_interfaces.msg import Detection, SpeedFactor
from .common import Base, QOS, run
from . import ssm_core as model
class SpeedScaling(Base):
    def __init__(self):
        super().__init__('speed_scaling')
        for key in ('TR_S','TS_S','C_M','ZR_M','ZD_FRACTION','VMAX_MPS',
                    'WORKSPACE_MAX_DISTANCE_M','DESIGN_VH_MPS','SAFETY_FACTOR','SPEED_STEP_MPS','NEAR_LOSS_HOLD_DISTANCE_M'):
            value=float(self.p(key.lower(),getattr(model,key)))
            if not math.isfinite(value) or value<0: raise ValueError(key)
            setattr(model,key,value)
        if model.VMAX_MPS<=0 or model.SPEED_STEP_MPS<=0 or model.SAFETY_FACTOR<=0:
            raise ValueError('Speed, step and factor must be positive')
        # Pass configured search resolution explicitly (original default is bound at definition).
        model.VPROG_MPS=model.find_max_safe_speed(model.WORKSPACE_MAX_DISTANCE_M,model.DESIGN_VH_MPS,model.VMAX_MPS,model.SPEED_STEP_MPS)
        self.pub=self.create_publisher(SpeedFactor,'/speed_scaling_factor',QOS)
        self.previous=False; self.distance=None; self.factor=0.; self.hold=False
        self.create_subscription(Detection,'/detection_1',self.receive,QOS)
    def receive(self,d):
        out=SpeedFactor(); out.header=d.header; out.packet_seq=d.packet_seq
        out.valid=d.valid and 0<=self.age(d.header)<=0.5
        if not out.valid: self.factor=0.
        elif not d.human_detected:
            if self.previous and self.distance is not None and self.distance<model.NEAR_LOSS_HOLD_DISTANCE_M: self.hold=True
            self.previous=False
            if not self.hold: self.factor=1.
        elif not math.isfinite(d.distance_mm) or d.distance_mm<0:
            out.valid=False; self.factor=0.
        else:
            self.previous=True; self.hold=False; self.distance=d.distance_mm/1000.
            v=d.velocity_mps if math.isfinite(d.velocity_mps) else 0.
            speed=model.find_max_safe_speed(self.distance,v,model.VPROG_MPS,model.SPEED_STEP_MPS)
            self.factor=speed/model.VPROG_MPS if model.VPROG_MPS>0 else 0.
        out.factor=float(self.factor); self.pub.publish(out)
def main(): run(SpeedScaling)
