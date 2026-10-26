import json
import math
import time
from std_msgs.msg import Bool, String
from hrc_interfaces.msg import Detection
from .common import Base, QOS, run
class LED(Base):
    def __init__(self):
        super().__init__('led_lighting')
        self.count=self.p('pixel_count',60); self.timeout=self.p('input_timeout_s',0.2)
        self.near=self.p('near_mm',100.); self.far=self.p('far_mm',1000.)
        if self.count<1 or self.count>120 or self.far<=self.near: raise ValueError('Invalid LED geometry')
        self.health=False; self.ht=-float('inf'); self.dt=-float('inf'); self.d=None
        self.serial=None; port=self.p('serial_port','')
        if port:
            import serial
            self.serial=serial.Serial(port,self.p('baudrate',115200),write_timeout=0.05)
        self.pub=self.create_publisher(String,'/led_packet',QOS)
        self.create_subscription(Bool,'/health_status',self.h,QOS)
        self.create_subscription(Detection,'/detection_1',self.detect,QOS)
        self.create_timer(0.1,self.tick)
    def h(self,m): self.health=m.data; self.ht=time.monotonic()
    def detect(self,m):
        self.d=m; self.dt=time.monotonic() if 0<=self.age(m.header)<=self.timeout else -float('inf')
    def tick(self):
        now=time.monotonic(); d=self.d
        if not self.health or now-self.ht>self.timeout or now-self.dt>self.timeout or d is None or not d.valid:
            rgb=[255,0,0]
        elif not d.human_detected: rgb=[0,255,0]
        elif not math.isfinite(d.distance_mm): rgb=[255,0,0]
        else:
            t=max(0.,min(1.,(d.distance_mm-self.near)/(self.far-self.near)))
            # Presence: orange (near) to yellow (far); red reserved for faults.
            rgb=[255,int(64+191*t),0]
        text=json.dumps({'pixels':[[i,*rgb] for i in range(self.count)]},separators=(',',':'))+'\n'
        self.pub.publish(String(data=text))
        if self.serial:
            try: self.serial.write(text.encode('ascii'))
            except Exception as exc: self.get_logger().error(f'LED serial: {exc}')
    def destroy_node(self):
        if self.serial: self.serial.close()
        return super().destroy_node()
def main(): run(LED)
