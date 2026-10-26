import time
from std_msgs.msg import Bool, Float64
from ur_msgs.srv import SetSpeedSliderFraction
from hrc_interfaces.msg import SpeedFactor
from .common import Base, QOS, run
from .policy import choose_factor
class Decision(Base):
    def __init__(self):
        super().__init__('decision_making')
        self.enabled=self.p('enable_robot',False)
        self.timeout=self.p('input_timeout_s',0.2)
        self.ack_timeout=self.p('service_timeout_s',0.2)
        self.health=False; self.ht=-float('inf'); self.ft=-float('inf'); self.factor=0.
        self.pending=None; self.sent=None; self.fault=False
        self.pub=self.create_publisher(Float64,'/robot_speed_command',QOS)
        self.client=self.create_client(SetSpeedSliderFraction,self.p('speed_service','/io_and_status_controller/set_speed_slider'))
        self.create_subscription(Bool,'/health_status',self.on_health,QOS)
        self.create_subscription(SpeedFactor,'/speed_scaling_factor',self.on_factor,QOS)
        self.create_timer(0.01,self.tick)
    def on_health(self,m): self.health=m.data; self.ht=time.monotonic()
    def on_factor(self,m):
        self.factor=m.factor if m.valid and 0<=self.age(m.header)<=self.timeout else 0.
        self.ft=time.monotonic()
    def tick(self):
        now=time.monotonic()
        factor=choose_factor(self.health,now-self.ht,self.factor,now-self.ft,self.timeout)
        if self.pending is not None:
            if self.pending.done():
                try:
                    if not self.pending.result().success: self.fault=True
                except Exception: self.fault=True
                self.pending=None
            elif now-self.sent>self.ack_timeout:
                self.fault=True
        if self.fault: factor=0.
        self.pub.publish(Float64(data=float(factor)))
        if not self.enabled: return
        if not self.client.service_is_ready():
            self.fault=True
            return
        # One outstanding call: never queue old nonzero commands behind a stop.
        if self.pending is None:
            req=SetSpeedSliderFraction.Request(); req.speed_slider_fraction=float(factor)
            self.pending=self.client.call_async(req); self.sent=now
        # A timed-out in-flight request cannot be safely cancelled at the robot.
        # Keep fault latched and send zero once it resolves; operator must restart.
def main(): run(Decision)
