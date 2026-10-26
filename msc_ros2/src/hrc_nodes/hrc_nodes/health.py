import time
from std_msgs.msg import Bool
from hrc_interfaces.msg import SensorStatus
from .common import Base, QOS, run
from .policy import HealthMonitor
class Health(Base):
    def __init__(self):
        super().__init__('sensor_health')
        self.monitor=HealthMonitor(self.p('minimum_hz',15.),self.p('window_s',1.),self.p('stale_s',0.10))
        self.pub=self.create_publisher(Bool,'/health_status',QOS)
        self.create_subscription(SensorStatus,'/status',self.receive,QOS)
        self.create_timer(0.01,self.tick)
    def receive(self,s):
        if 0 <= self.age(s.header) <= 0.1:
            self.monitor.update(s,time.monotonic())
    def tick(self):
        self.pub.publish(Bool(data=self.monitor.healthy(time.monotonic())))
def main(): run(Health)
