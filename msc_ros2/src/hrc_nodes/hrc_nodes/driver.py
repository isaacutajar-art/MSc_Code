import socket
from .common import Base, QOS, run
from .protocol import parse_packet
from hrc_interfaces.msg import Measurement, SensorStatus
class Driver(Base):
    def __init__(self):
        super().__init__('stvl53l7cx_driver')
        self.m = self.create_publisher(Measurement, '/measurement', QOS)
        self.s = self.create_publisher(SensorStatus, '/status', QOS)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.p('udp_bind', '0.0.0.0'), self.p('udp_port', 8888)))
        self.sock.setblocking(False)
        self.last_seq = None
        self.create_timer(0.001, self.poll)
    def poll(self):
        for _ in range(32):
            try:
                raw, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            p = parse_packet(raw)
            if p is None or sorted(b.sensor_id for b in p.sensors) != list(range(1,7)):
                continue
            if self.last_seq is not None and not 0 < ((p.packet_seq-self.last_seq) & 0xffffffff) < 0x80000000:
                continue
            self.last_seq = p.packet_seq
            blocks = sorted(p.sensors, key=lambda b:b.sensor_id)
            m = Measurement(); s = SensorStatus()
            m.header.stamp = self.stamp(); m.header.frame_id = 'tof_ring'
            s.header = m.header
            m.packet_seq = s.packet_seq = p.packet_seq
            m.device_timestamp_ms = p.timestamp_ms
            m.distance_mm = [v for b in blocks for v in b.distance_mm]
            s.zone_status = [v for b in blocks for v in b.status]
            for name in ('fresh','init_ok','range_ok','data_ok','frame_seq'):
                setattr(s, name, [getattr(b,name) for b in blocks])
            self.m.publish(m); self.s.publish(s)
    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()
def main(): run(Driver)
