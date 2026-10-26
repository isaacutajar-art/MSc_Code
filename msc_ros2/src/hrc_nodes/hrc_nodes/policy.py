from collections import deque
import math
class HealthMonitor:
    def __init__(self, minimum_hz=15., window_s=1., stale_s=0.10):
        self.hz=minimum_hz; self.window=window_s; self.stale=stale_s
        self.times=[deque() for _ in range(6)]; self.seq=[None]*6
        self.ok=[False]*6
    def update(self, status, now):
        for i in range(6):
            self.ok[i]=bool(status.init_ok[i] and status.range_ok[i] and status.data_ok[i])
            seq=status.frame_seq[i]
            delta=1 if self.seq[i] is None else (seq-self.seq[i]) & 0xffff
            if status.fresh[i] and 0 < delta < 0x8000 and self.ok[i]:
                self.times[i].append(now); self.seq[i]=seq
    def healthy(self, now):
        for i, q in enumerate(self.times):
            while q and now-q[0]>self.window: q.popleft()
            if not self.ok[i] or len(q)<2 or now-q[-1]>self.stale:
                return False
            # Include current silence in the denominator so a stalled stream degrades.
            if (len(q)-1)/max(now-q[0],1e-9) < self.hz:
                return False
        return True

def choose_factor(health, health_age, factor, factor_age, timeout):
    if not health or health_age>timeout or factor_age>timeout:
        return 0.
    return factor if math.isfinite(factor) and 0.<=factor<=1. else 0.
