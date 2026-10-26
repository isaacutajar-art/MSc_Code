from dataclasses import dataclass
import struct
SENSOR_COUNT=6
ZONES=64
HEADER_STRUCT=struct.Struct('<I H I I B')
SENSOR_BLOCK_STRUCT=struct.Struct('<B B B B B H I 64H 64B')
EXPECTED_PACKET_SIZE=HEADER_STRUCT.size+6*SENSOR_BLOCK_STRUCT.size
@dataclass(frozen=True)
class SensorBlock:
    sensor_id:int
    fresh:int
    init_ok:int
    range_ok:int
    data_ok:int
    frame_seq:int
    timestamp_ms:int
    distance_mm:tuple
    status:tuple
@dataclass(frozen=True)
class Packet:
    packet_seq:int
    timestamp_ms:int
    sensors:tuple

def parse_packet(data):
    if len(data)!=EXPECTED_PACKET_SIZE: return None
    magic,version,seq,ts,count=HEADER_STRUCT.unpack_from(data)
    if (magic,version,count)!=(0x4C375358,1,6): return None
    blocks=[]
    for i in range(6):
        b=SENSOR_BLOCK_STRUCT.unpack_from(data,HEADER_STRUCT.size+i*SENSOR_BLOCK_STRUCT.size)
        blocks.append(SensorBlock(*b[:7],tuple(b[7:71]),tuple(b[71:135])))
    if sorted(b.sensor_id for b in blocks)!=list(range(1,7)): return None
    return Packet(seq,ts,tuple(blocks))
