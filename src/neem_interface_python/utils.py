from datetime import datetime
from typing import List

from scipy.spatial.transform import Rotation
import dateutil.parser

from neem_interface_python.rosprolog_client import atom


class Datapoint:
    def __init__(self, timestamp: float, frame: str, reference_frame: str, pos: List[float], ori: Rotation,
                 wrench: List[float] = None):
        """
        :param timestamp:
        :param reference_frame: e.g. 'world'
        :param pos: [x,y,z]
        :param ori: [qx,qy,qz,qw]
        :param wrench: [fx,fy,fz,mx,my,mz]
        """
        self.timestamp = timestamp
        self.frame = frame
        self.reference_frame = reference_frame
        self.pos = pos
        self.ori = ori
        self.wrench = wrench

    @staticmethod
    def from_prolog(prolog_dp: dict):
        frame = None # FIXME
        ori = Rotation.from_quat(prolog_dp["term"][2][2])
        return Datapoint(timestamp=prolog_dp["term"][1], frame=frame, reference_frame=prolog_dp["term"][2][0],
                              pos=prolog_dp["term"][2][1], ori=ori)

    def to_knowrob_string(self):
        """
        Convert to a KnowRob pose "[reference_cs, [x,y,z],[qx,qy,qz,qw]]"
        """
        return f"[{atom(self.reference_frame)}, [{self.pos[0]},{self.pos[1]},{self.pos[2]}], [{self.pos[3]}," \
               f"{self.pos[4]},{self.pos[5]},{self.pos[6]}]]"

    @staticmethod
    def from_tf(tf_msg: dict):
        timestamp = dateutil.parser.parse(tf_msg["header"]["stamp"]["$date"]).timestamp()
        frame = tf_msg["child_frame_id"]
        reference_frame = tf_msg["header"]["frame_id"]
        trans = tf_msg["transform"]["translation"]
        pos = [trans["x"], trans["y"], trans["z"]]
        rot = tf_msg["transform"]["rotation"]
        ori = Rotation.from_quat([rot["x"], rot["y"], rot["z"], rot["w"]])
        return Datapoint(timestamp, frame, reference_frame, pos, ori)

