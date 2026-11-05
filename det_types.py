"""Mac-monotonic message contracts and calibrated pinhole geometry, ENU bearing."""
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Tuple
import math
import numpy as np


def angle_error(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


@dataclass
class Pose:
    timestamp: float
    az_deg: float
    el_deg: float
    valid: bool = True


@dataclass
class CameraModel:
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    az_offset_deg: float = 0.0
    el_offset_deg: float = 0.0
    roll_deg: float = 0.0
    calibrated: bool = False
    distortion_coeffs: Tuple[float, float, float, float, float] = (0.0,0.0,0.0,0.0,0.0)

    def __post_init__(self):
        values = (self.fx, self.fy, self.cx, self.cy, self.az_offset_deg,
                  self.el_offset_deg, self.roll_deg)
        if not all(math.isfinite(v) for v in values) or min(self.fx, self.fy) <= 0:
            raise ValueError('Camera geometry must be finite with positive focal lengths')
        if len(self.distortion_coeffs) != 5 or not all(math.isfinite(v) for v in self.distortion_coeffs):
            raise ValueError('Use five finite Brown-Conrady distortion coefficients')
        if isinstance(self.width,bool) or isinstance(self.height,bool) or not isinstance(self.width,int) or not isinstance(self.height,int):
            raise ValueError('Camera dimensions must be integers')
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Camera dimensions must be positive')

    def basis(self, pose):
        # Intrinsic boresight rotations in the gimbal tangent frame.
        az, el = math.radians(pose.az_deg), math.radians(pose.el_deg)
        f = np.array([math.sin(az)*math.cos(el), math.cos(az)*math.cos(el), math.sin(el)])
        r = np.array([math.cos(az), -math.sin(az), 0.0])
        u = np.cross(r, f)
        yaw, pitch, roll = map(math.radians, (self.az_offset_deg, self.el_offset_deg, self.roll_deg))
        f1 = math.cos(yaw)*f + math.sin(yaw)*r
        r1 = math.cos(yaw)*r - math.sin(yaw)*f
        f2 = math.cos(pitch)*f1 + math.sin(pitch)*u
        u2 = math.cos(pitch)*u - math.sin(pitch)*f1
        return f2, math.cos(roll)*r1 + math.sin(roll)*u2, -math.sin(roll)*r1 + math.cos(roll)*u2

    def project(self, az_deg, el_deg, pose):
        az, el = map(math.radians, (az_deg, el_deg))
        ray = np.array([math.sin(az)*math.cos(el), math.cos(az)*math.cos(el), math.sin(el)])
        f,r,u = self.basis(pose)
        z = float(ray@f)
        if z <= 0:
            return None
        x,y=float(ray@r)/z,-float(ray@u)/z
        k1,k2,p1,p2,k3=self.distortion_coeffs
        radius=x*x+y*y;radial=1+k1*radius+k2*radius**2+k3*radius**3
        xd=x*radial+2*p1*x*y+p2*(radius+2*x*x)
        yd=y*radial+p1*(radius+2*y*y)+2*p2*x*y
        return self.cx+self.fx*xd,self.cy+self.fy*yd

    def unproject(self, x, y, pose):
        f,r,u = self.basis(pose)
        xn,yn=(x-self.cx)/self.fx,(y-self.cy)/self.fy
        if any(self.distortion_coeffs):
            import cv2
            matrix=np.array([[self.fx,0,self.cx],[0,self.fy,self.cy],[0,0,1]],dtype=float)
            xn,yn=cv2.undistortPoints(np.array([[[x,y]]],dtype=float),matrix,np.array(self.distortion_coeffs)).reshape(2)
        ray = f + xn*r - yn*u
        ray /= np.linalg.norm(ray)
        return math.degrees(math.atan2(ray[0],ray[1])) % 360.0, math.degrees(math.asin(float(np.clip(ray[2],-1,1))))

    def mount_for_boresight(self, az_deg, el_deg):
        """Solve the mount pose that puts this world ray on the optical axis."""
        if not self.az_offset_deg and not self.el_offset_deg:
            return az_deg%360,el_deg
        values=np.array([az_deg,el_deg],dtype=float)
        def residual(v):
            az,el=self.unproject(self.cx,self.cy,Pose(0,float(v[0]),float(v[1])))
            return np.array([angle_error(az,az_deg),el-el_deg])
        for _ in range(12):
            error=residual(values)
            if np.linalg.norm(error)<1e-9:return float(values[0]%360),float(values[1])
            epsilon=1e-4
            jacobian=np.column_stack([(residual(values+np.eye(2)[i]*epsilon)-error)/epsilon for i in range(2)])
            try: values-=np.linalg.solve(jacobian,error)
            except np.linalg.LinAlgError:raise ValueError('Optical boresight is singular for this mount direction')
        raise ValueError('Optical boresight inverse did not converge')

    def inside(self, point, margin=0):
        return point is not None and margin <= point[0] < self.width-margin and margin <= point[1] < self.height-margin


@dataclass
class Frame:
    camera: str
    timestamp: float
    image: np.ndarray
    pose: Pose
    sequence: int = 0


@dataclass
class Detection:
    timestamp: float
    camera: str
    x: float
    y: float
    bearing_deg: float
    elevation_deg: float
    confidence: float
    identity: str = 'rocket'
    aim_kind: str = 'body_centroid_uncertain_com'
    bbox: Optional[Tuple[int,int,int,int]] = None
    angular_sigma_deg: float = 0.1


@dataclass
class Phase:
    name: str = 'unknown'
    confidence: float = 0.0
    source: str = 'unknown'


@dataclass
class VisionResult:
    timestamp: float
    selected: Optional[Detection] = None
    candidates: Dict[str,List[Detection]] = field(default_factory=dict)
    phase: Phase = field(default_factory=Phase)
    narrow_found: bool = False
    wide_found: bool = False
    warnings: List[str] = field(default_factory=list)


@dataclass
class Prediction:
    timestamp: float
    az_deg: float = 0.0
    el_deg: float = 0.0
    az_rate_deg_s: float = 0.0
    el_rate_deg_s: float = 0.0
    valid: bool = False
    source: str = 'none'
    uncertainty_deg: float = 180.0
    reason: str = ''
    phase: str = 'unknown'
    lead_time_s: float = 0.0
    image_error_az_deg: Optional[float] = None
    image_error_el_deg: Optional[float] = None
    image_timestamp: Optional[float] = None


@dataclass
class Command:
    timestamp: float
    az_velocity_deg_s: float = 0.0
    el_velocity_deg_s: float = 0.0
    valid_until: float = 0.0
    enabled: bool = False
    reason: str = ''
    sequence: int = 0
