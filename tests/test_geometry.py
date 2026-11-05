import unittest
import math
from det_types import CameraModel,Pose,angle_error

class GeometryTests(unittest.TestCase):
    def test_project_unproject_with_boresight_roll_and_heading_wrap(self):
        camera=CameraModel('wide',640,480,2000,2000,319.5,239.5,az_offset_deg=.7,el_offset_deg=-.3,roll_deg=16)
        for az,el in [(359.8,30),(0.2,85),(90,-2)]:
            pose=Pose(1,az,el)
            for xy in [(319.5,239.5),(20,30),(620,450)]:
                bearing,elevation=camera.unproject(*xy,pose)
                point=camera.project(bearing,elevation,pose)
                self.assertAlmostEqual(point[0],xy[0],places=7)
                self.assertAlmostEqual(point[1],xy[1],places=7)
    def test_camera_axes_and_behind(self):
        camera=CameraModel('a',100,100,1000,1000,50,50)
        pose=Pose(0,0,0)
        az,el=camera.unproject(60,40,pose)
        self.assertGreater(az,0);self.assertGreater(el,0)
        self.assertIsNone(camera.project(180,0,pose))
        self.assertEqual(angle_error(1,359),2)
    def test_invalid_geometry(self):
        for fx in [0,-1,float('nan')]:
            with self.assertRaises(ValueError):CameraModel('a',100,100,fx,1,50,50)

    def test_distortion_roundtrip(self):
        camera=CameraModel('wide',640,480,500,500,319.5,239.5,distortion_coeffs=(.1,-.03,.001,-.002,.01))
        pose=Pose(0,345,50)
        for x,y in [(319,240),(100,100),(550,400)]:
            az,el=camera.unproject(x,y,pose)
            actual=camera.project(az,el,pose)
            self.assertAlmostEqual(actual[0],x,places=3)
            self.assertAlmostEqual(actual[1],y,places=3)

    def test_calibrated_boresight_inverse_centers_world_target(self):
        camera=CameraModel('narrow',640,480,50000,50000,319.5,239.5,az_offset_deg=.8,el_offset_deg=-.5,roll_deg=20)
        for az,el in [(0,0),(359.5,30),(16,82)]:
            mount=camera.mount_for_boresight(az,el)
            actual=camera.project(az,el,Pose(0,*mount))
            self.assertAlmostEqual(actual[0],camera.cx,places=5)
            self.assertAlmostEqual(actual[1],camera.cy,places=5)
