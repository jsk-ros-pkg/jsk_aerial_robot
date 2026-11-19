#!/usr/bin/env python3
"""Numerical regressions against the actual expanded grape_with_arm URDF."""
import pathlib
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'script'))
from arm_world_core import ArmKinematics, transform


class ArmWorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = pathlib.Path(__file__).resolve().parents[1] / 'robots/grape_with_arm/gimbalrotor.urdf.xacro'
        cls.xml = subprocess.check_output(['xacro', str(model)], text=True)
        cls.kin = ArmKinematics(cls.xml, velocity_cap=.6)
        cls.home = np.array([0., -.6, .9, -.3])

    def test_tool_has_four_positioning_joints(self):
        movable = [name for name, kind, *_ in self.kin.tip_chain if kind != 'fixed']
        self.assertEqual(movable, self.kin.names)
        self.assertGreater(np.linalg.svd(self.kin.jacobian(self.home)[:3],compute_uv=False)[-1], .01)

    def test_xl430_12v_catalog_limits_used_without_hidden_cap(self):
        k = ArmKinematics(self.xml)
        np.testing.assert_allclose(k.velocity, np.full(4, 61*2*np.pi/60), rtol=1e-10)
        np.testing.assert_allclose(k.effort, np.full(4, 1.5))
        joints = {j.get('name'): j for j in ET.fromstring(self.xml).findall('joint')}
        for i in range(1,6):
            limit = joints['arm_joint%d_joint1'%i].find('limit')
            self.assertAlmostEqual(float(limit.get('velocity')),61*2*np.pi/60)
            self.assertEqual(float(limit.get('effort')),1.5)

    def test_small_base_motion_compensated(self):
        target = self.kin.fk(self.home)
        base = transform([.008,-.008,.004], Rotation.from_euler('xyz',[.01,-.01,.02]).as_matrix())
        q = self.home.copy()
        initial = np.linalg.norm((base @ self.kin.fk(q))[:3,3]-target[:3,3])
        for unused in range(180):
            updated, singular, saturated = self.kin.step(q,base,target,1/30,self.home)
            self.assertLessEqual(float(np.max(np.abs(updated-q))), .6/30+1e-10)
            q = updated
        final = np.linalg.norm((base @ self.kin.fk(q))[:3,3]-target[:3,3])
        self.assertLess(final, .0005)
        self.assertLess(final, initial*.05)

    def test_unreachable_target_bounded(self):
        target = transform([3,3,3])
        q = self.home.copy()
        for unused in range(100):
            next_q, singular, saturated = self.kin.step(q,np.eye(4),target,1/30,self.home)
            self.assertTrue(np.isfinite(next_q).all())
            self.assertTrue(np.all(next_q >= self.kin.lower))
            self.assertTrue(np.all(next_q <= self.kin.upper))
            self.assertLessEqual(float(np.max(np.abs(next_q-q))),.6/30+1e-10)
            q = next_q

    def test_position_priority_under_orientation_conflict(self):
        target = self.kin.fk(self.home)
        target[:3,:3] = Rotation.from_euler('xyz',[.1,-.1,.1]).as_matrix() @ target[:3,:3]
        q = self.home.copy()
        for unused in range(120):
            q, *_ = self.kin.step(q,np.eye(4),target,1/30,self.home)
        self.assertLess(np.linalg.norm(self.kin.fk(q)[:3,3]-target[:3,3]),.001)


if __name__ == '__main__':
    unittest.main()
