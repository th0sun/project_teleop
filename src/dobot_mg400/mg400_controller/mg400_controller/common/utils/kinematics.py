#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🔧 Kinematics Calculator
คำนวณ Parallel Link Mechanism สำหรับ MG400

ใช้งาน:
    calc = KinematicsCalculator()
    all_joints = calc.calculate_passive_joints(active_joints)
"""

import numpy as np

class KinematicsCalculator:
    def __init__(self):
        # Joint names ตามลำดับใน URDF
        self.joint_names = [
            'mg400_j1',      # Active
            'mg400_j2_1',    # Active
            'mg400_j2_2',    # Passive (mimic j2_1)
            'mg400_j3',      # Active
            'mg400_j3_1',    # Passive (mimic j2_2 * -1)
            'mg400_j3_2',    # Passive (mimic j3_1)
            'mg400_j4_1',    # Passive (mimic j4_2 * -1)
            'mg400_j4_2',    # Passive (mimic j3)
            'mg400_j5'       # Active
        ]
    
    def calculate_passive_joints(self, active_joints_rad):
        """
        คำนวณ Passive Joints จาก Active Joints
        
        Args:
            active_joints_rad: [j1, j2_1, j3, j5] (4 values)
        
        Returns:
            dict: {'names': [...], 'positions': [...]}
        """
        # Active Joints
        j1 = active_joints_rad[0]
        j2_1 = active_joints_rad[1]
        j3 = active_joints_rad[2]
        j5 = active_joints_rad[3]
        
        # Passive Joints (ตามสูตรใน URDF)
        j2_2 = j2_1           # mimic j2_1
        j3_2 = -j2_1          # mimic j2_2 * -1
        j3_1 = -j2_1          # mimic j3_2
        j4_2 = j3             # mimic j3
        j4_1 = -j3            # mimic j4_2 * -1
        
        # รวมทั้งหมด (ลำดับตาม URDF)
        all_positions = [
            j1,    # mg400_j1
            j2_1,  # mg400_j2_1
            j2_2,  # mg400_j2_2
            j3,    # mg400_j3
            j3_1,  # mg400_j3_1
            j3_2,  # mg400_j3_2
            j4_1,  # mg400_j4_1
            j4_2,  # mg400_j4_2
            j5     # mg400_j5
        ]
        
        return {
            'names': self.joint_names,
            'positions': all_positions
        }
    
    def validate_sanity(self, current_joints, new_joints, threshold=0.5):
        """
        ตรวจสอบความสมเหตุสมผลของข้อมูล
        (กันค่า Jump ที่ผิดปกติ)
        
        Returns:
            True ถ้าข้อมูลดูปกติ
        """
        if np.all(current_joints == 0):
            # ครั้งแรกยังไม่มีข้อมูล
            return True
        
        diff = np.abs(new_joints - current_joints)
        if np.any(diff > threshold):
            # กระโดดมากเกินไป
            return False
        
        return True

    def forward_kinematics(self, joints_deg):
        """
        คำนวณ Forward Kinematics ของ MG400
        
        Args:
            joints_deg: [j1, j2, j3, j4] in DEGREES (same as Unity input)
        
        Returns:
            np.array [x, y, z, rx, ry, rz] in mm
            None if outside workspace
        """
        # MG400 Link Constants (mm) - ported from MG400_Mock/kinematics_mg400.py
        LINK1 = np.array([43.0,  0.0,   0.0])
        LINK2 = np.array([0.0,   0.0, 175.0])
        LINK3 = np.array([175.0, 0.0,   0.0])
        LINK4 = np.array([66.0,  0.0, -57.0])

        j1, j2, j3, j4 = float(joints_deg[0]), float(joints_deg[1]), \
                          float(joints_deg[2]), float(joints_deg[3])

        def rot_y(vec, angle_deg):
            a = np.deg2rad(angle_deg)
            R = np.array([[np.cos(a), 0, np.sin(a)],
                          [0,         1, 0          ],
                          [-np.sin(a),0, np.cos(a)  ]])
            return R @ vec

        def rot_z(vec, angle_deg):
            a = np.deg2rad(angle_deg)
            R = np.array([[np.cos(a), -np.sin(a), 0],
                          [np.sin(a),  np.cos(a), 0],
                          [0,          0,         1]])
            return R @ vec

        pos = LINK1 + rot_y(LINK2, j2) + rot_y(LINK3, j3) + LINK4
        px, py, pz = rot_z(pos, j1)
        rx = j1 + j4   # yaw of end-effector

        return np.array([px, py, pz, rx, 0.0, 0.0])