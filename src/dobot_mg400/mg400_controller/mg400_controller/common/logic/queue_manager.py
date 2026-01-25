#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📊 Queue Manager
จัดการคิวคำสั่งด้วยระบบ Leaky Bucket

ใช้งาน:
    queue_mgr = QueueManager(logger)
    if queue_mgr.can_add_command():
        queue_mgr.add_command()
"""

import time
from mg400_controller.common.config.motion_config import QUEUE_LIMIT, QUEUE_LEAK_INTERVAL

class QueueManager:
    def __init__(self, logger):
        self.logger = logger
        self.queue_depth = 0
        self.last_leak_time = time.time()
        self.last_warning_time = 0
    
    def can_add_command(self):
        """ตรวจสอบว่าสามารถเพิ่มคำสั่งได้หรือไม่"""
        if self.queue_depth >= QUEUE_LIMIT:
            now = time.time()
            # แสดง warning ทุก 1 วินาที
            if now - self.last_warning_time > 1.0:
                self.logger.warn(
                    f"⚠️ Queue Full ({self.queue_depth}/{QUEUE_LIMIT}) - Skipping"
                )
                self.last_warning_time = now
            return False
        return True
    
    def add_command(self):
        """เพิ่มคำสั่งเข้าคิว"""
        self.queue_depth += 1
    
    def leak_queue(self):
        """
        ระบบรั่ว (Leaky Bucket)
        ลดคิวทีละ 1 ตามช่วงเวลาที่กำหนด
        
        Returns:
            True ถ้ามีการรั่ว, False ถ้ายังไม่ถึงเวลา
        """
        now = time.time()
        dt = now - self.last_leak_time
        
        if dt >= QUEUE_LEAK_INTERVAL:
            if self.queue_depth > 0:
                self.queue_depth -= 1
            self.last_leak_time = now
            return True
        
        return False
    
    def get_depth(self):
        """ดึงค่า queue depth ปัจจุบัน"""
        return self.queue_depth
    
    def reset(self):
        """รีเซ็ตคิว"""
        self.queue_depth = 0
        self.last_leak_time = time.time()