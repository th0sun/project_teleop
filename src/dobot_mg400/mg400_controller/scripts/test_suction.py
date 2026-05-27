#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ทดสอบพอร์ต Digital Output (DO) ของบอร์ด MG400
เพื่อให้หาว่าหัวดูด (Suction Cup) หรือ Gripper ต่ออยู่ที่พอร์ตไหน
"""

import socket

ROBOT_IP = "192.168.1.6"  # หรือ 192.168.2.6 ขึ้นอยู่กับการตั้งค่า LAN ของหุ่น
ROBOT_PORT = 29999

def test_do_port():
    print("🔌 กำลังเชื่อมต่อกับ MG400...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    
    try:
        s.connect((ROBOT_IP, ROBOT_PORT))
        print("✅ เชื่อมต่อสำเร็จ!")
    except Exception as e:
        print(f"❌ ไม่สามารถเชื่อมต่อกับหุ่นยนต์ที่ IP {ROBOT_IP} ได้: {e}")
        return

    print("\n" + "="*50)
    print("🛠️  เครื่องมือทดสอบพอร์ต Digital Output (DO)")
    print("💡 พิมพ์หมายเลขพอร์ต (1-18) เพื่อสลับสถานะ เปิด/ปิด")
    print("💡 พิมพ์ 'q' เพื่อออก")
    print("="*50 + "\n")

    port_status = {i: 0 for i in range(1, 19)}

    while True:
        try:
            user_input = input("👉 ระบุหมายเลขพอร์ต (1-18) หรือ q: ")
            
            if user_input.lower() == 'q':
                break
                
            port = int(user_input)
            if port < 1 or port > 18:
                print("⚠️ พอร์ตต้องอยู่ระหว่าง 1 ถึง 18")
                continue
                
            # สลับสถานะของพอร์ตนี้ (0 -> 1, 1 -> 0)
            port_status[port] = 1 - port_status[port]
            status = port_status[port]
            
            # ส่งคำสั่ง DOInstant(index, status)
            cmd = f"DOInstant({port}, {status})"
            print(f"📡 กำลังส่งคำสั่ง: {cmd}")
            
            s.sendall(f"{cmd}\n".encode('utf-8'))
            
            # รอรับผลลัพธ์
            response = s.recv(1024).decode('utf-8').strip()
            print(f"🤖 หุ่นตอบกลับ: {response}")
            
            if status == 1:
                print(f"🔊 เปิดพอร์ต {port} แล้ว (ลองฟังเสียงปั๊มว่าทำงานไหม)")
            else:
                print(f"🔇 ปิดพอร์ต {port} แล้ว")
                
            print("-" * 30)
            
        except ValueError:
            print("⚠️ กรุณาพิมพ์ตัวเลขปรกติ")
        except Exception as e:
            print(f"❌ เกิดข้อผิดพลาด: {e}")
            break

    print("ปิดการเชื่อมต่อ...")
    s.close()
    print("ลาก่อน!")

if __name__ == "__main__":
    test_do_port()
