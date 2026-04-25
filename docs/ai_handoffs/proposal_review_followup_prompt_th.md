# Thai Prompt For Proposal Re-Check

ไฟล์นี้คือ prompt ภาษาไทยสำหรับส่งกลับไปให้ AI อีกตัวตรวจ proposal
สถาปัตยกรรมรอบล่าสุดใหม่อีกครั้ง โดยตั้งใจให้มัน **ไม่เชื่อ review findings
หรือความเห็นใดๆ แบบอัตโนมัติ** แต่ต้องไปหาหลักฐานจริงจากทั้ง repository และ
แหล่งข้อมูลภายนอกก่อน แล้วค่อยยืนยันหรือโต้แย้งอย่างมีเหตุผล

## Prompt พร้อมใช้

```text
คุณกำลังทำงานอยู่ใน repository นี้:
/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop

branch ที่กำลังใช้ต่อคือ:
feat/multi-robot-teaching-architecture

ภารกิจรอบนี้ไม่ใช่การรีบแก้ตาม review findings แบบเชื่อตรงๆ
แต่คือการ "ตรวจสอบอีกครั้งอย่างเข้มงวด" ว่า review findings เหล่านั้นถูกต้องจริงหรือไม่
และถ้าถูกต้อง ควรแก้อย่างไรให้ดีที่สุดโดยอิงหลักฐาน ไม่ใช่ intuition อย่างเดียว

ให้ถือ review findings ด้านล่างเป็นเพียง:
- hypotheses
- pressure points
- possible risks

ไม่ใช่คำสั่งที่ต้องเชื่อตามทันที

## เอกสารที่ต้องอ่านก่อน

ให้อ่านไฟล์เหล่านี้ก่อนตามลำดับ:

1. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/project_objective.md
2. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/current_refactor_status.md
3. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_brief.md
4. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_proposal.md
5. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_revision.md
6. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/skills/multi_robot_architecture/SKILL.md
7. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/controller_continuation_guide.md
8. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/teleop_command_logic.md

จากนั้นค่อยอ่านโค้ดและโครงสร้างที่เกี่ยวข้องเท่าที่จำเป็น เช่น:

- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/test
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/MG400_Mock
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/Dobot_TCP_IP_Python_V4

## Review findings ที่ต้องตรวจสอบใหม่

ให้ตรวจสอบ findings เหล่านี้ใหม่อย่างเป็นอิสระ:

1. canonical IR ยังไม่มี per-step orientation intent/constraint ที่พอ
2. raw capture format ยังไม่สอดคล้องกันระหว่าง MCAP กับ session.json
3. การบังคับให้ทุก adapter ต้องมี URDF อาจแคบเกินไป
4. migration plan M4 ยังฝัง MG400 kinematics bias ไว้เร็วเกินไป

อีกครั้ง: อย่าถือว่าทั้ง 4 ข้อนี้ถูกเสมอ
ให้พิสูจน์ทีละข้อ

## วิธีทำงานที่ต้องการ

ให้ทำงานตามลำดับนี้:

1. สรุปก่อนว่าแต่ละ finding กำลังตั้งคำถามเรื่องอะไรจริงๆ
2. ไปหา evidence จาก proposal/revision/repo ก่อน
3. ไปหา evidence จากภายนอกเพิ่ม โดยใช้:
   - primary sources
   - official docs
   - ROS2 / MoveIt / ros2_control docs
   - vendor docs
   - papers / surveys / prior art ที่เกี่ยวข้อง
4. สำหรับแต่ละ finding ให้ตอบให้ได้ว่า:
   - จริงไหม
   - จริงแค่บางส่วนไหม
   - มี framing ที่ดีกว่านี้ไหม
   - ถ้าจะแก้ ควรแก้อย่างไร
   - ถ้าไม่ควรแก้ ควรอธิบายอย่างไร
5. ถ้ามี design ที่ดีกว่าทั้ง proposal เดิมและ review finding เดิม ให้เสนอ design นั้น
6. ถ้าจะเปลี่ยน proposal ให้เปลี่ยนบนหลักฐาน ไม่ใช่เพราะ reviewer พูดไว้

## หลักคิดที่ต้องยึด

1. เป้าหมายหลักของโปรเจคนี้คือ multi-robot teaching
2. ระบบต้องยังเป็น robotics / ROS2 architecture ที่ใช้ได้จริง
3. อย่า optimize เฉพาะ MG400 จนทำให้แกนกลางแคบ
4. แต่ก็อย่า abstract กว้างเกินจน implement/test ไม่ได้
5. ทุกข้อเสนอควรมี validation path
6. ถ้าแก้อะไร ต้องคิดเผื่อคนหรือ AI ตัวถัดไปด้วย

## สิ่งที่อยากให้ตรวจเป็นพิเศษ

### A. Orientation semantics

ตรวจให้ลึกว่าใน canonical IR ควรมีอะไรแน่ระหว่าง:
- full target orientation only
- per-step orientation constraint
- orientation priority / tolerance
- task-space intent เช่น exact / yaw_only / tool_axis_align / free

อย่าเดาจากความรู้สึก
ให้ดูจาก prior art ของ:
- teleoperation systems
- manipulation task representations
- motion planning / skill representations
- robots with limited orientation authority เช่น SCARA / 4-axis arms

ถ้าพบว่าควรมี per-step field จริง ให้เสนอ schema ที่ไม่ overengineer
ถ้าพบว่าไม่จำเป็น ให้พิสูจน์ให้ได้ว่าทำไมระดับ adapter/profile เพียงพอ

### B. Raw capture format

ตรวจให้ชัดว่า raw capture ของเฟสนี้ควรเป็นอะไร:
- MCAP
- session.json
- ทั้งคู่แต่มี canonical source of truth เพียงอันเดียว

ให้มองทั้งเรื่อง:
- compatibility กับ ROS2 ecosystem
- practicality ใน repo นี้
- ease of testing
- ease of conversion
- burden of implementation

อย่าเลือกเพราะดูสวยหรือดูทันสมัย
ให้เลือกเพราะเหมาะกับ workflow จริงของโปรเจคนี้

### C. URDF requirement

ตรวจสอบให้ชัดว่า architecture ควรบังคับระดับไหน:
- `urdf_path` required เสมอ
- optional แต่ strongly recommended
- ไม่บังคับ URDF แต่บังคับ `kinematics strategy`

ให้เทียบกับ robot families ต่างกัน:
- ROS2-native arms
- queued TCP robots
- offline-program robots
- vendor-specific stacks

ถ้าพบว่า URDF required แรงเกินไป ให้เสนอ contract ใหม่ที่ precise กว่า
เช่น `kinematics capability` หรือ `retargeting strategy requirement`

### D. Lifter bias / M4-M5 split

ตรวจสอบว่า M4/M5 ใน migration plan ควรแบ่งอย่างไรจริง
โจทย์คือ:
- อยากได้เส้นทางเริ่ม implement ได้เร็ว
- แต่ไม่อยากฝัง MG400 worldview ลง core ตั้งแต่ milestone แรก

ให้คิดว่ามีทางเลือกที่ดีกว่าไหม เช่น:
- ตั้ง seam ตั้งแต่ M4 แต่มี MG400 provider ตัวแรก
- แยก capture/lifter/kinematics provider ตั้งแต่ PR แรก
- ใช้ mock provider / URDF provider กลางตั้งแต่ต้น

## ผลลัพธ์ที่ต้องการ

ผลลัพธ์สุดท้ายควรมี 4 ส่วน:

1. **Finding-by-finding judgment**
   - สำหรับแต่ละ finding ให้สรุปว่า:
     - confirmed
     - partially confirmed
     - rejected
     - reframed

2. **Evidence**
   - อ้างทั้ง evidence จาก repo
   - และ evidence จากภายนอก
   - ชี้ชัดว่าอะไรคือหลักฐานที่ load-bearing จริง

3. **Recommended proposal changes**
   - ถ้าควรแก้ proposal ให้ระบุชัดว่าแก้ section ไหน
   - แก้อย่างไร
   - ตัดอะไร
   - เพิ่มอะไร
   - รวมกลับเข้า proposal หลักหรือ brief อย่างไร

4. **Implementation safety note**
   - ถ้าวันนี้เริ่ม implement เลย อะไรคือจุดที่ยังไม่ควรรีบล็อก
   - อะไรคือจุดที่ควร finalize ก่อนเริ่ม PR แรก

## รูปแบบการตัดสิน

อย่าตอบแค่ว่า "เห็นด้วย" หรือ "ไม่เห็นด้วย"
ให้ใช้หลักนี้:

- ถ้า finding ถูก:
  อธิบายว่าถูกเพราะอะไร และทางแก้ที่เล็กแต่คมที่สุดคืออะไร

- ถ้า finding ถูกแค่บางส่วน:
  อธิบายว่าส่วนไหนถูก ส่วนไหน framing ยังไม่ดีพอ แล้วเสนอ framing ใหม่

- ถ้า finding ไม่ถูก:
  อธิบายว่าทำไม พร้อมหลักฐาน และบอกว่าจะป้องกัน misunderstanding นี้อย่างไรในเอกสาร

## ข้อสำคัญเรื่องเอกสาร

อย่าสร้างเอกสารคู่ขนานเพิ่มไปเรื่อยๆ ถ้าไม่จำเป็น

ถ้าคุณเชื่อว่าข้อเสนอใหม่ถูกต้องกว่า:
- ให้พิจารณารวมกลับเข้า
  `/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_proposal.md`
  เป็นหลัก

ถ้าจะเก็บเอกสาร review แยก:
- ต้องอธิบายชัดว่าไฟล์ไหนคือ source of truth
- และต้องไม่ทำให้ handoff แตกเป็นหลายเส้น

## เป้าหมายของงานรอบนี้

เมื่อจบงานรอบนี้ ควรตอบได้อย่างมั่นใจว่า:

- review findings 4 ข้อนี้อันไหนจริง อันไหนไม่จริง
- ถ้าจริง ควรแก้อย่างไรให้ดีกว่าเดิมจริง
- proposal หลักควรถูกปรับตรงไหนบ้าง
- อะไรควรถูก finalize ก่อน implementation
- อะไรยังควรปล่อยเป็น open question

ถ้ายังตอบไม่ได้โดยมีหลักฐานรองรับ
ให้ถือว่างานรอบนี้ยังไม่จบ
```

## ใช้ตอนไหน

ใช้ prompt นี้เมื่อต้องการให้ AI อีกตัว:

- re-check proposal อย่างเข้มงวด
- ไม่เชื่อ reviewer แบบอัตโนมัติ
- research เพิ่มก่อนตัดสิน
- ลดโอกาสหลงทางก่อนเริ่ม implementation

## ไฟล์ที่เกี่ยวข้อง

- [next_phase_architecture_proposal.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_proposal.md)
- [next_phase_architecture_revision.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_revision.md)
- [next_phase_architecture_brief.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_brief.md)
- [architecture_prompt_th.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/architecture_prompt_th.md)
