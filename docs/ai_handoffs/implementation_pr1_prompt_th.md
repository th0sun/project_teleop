# Thai Prompt For PR1 Implementation

ไฟล์นี้คือ prompt ภาษาไทยสำหรับส่งต่อให้ AI ตัวถัดไปเริ่มงาน
implementation รอบแรกของสถาปัตยกรรมใหม่ หลังจาก proposal v1.3
นิ่งพอแล้ว

## Prompt พร้อมใช้

```text
คุณกำลังทำงานอยู่ใน repository นี้:
/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop

branch ที่ต้องทำงานต่อ:
feat/multi-robot-teaching-architecture

ภารกิจรอบนี้ไม่ใช่การออกแบบสถาปัตยกรรมใหม่อีกครั้งจากศูนย์
แต่คือการเริ่ม implementation รอบแรกของ architecture ที่ถูกเลือกไว้แล้ว
โดยต้องยึด proposal ปัจจุบันเป็น source of truth

## Source of truth

ให้อ่านไฟล์เหล่านี้ก่อนตามลำดับ:

1. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/project_objective.md
2. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/current_refactor_status.md
3. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_brief.md
4. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_proposal.md
5. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_revision.md

กฎสำคัญ:
- `next_phase_architecture_proposal.md` คือ source of truth
- `next_phase_architecture_revision.md` คือ audit log / reasoning trail
- ถ้ามี conflict ให้ proposal ชนะ

## เป้าหมายของรอบนี้

ให้เริ่ม implementation ของ “PR1-sized milestone” เท่านั้น
โดยโฟกัสที่การวางแกนกลางให้ถูกก่อน

สิ่งที่ควรทำในรอบนี้:

1. freeze canonical IR schema รุ่นแรกในโค้ด
2. วาง data model ขั้นต่ำสำหรับ calibration / workspace feedback
3. วาง `KinematicsContract`
4. วาง `KinematicsProvider` Protocol + stub providers ขั้นต่ำ
5. เพิ่ม test ที่ยืนยัน contract หลักพวกนี้
6. อย่ากระโดดไปของหนักเกิน milestone เช่น MoveIt integration เต็ม, UR adapter เต็ม, BT layer

## สิ่งที่อยากได้เป็น deliverables รอบนี้

อย่างน้อยให้ได้:

1. package หรือ module แกนใหม่สำหรับ robot-neutral core
2. canonical program dataclasses / schema / IO path ขั้นต้น
3. `move` step schema ที่มี field สำคัญครบ:
   - `kind`
   - `motion`
   - `pose_frame`
   - `pose`
   - `orientation_intent`
   - `orientation_tolerance_rad` (optional)
   - `joint_hint_rad` (optional)
4. `KinematicsContract`
5. `WorkspaceModel` และ `TeachingFeedbackContract` ขั้นต้น
6. `KinematicsProvider` Protocol
7. stub provider อย่างน้อย:
   - null provider
   - urdf provider skeleton
8. regression/unit tests ที่ยืนยัน:
   - schema round-trip
   - missing `orientation_intent` ถูก reject
   - VR-captured program ที่ไม่มี `calibration` ถูก reject หรือถูก mark ว่ายัง replay ไม่ได้ตาม rule ใน proposal
   - `KinematicsContract` validation rules ทำงาน
   - `WorkspaceModel` รองรับ serial-arm และ delta/fake profile fixture ขั้นต่ำ
   - core layer ยังไม่ import จาก adapter layer

## ขอบเขตที่ยังไม่ควรทำรอบนี้

อย่าเพิ่งไปไกลถึง:

- MoveIt Task Constructor integration เต็ม
- UR / Franka adapter เต็มตัว
- BehaviorTree layer
- เลือก pinocchio vs KDL แบบ hard lock ถ้ายังไม่จำเป็น
- ออกแบบ MCAP message schema ให้ใหญ่เกินต้องใช้
- rewrite runtime เดิมครั้งใหญ่

ถ้าต้องทำอะไรที่เกิน PR1 ให้หยุดและจัดเป็น note/open question แทน

## หลักคิดที่ต้องยึด

1. โปรเจคนี้คือ robotics / ROS2 project จริง
2. reusable สำคัญ แต่ต้องไม่ abstract จนจับต้องไม่ได้
3. อย่าให้ core ใหม่โตจาก MG400 worldview โดยไม่รู้ตัว
4. แต่ก็ให้ MG400 เป็น first concrete adapter case อย่างซื่อสัตย์
5. testability สำคัญตั้งแต่ PR แรก
6. docs/handoff ต้องอัปเดตตามของจริงที่ลงโค้ด

## งานลงมือที่แนะนำ

ให้เริ่มจากการสำรวจโครง package ปัจจุบันก่อน แล้วเลือกตำแหน่งที่เหมาะสุดสำหรับ
robot-neutral core ใหม่ โดยต้องสอดคล้องกับ repo นี้จริง ไม่ใช่ตาม proposal แบบทื่อๆ

จากนั้น:

### Step A — package/layout kickoff

- ตรวจโครง package ปัจจุบันของ ROS2 workspace
- เลือกตำแหน่งสำหรับ core package ใหม่
- อย่าเพิ่งย้ายโค้ดก้อนใหญ่จาก MG400 runtime ถ้ายังไม่จำเป็น

### Step B — canonical schema

- ลง dataclasses / schema / serialization path สำหรับ canonical program v0.1
- ให้ `orientation_intent` เป็น required สำหรับทุก `move`
- ให้ `orientation_tolerance_rad` เป็น optional
- ยืนยัน file kinds:
  - raw = `*.session.mcap`
  - canonical = `*.program.json`
- VR-captured canonical program ต้องมี `calibration` block ตาม proposal v1.3
- อย่าให้ Unity/VR เป็น source of truth ด้าน safety; feedback จาก VR เป็น advisory เท่านั้น

### Step C — calibration / workspace feedback

- ลง data model สำหรับ `calibration`
- ลง `WorkspaceModel`
- ลง `TeachingFeedbackContract`
- เพิ่ม fixture อย่างน้อย 1 ตัวที่แทน delta / translation-only robot เพื่อกัน core โตจาก serial-arm worldview

### Step D — kinematics seam

- ลง `KinematicsKind`
- ลง `KinematicsContract`
- ลง `KinematicsProvider` Protocol
- ลง stub providers ให้ seam ใช้งานได้จริงตั้งแต่แรก

### Step E — tests

- schema validation test
- round-trip test
- calibration validation test
- contract validation test
- import-discipline test หรือ equivalent guard

### Step F — docs sync

- อัปเดต proposal ถ้า implementation บีบให้แก้รายละเอียดจริง
- อัปเดต revision/audit log ถ้ามี decision สำคัญ
- อัปเดต handoff note ถ้าจำเป็น

## ข้อกำหนดเรื่องการตัดสินใจ

ถ้ามีประเด็นที่ proposal ยังเปิดอยู่ ให้แยกเป็น 2 แบบ:

1. สิ่งที่ต้อง finalize ตอนนี้จริงๆ
2. สิ่งที่ยังปล่อย open ได้โดยไม่ทำให้ PR1 พัง

ตัวอย่างที่ยังไม่ควร over-lock ถ้าไม่จำเป็น:
- pinocchio vs KDL
- quaternion vs axis-angle
- MCAP message-type รายละเอียด

## รูปแบบผลลัพธ์ที่อยากได้จากคุณ

เมื่อทำเสร็จ ขอให้สรุปเป็น:

1. What was implemented
2. What was intentionally deferred
3. What proposal details were confirmed by implementation
4. What proposal details had to be adjusted
5. What the next PR should do

## Validation ที่อยากให้รัน

อย่างน้อยให้พยายามรัน:

- targeted unit tests
- `py_compile` กับไฟล์ Python ที่เพิ่ม/แก้
- `git diff --check`

ถ้ามี ROS2 tools ใน environment ค่อยพิจารณา build เพิ่ม
แต่ถ้าไม่มี ให้บอกตรงๆ ว่ายังไม่ได้ run integration build

## เป้าหมายของรอบนี้

เมื่อจบ PR1 ควรได้สิ่งนี้:

- core seam แรกเริ่มลงโค้ดแล้ว
- canonical IR ไม่ได้อยู่แค่ในเอกสาร
- kinematics seam มีอยู่จริง
- test ยืนยันสัญญาหลักได้
- ยังไม่ได้ overcommit กับ implementation choices ที่ยังควรเปิดไว้

ถ้างานรอบนี้ยังไม่ทำให้ implementation ของ phase ใหม่ “เริ่มจับต้องได้”
ให้ถือว่ายังไปไม่ถึงเป้าหมาย
```

## ใช้ตอนไหน

ใช้ prompt นี้เมื่อจะส่งงานต่อให้ AI ตัวใหม่เริ่มลงมือ implement
architecture phase ตาม proposal v1.3

## ไฟล์ที่เกี่ยวข้อง

- [next_phase_architecture_proposal.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_proposal.md)
- [next_phase_architecture_revision.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_revision.md)
- [next_phase_architecture_brief.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_brief.md)
- [project_objective.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/project_objective.md)
