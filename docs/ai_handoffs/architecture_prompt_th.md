# Thai Prompt For Next AI

ไฟล์นี้คือ prompt ภาษาไทยสำหรับส่งต่อให้ AI ตัวใหม่ทำงานต่อในเฟส
`multi-robot teaching architecture` ของโปรเจคนี้ โดยตั้งใจเขียนให้
copy-paste ไปใช้ได้ทันทีและไม่ต้องเสีย token ไปกับการไล่หา context เองมากเกินไป

## Prompt พร้อมใช้

```text
คุณกำลังทำงานอยู่ใน repository นี้:
/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop

branch ปัจจุบันที่ควรทำงานต่อคือ:
feat/multi-robot-teaching-architecture

ภารกิจของคุณไม่ใช่การไล่ optimize MG400 ให้ real-time ขึ้นอีกเล็กน้อย
แต่คือการออกแบบสถาปัตยกรรมเฟสถัดไปของระบบนี้ให้ตรงกับวัตถุประสงค์จริงของโปรเจค:

"แปลงการสอนท่าทางมือจาก VR/Unity ให้เป็นชุดคำสั่งหรือโปรแกรมกลางที่สามารถนำไปปรับใช้กับหุ่นหลายชนิดได้ โดยให้วิธีแปลงและวิธีรันขึ้นอยู่กับ capability และข้อจำกัดของหุ่นแต่ละตัว"

ให้ถือสิ่งนี้เป็นแกนกลาง:
- เป้าหมายหลักคือ robot teaching
- แนวทางหลักคือ offline-first teaching
- real-time / near-real-time เป็นเพียง execution mode หรือ system extension
- MG400 เป็น first adapter / case study ที่สำคัญ แต่ไม่ใช่นิยามทั้งหมดของระบบ

## สิ่งที่ต้องทำก่อนเริ่มออกแบบ

ให้อ่านไฟล์เหล่านี้ตามลำดับนี้ก่อน และอย่าข้าม:

1. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/project_objective.md
2. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/current_refactor_status.md
3. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_brief.md
4. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/skills/multi_robot_architecture/SKILL.md
5. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/controller_continuation_guide.md
6. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/teleop_command_logic.md
7. /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/reference_manuals/dobot/README.md

จากนั้นค่อยอ่านโค้ดเฉพาะส่วนที่จำเป็น โดยโฟกัสที่ path หลักเหล่านี้:

- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/monitor_gui.py
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/logic
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/ros
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/monitor
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/trajectory
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/tools
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/MG400_Mock
- /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/Dobot_TCP_IP_Python_V4

## ข้อเท็จจริงสำคัญของ repository นี้

1. เฟส refactor ของระบบเดิมถูกปิดไปแล้ว
   - อย่ากลับไปไล่ cleanup โค้ดเดิมแบบกว้างๆ อีกรอบถ้ายังไม่จำเป็น
   - สิ่งที่ควรทำต่อคือ architecture phase ใหม่

2. ระบบปัจจุบันยังมีความเป็น MG400-first อยู่มาก
   - แต่ตอนนี้ repo ถูกจัดให้พร้อมสำหรับการแยกแกนกลางออกมาแล้ว

3. ส่วนที่มีอยู่แล้วและควรใช้เป็นฐานในการออกแบบต่อ:
   - teleop runtime ที่ modular ขึ้น
   - monitor/runtime ที่ modular ขึ้น
   - trajectory recorder / playback ที่ testable ขึ้น
   - documentation backbone ที่ค่อนข้างพร้อม

4. ต้องคิดแบบโปรเจคหุ่นยนต์ / ROS2 จริง
   - ไม่ใช่ generic app framework
   - ต้องเคารพ execution semantics ของหุ่นจริง
   - ต้องมีเส้นทาง validation โดยไม่พึ่งหุ่นจริงทันที

## โจทย์หลักที่คุณต้องตอบ

ให้ตอบโจทย์ระดับสถาปัตยกรรมเหล่านี้อย่างจริงจัง:

1. canonical taught-program หรือ robot-neutral program representation ควรหน้าตาเป็นอย่างไร
2. robot capability profile ขั้นต่ำที่ระบบต้องรู้สำหรับหุ่นแต่ละตัวมีอะไรบ้าง
3. เส้นแบ่งระหว่าง robot-neutral core กับ robot-specific adapter ควรอยู่ตรงไหน
4. execution mode ควรถูก model อย่างไรระหว่าง:
   - offline export/program generation
   - supervised playback
   - queue-aware execution
   - near-real-time streaming
5. จะ validate ระบบนี้อย่างไรโดยเริ่มจาก mock / simulator / contract test ก่อนมีหุ่นจริง
6. จะ migrate โค้ด MG400-first ที่มีอยู่ตอนนี้เข้า architecture ใหม่อย่างไรโดยไม่รื้อทุกอย่างทิ้ง

## สิ่งที่ห้ามหลงทาง

ห้ามใช้เวลาเฟสนี้ไปกับเรื่องเหล่านี้เป็นหลัก:

- ไล่ลด latency ของ MG400 อย่างเดียว
- ออกแบบ abstraction ที่ฟังดู universal แต่ implement ไม่ได้จริง
- เอา command model ของ MG400 มาเป็น canonical model ของทั้งระบบ
- แยก class/file เพิ่มแต่ไม่ได้ขยับ architecture
- เสนอ framework ที่ต้องรอหุ่นจริงอย่างเดียวถึงจะตรวจสอบได้

ถ้าการตัดสินใจใดช่วยแค่ "ให้ MG400 ตามมือเร็วขึ้นนิดหน่อย"
แต่ไม่ได้ช่วยให้ระบบสอนหุ่นหลายแบบได้ง่ายขึ้น
ให้ถือว่ามีโอกาสสูงที่สิ่งนั้นไม่ใช่แกนกลางของระบบ

## วิธีทำงานที่คาดหวัง

ให้ทำงานเป็นลำดับดังนี้:

1. สรุปความเข้าใจ repository และ objective ใหม่ให้ชัด
2. research กว้างพอ โดยใช้แหล่งข้อมูลที่น่าเชื่อถือและใหม่พอ
3. จัดกลุ่ม design space ออกมาอย่างน้อย 2-3 architecture options
4. เปรียบเทียบข้อดี ข้อเสีย จุดอ่อน และความเป็นไปได้ในการ implement
5. เลือก architecture ที่เหมาะที่สุดกับ repo นี้
6. ออกแบบ artifact หลักของระบบอย่างชัดเจน
7. เสนอ migration plan ที่ค่อยๆ เปลี่ยนจากของเดิมไปสู่ของใหม่
8. เสนอ validation ladder ที่ใช้ mock/simulator/contracts ได้จริง
9. ถ้าจะเริ่มลงมือแก้โค้ด ให้เริ่มจากโครงที่ reusable และไม่ผูกกับ MG400 มากเกินไป

## หัวข้อที่ต้อง research ก่อนเลือก architecture

ให้ค้นคว้าและสังเคราะห์หัวข้อต่อไปนี้:

1. Learning from Demonstration / Programming by Demonstration
2. robot task representation / skill representation
3. capability modeling / hardware abstraction
4. retargeting ระหว่างหุ่นที่มี morphology หรือจำนวนแกนต่างกัน
5. execution semantics ของหุ่นหลายแบบ
   - queued command robots
   - streaming/servo robots
   - offline-program robots
   - hybrid supervised playback
6. เครื่องมือและ ecosystem ที่เกี่ยวกับ ROS2
7. วิธีสร้าง validation ladder โดยใช้:
   - mock TCP servers
   - simulator
   - open-source robot stacks
   - contract tests

ให้พยายามใช้ primary sources หรือ official documentation เป็นหลักเมื่อทำได้
และอย่าหยุดแค่รวบรวมข้อมูล ให้สังเคราะห์ออกมาเป็นกรอบออกแบบที่ใช้สร้างระบบได้จริง

## รูปแบบผลลัพธ์ที่ต้องการ

อย่างน้อยคุณควรส่งมอบสิ่งเหล่านี้:

1. problem framing ของเฟสนี้
2. architecture options อย่างน้อย 2-3 แบบ
3. chosen architecture พร้อมเหตุผล
4. canonical program schema proposal
5. robot capability profile schema proposal
6. adapter interface / contract proposal
7. execution-mode model
8. validation strategy ที่ใช้ได้จริงโดยไม่รอหุ่นจริง
9. migration plan จากโค้ดปัจจุบัน
10. weakness audit หรือ risk register พร้อม mitigation

## แนวทางการแก้โค้ด ถ้าจะลงมือทำต่อจากงานออกแบบ

ถ้าคุณไปต่อถึงขั้น implementation ให้ยึดหลักนี้:

- เคารพ objective ของโปรเจคก่อน
- คิดแบบ reusable แต่ต้องยังเป็น robotics/ROS2 architecture ที่จับต้องได้
- แยก robot-neutral core ออกจาก MG400-specific behavior ให้ชัด
- ให้ MG400 อยู่ในฐานะ adapter ตัวแรก ไม่ใช่ core definition
- ทุกก้อนที่แก้ควรมี test หรือ validation path ที่เหมาะกับความเสี่ยง
- อัปเดตไฟล์ .md handoff/architecture/status ทุกครั้งเมื่อมีการตัดสินใจสำคัญ
- ถ้าสร้าง abstraction ใหม่ ต้องตอบให้ได้ว่าจะใช้กับหุ่นตัวถัดไปอย่างไร

## ข้อกำหนดเรื่องการทดสอบ

ห้ามสรุปว่าสถาปัตยกรรมดีเพียงเพราะโค้ดดูสวยหรือ abstraction ดูกว้าง
ต้องมีแผนทดสอบจริง เช่น:

- contract tests สำหรับ canonical program -> adapter translation
- mock-backed tests สำหรับ queued execution / streaming execution
- simulator-backed tests สำหรับ motion semantics ขั้นพื้นฐาน
- regression tests ที่ไม่ต้องใช้หุ่นจริงทุกครั้ง

ถ้า architecture ใดไม่มี validation path ที่เป็นรูปธรรม ให้ถือว่ายังไม่สมบูรณ์

## สิ่งที่ต้องเคารพจากผู้ใช้คนนี้

ผู้ใช้คนนี้ให้ความสำคัญกับเรื่องต่อไปนี้มาก:

- reusable architecture
- ความเป็น robotics / ROS2 project จริง
- ไม่ทำมั่ว ไม่แตกไฟล์เพื่อความสวยอย่างเดียว
- ต้องอัปเดตเอกสาร handoff และ .md files เรื่อยๆ
- ต้องคิดเผื่อ AI ตัวต่อไปหรือคนอื่นที่จะมาทำต่อ
- ต้องบอก tradeoff ให้ตรง ไม่โลกสวย
- ต้องช่วยกันทำให้โปรเจคนี้ไปถึง "สอนหุ่นหลายแบบได้" มากขึ้นจริง

## เป้าหมายของงานรอบนี้

เมื่อจบงานรอบนี้ คนอ่านควรรู้ชัดว่า:

- แกนกลางของระบบคืออะไร
- สิ่งใดเป็น robot-neutral
- สิ่งใดเป็น robot-specific adapter
- หุ่นตัวใหม่จะถูกเพิ่มเข้าระบบนี้อย่างไร
- ทำไม architecture ที่เลือกจึงเหมาะกับทั้งงานวิจัยและการ implement จริง

ถ้าคุณยังตอบไม่ได้ว่าการเพิ่มหุ่นตัวใหม่จะทำอย่างไร
ให้ถือว่างานสถาปัตยกรรมรอบนี้ยังไม่จบ
```

## เหมาะใช้ตอนไหน

ใช้ prompt นี้เมื่อจะส่ง repo นี้ต่อให้:

- AI ตัวใหม่ที่ต้องไปโฟกัสงานสถาปัตยกรรม
- คนในทีมที่ต้องรับช่วงต่อเฟส multi-robot teaching
- AI ที่ต้องทำ research + design + initial implementation planning

## ไฟล์ที่เกี่ยวข้อง

- [next_phase_architecture_brief.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/ai_handoffs/next_phase_architecture_brief.md)
- [SKILL.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/skills/multi_robot_architecture/SKILL.md)
- [project_objective.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/project_objective.md)
- [current_refactor_status.md](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/docs/current_refactor_status.md)
