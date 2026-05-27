# สคริปต์พรีเซนต์ Thesis_Final_sun.pdf

ใช้กับไฟล์: `Thesis_Final_sun.pdf` จำนวน 15 หน้า  
อ้างอิงภาพรวมจาก: `Thesis_Fanal.pdf` จำนวน 55 หน้า  
โทน: ภาษาพูดแบบพรีเซนต์จริง สุภาพ แต่ไม่ท่องเป็นรายงาน

## แกนเรื่องที่ต้องคุมทั้งชุด

ประโยคหลักที่ควรย้ำตลอด:

> งานนี้ไม่ใช่แค่การสั่งให้หุ่นขยับจาก VR แต่เป็น framework ที่แปลงการเคลื่อนไหวของมนุษย์ให้กลายเป็นเส้นทางที่หุ่นยนต์อุตสาหกรรมสามารถนำไปทำซ้ำได้ โดยระบบถูกแบ่งเป็นหลาย layer ตั้งแต่ VR, Unity, ROS2, ไปจนถึง Dobot MG400 ดังนั้นตอนวัดผล เราจึงไม่ได้วัดแค่ปลายทางของหุ่น แต่แยกดูว่า error เกิดขึ้นใน layer ไหน

ท่าทีเรื่อง KPI/error:

> ตัวเลขบางตัวสูงจริงครับ โดยเฉพาะ peak error และ path distance error แต่สิ่งสำคัญคือเราไม่ได้สรุปว่า “หุ่นไม่แม่น” ตรง ๆ เพราะข้อมูลที่วัดมามีทั้ง error จากตัวระบบจริง และ error จากวิธีการเก็บ log/การจับคู่ข้อมูลที่ยังไม่ละเอียดพอ เช่น target จาก Unity, command จาก ROS, และ feedback จากหุ่นไม่ได้เกิดใน timestamp เดียวกันทั้งหมด รวมถึง Dobot MG400 ไม่ได้ส่ง command ID กลับมาให้จับคู่โดยตรง ทำให้บางช่วงเป็นการเทียบ target เก่ากับ feedback ใหม่ หรือ feedback ขณะหุ่นกำลังเคลื่อนที่ ไม่ใช่ตำแหน่ง settled สุดท้าย

## หน้า KPI ที่ต้องจำไว้ แม้ไม่ได้อยู่ในไฟล์ _Sun

ใน deck ใหญ่หน้า KPI มี 3 ข้อ:

1. Controller to Virtual Robot latency: target <= 500 ms, actual <= 40 ms  
   อันนี้ผ่าน และต้องพูดชัดว่าเป็น latency ของ master device ไป virtual robot ไม่ใช่ full robot execution

2. Average coordinate error: target <= 0.50 mm, actual 0.410 mm  
   อันนี้ผ่านในค่าเฉลี่ย แต่ peak error ยังสูง จึงต้องแยก average กับ peak transient

3. Path distance error: target <= 0.50 mm ต่อ 100 mm, actual 0.860 mm ต่อ 100 mm  
   อันนี้ยังไม่ผ่าน และต้องยอมรับว่าเป็น limitation ของ prototype

คำพูดสำหรับหน้า KPI หรือถ้าโดนถาม:

> KPI ของระบบแบ่งเป็น 2 มุมครับ มุมแรกคือความไวของ interface ซึ่งเราวัดจาก controller ไป virtual robot ตรงนี้ได้ไม่เกิน 40 ms ต่ำกว่าเกณฑ์ 500 ms ชัดเจน แปลว่าผู้ใช้ขยับมือแล้วภาพใน Unity ตอบสนองเร็วพอสำหรับการใช้งาน ส่วนมุมที่สองคือความแม่นยำของเส้นทาง อันนี้ค่าเฉลี่ยบาง layer ผ่าน เช่นค่าเฉลี่ย coordinate error 0.410 mm แต่พอไปดู peak และ path distance จะเห็นว่ายังมี error สูง โดยเฉพาะ path distance ได้ 0.860 mm ต่อ 100 mm ซึ่งเกิน target 0.5 mm จุดนี้เราไม่ปิดบังครับ แต่ตีความว่าเป็น limitation ของ prototype ที่เกิดจาก hardware queue, sampling/filtering, และการวัดผลแบบ multi-layer ที่ timestamp ยังไม่ perfectly synchronized

## ข้อเท็จจริงจาก log ที่ใช้รองรับเวลาอธิบาย error

ใช้เฉพาะเมื่อโดนถามลึก หรือใช้ช่วยเล่าในหน้าผลลัพธ์:

- log ล่าสุด `teleop_session_20260526_031432.csv` มี 1,442,200 rows ในช่วงประมาณ 13,372.8 s หรือ 3.7 ชั่วโมง
- ในไฟล์นี้มี `robot_feedback` 1,441,988 rows แต่มี `unity_sample` แค่ 26 rows, `unity_target` 22 rows, และ `ros_command` 134 rows
- แปลว่า raw log ล่าสุดถูกครอบด้วย feedback จากหุ่นยาวมากหลังช่วง input จาก Unity แทบไม่มีแล้ว ถ้าเอาไฟล์นี้ไปคิด KPI ตรง ๆ จะเกิด error/age ที่ misleading
- ค่า `active_ros_command_age_ms` median ประมาณ 6,590,406 ms หรือประมาณ 109.8 นาที แปลว่า feedback จำนวนมากถูกจับคู่กับ command เก่ามาก ไม่ควรใช้เป็น settled accuracy
- ไฟล์ validation ที่เหมาะกับการอธิบายระบบมากกว่า เช่น `teleop_session_20260525_184839.csv` มี 2,615 rows ในช่วงประมาณ 11.82 s
- rate ของรอบ validation: Unity sample 567 rows หรือประมาณ 47.97 Hz, Unity target 419 rows หรือประมาณ 35.45 Hz, ROS command 65 rows หรือประมาณ 5.50 Hz, MG400 feedback 1,445 rows หรือประมาณ 122.24 Hz
- ในไฟล์ validation นั้น Unity -> ROS excess delay หลัง clock calibration เฉลี่ย 84.092 ms, median 71.837 ms, p95 228.277 ms, max 335.370 ms
- ROS decision delay เฉลี่ย 17.574 ms, median 9.193 ms, p95 65.627 ms, max 121.110 ms
- Tracking error ระหว่าง MG400 ToolVectorTarget -> ToolVectorActual เฉลี่ย 1.629 mm, p95 4.645 mm, max 7.132 mm
- Unity -> ROS mapping จาก 3 session ล่าสุด match ได้ 100%: Unity sample 28,158/28,158 rows และ Unity target 18,547/18,547 rows ภายใน 0.0002 degree
- ROS -> Robot ยังไม่ใช่ direct command-ID trace เพราะ MG400 feedback command id เป็น `0` จึงต้อง infer จาก sequence/pose และควรพูดว่าเป็น medium-confidence mapping ไม่ใช่ hardware ID match

ประโยคสั้นสำหรับอธิบาย log:

> ถ้าใช้ raw log ทั้งไฟล์ ตัวเลข error จะดูแย่เกินจริง เพราะ log เป็น event log ที่แต่ละ row ไม่ได้หมายความว่าทุก layer ถูก sample พร้อมกัน ในรุ่นถัดไปต้องแยก KPI เป็น settled-window accuracy และเพิ่ม command ID/timestamp กลาง เพื่อไม่ให้ feedback หลังการทดลองถูกเอาไปเทียบกับ command เก่า

ประโยคตัวเลขสั้นสำหรับหน้า KPI:

> รอบ validation ที่ clean กว่ามี Unity sample ประมาณ 48 Hz, ROS command ประมาณ 5.5 Hz และ feedback จาก MG400 ประมาณ 122 Hz ดังนั้นฝั่ง feedback ของหุ่นไม่ได้ช้า แต่ bottleneck คือการเลือกว่าจะส่ง command ใหม่เมื่อไร และการ execute ผ่าน command queue ของ Dobot มากกว่า

---

# สคริปต์ทีละหน้า

## หน้า 1: เนื้อหา / เข้าสู่ Development

พูด:

> หน้านี้ผมจะเริ่มเข้าสู่ส่วน Development ครับ ก่อนหน้านี้ใน deck หลักเราได้ปูไปแล้วว่างานนี้มี objective คือทำระบบ Teach and Repeat โดยใช้ VR เป็น master device และให้หุ่นยนต์ทำซ้ำเส้นทางได้ ส่วนที่ผมจะพรีเซนต์ต่อจากนี้จะเน้นว่า framework ข้างในถูกออกแบบยังไง ตั้งแต่ Unity, ROS2, ไปจนถึง Dobot MG400 แล้วค่อยไล่ไปที่ผลการทดสอบ error ของแต่ละ layer

เชื่อม:

> เหตุผลที่ต้องเริ่มจาก architecture ก่อน เพราะถ้าดูแค่กราฟ error ตอนท้ายอย่างเดียว จะไม่รู้ว่า error มาจาก controller, Unity, ROS, หรือ hardware robot ครับ

## หน้า 2: Architecture ภาพรวมการเชื่อมต่อ

พูด:

> ภาพนี้คือ architecture ระดับสูงของระบบครับ ฝั่งซ้ายคือ Meta Quest 3 ที่เป็นอุปกรณ์ VR สำหรับรับการเคลื่อนไหวของผู้ใช้ ข้อมูลจาก VR เข้ามาที่ Unity เพื่อแสดงผล digital twin และคำนวณตำแหน่งหรือ joint target จากนั้น Unity ส่งข้อมูลเข้า ROS2 ผ่าน ROS-TCP Connector/Endpoint แล้ว ROS2 เป็นตัวกลางที่รับคำสั่ง ตรวจสอบ safety และส่งต่อไปยัง Dobot MG400 ผ่าน Ethernet TCP/IP

> จุดสำคัญคือระบบไม่ได้ต่อ VR เข้าหุ่นโดยตรง แต่ผ่าน middleware หลายชั้น เพราะเราต้องการให้ Unity เป็น visual/interaction layer และให้ ROS2 เป็น control/safety layer ก่อนถึง hardware จริง

เชื่อม:

> จากภาพนี้จะเห็นว่า latency และ error สามารถเกิดได้หลายจุด ดังนั้นในผลลัพธ์ท้าย ๆ ผมจะแยกวัดเป็น Unity-to-ROS และ ROS-to-Robot แทนที่จะวัดรวมจุดเดียวครับ

## หน้า 3: Architecture แบบแบ่ง layer

พูด:

> หน้านี้ขยาย architecture ออกเป็น layer ครับ เริ่มจาก VR Interface Layer รับ pose และ interaction จาก Meta Quest 3 ต่อมาคือ Interaction and Visualization Layer ใน Unity ซึ่งมี VR UI, digital twin, FK/IK solver และการ publish command ไป ROS2

> ถัดมาคือ Control and Middleware Layer ใน ROS2 มี teleoperation node, MG400 control engine และ safety/validation module หน้าที่หลักคือรับ command จาก Unity ตรวจ joint limit ตรวจความถูกต้อง แล้วค่อยส่ง motion command ไปหาหุ่นจริง

> ชั้นสุดท้ายคือ Robot Hardware Layer คือ Dobot MG400 controller และ servo motor จริง ๆ ที่รับคำสั่งผ่าน TCP/IP แล้วส่ง robot feedback กลับมา

เชื่อม:

> การแบ่ง layer แบบนี้ทำให้เวลา debug เราไม่ต้องเดาว่า error มาจากไหน แต่สามารถถามเป็นชั้น ๆ ได้ เช่น Unity ส่งถูกไหม, ROS แปลงถูกไหม, และหุ่นตาม command ได้ดีแค่ไหน

## หน้า 4: Flow Chart

พูด:

> Flow chart นี้อธิบาย logic การทำงานของระบบครับ เริ่มจากผู้ใช้ขยับ VR controller หรือ interact ใน Unity จากนั้น Unity คำนวณตำแหน่งเป้าหมายและแปลงเป็น joint/pose command ก่อนส่งเข้า ROS2

> ROS2 จะรับข้อมูลแล้วทำ validation เช่น เช็ค joint limit, เช็คว่าคำสั่ง valid หรือไม่, เช็ค error/status ของหุ่น และจัดการว่าจะส่ง command ต่อไปที่ Dobot หรือหยุด/แจ้งเตือน

> อีกด้านหนึ่ง หุ่นยนต์ส่ง feedback กลับมาที่ ROS2 แล้ว ROS2 ส่งสถานะกลับ Unity เพื่อให้ผู้ใช้เห็นว่า robot อยู่ตรงไหนและระบบยังเชื่อมต่ออยู่หรือไม่

เชื่อม:

> Flow นี้เป็นเหตุผลที่ log ของเรามีหลาย event type เช่น Unity sample, ROS command, robot feedback และ command result เพราะแต่ละ event เกิดคนละจังหวะใน pipeline

## หน้า 5: ROS2 Workspace / Bridge Layer

พูด:

> หน้านี้โฟกัสที่ ROS2 workspace ครับ ROS2 ทำหน้าที่เป็น bridge layer ระหว่าง Unity กับ robot จริง โดย Unity ส่งข้อมูลผ่าน ROS-TCP Endpoint แล้ว ROS node จะรับ topic เข้ามา

> หลังจากรับข้อมูล ROS จะทำ 4 อย่างหลัก ๆ คือ หนึ่ง แปลง input จาก VR/Unity เป็นคำสั่งที่หุ่นเข้าใจ สอง วางแผนหรือจัดรูปแบบการเคลื่อนที่ สาม ตรวจสอบ error และ safety แบบ real-time และสี่ ส่งสถานะหุ่นยนต์กลับไปที่ Unity

> ในแง่การออกแบบ เราเลือกให้ ROS2 เป็นตัวกลางเพราะมันเหมาะกับระบบ robotics มากกว่าให้ Unity คุยกับ hardware โดยตรง ทั้งเรื่อง topic, logging, safety, และการแยก module

เชื่อม:

> พอถึงการวัดผล Unity-to-ROS เรากำลังวัดว่าข้อมูลที่ออกจาก Unity ผ่าน bridge นี้แล้วยังตรงกับ command ใน ROS แค่ไหน

## หน้า 6: Dobot MG400 Hardware Controller / TCP ports

พูด:

> Dobot MG400 รับคำสั่งผ่าน TCP/IP โดยมี port หลัก ๆ 3 ส่วนครับ Port 29999 ใช้กับ dashboard command เช่น EnableRobot, DisableRobot, ClearError ส่วน Port 30003 ใช้ส่ง motion command เช่น MovJ, MovL, JointMovJ และ Port 30004 ใช้รับ real-time feedback เช่น QActual และ RobotMode

> ในระบบของเรา ROS2 node จึงต้องคุยกับหุ่น 2 ทิศทาง คือส่งคำสั่ง motion เข้า controller และอ่าน feedback กลับมาเพื่อตรวจสถานะจริงของหุ่น

> ตรงนี้สำคัญกับการวัดผลมาก เพราะ feedback จาก Port 30004 คือสิ่งที่เราเอามาเทียบกับ command ที่ ROS ส่งออกไป

เชื่อม:

> แต่ข้อจำกัดคือ feedback ของหุ่นไม่ได้บอกเสมอว่า feedback แถวนั้นเป็นผลจาก command ID ไหนโดยตรง อันนี้เป็นหนึ่งในสาเหตุที่การวัด ROS-to-Robot error ต้องตีความอย่างระวัง

## หน้า 7: Hardware Constraints ของ Dobot MG400

พูด:

> หน้านี้คือข้อจำกัดสำคัญที่สุดของ implementation ครับ Dobot MG400 ไม่มีคำสั่งแบบ direct motor control หรือคำสั่ง streaming แบบ ServoJ/ServoP ที่สามารถส่ง target ความถี่สูงแล้วให้หุ่นตามทันทีเหมือน robot controller บางรุ่น

> วิธีที่เราใช้คือส่ง target เข้า command queue ด้วยคำสั่งอย่าง JointMovJ แทน ซึ่งทำให้ระบบควบคุมได้จริง แต่ไม่ใช่ real-time servo control แบบ 100% ผลคือถ้าเราส่งคำสั่งถี่เกินไป หรือ queue จัดการไม่ดี หุ่นอาจวิ่งตามคำสั่งเก่า เกิด latency สะสม หรือมีการกระตุก/หยุดระหว่างเส้นทาง

> จุดนี้เป็นเหตุผลหลักที่ผมไม่ตีความ error ช่วง ROS-to-Robot ว่าเป็นความแม่นยำเชิง repeatability ของตัวหุ่นอย่างเดียว เพราะมันรวม behavior ของ command queue และ timing ของการส่งคำสั่งด้วย

เชื่อม:

> ดังนั้นพอเราไปดูผล KPI ถ้าเห็นว่า full robot response หรือ path error สูง ส่วนหนึ่งไม่ได้แปลว่าหุ่นยนต์ทำไม่ได้ แต่แปลว่า control strategy แบบส่ง target ทีละจุดผ่าน queue ยังไม่เหมาะกับงาน real-time streaming มากพอ

## หน้า 8: Teach and Repeat Core Concept

พูด:

> หน้านี้คือ core concept ของระบบ Teach and Repeat ครับ เราแบ่งเป็น 3 ขั้น คือ Teach, Generate, และ Repeat

> ขั้น Teach ผู้ใช้ขยับ VR controller ระบบ Unity จะดึงข้อมูล 6 DoF ทั้งตำแหน่งและการหมุน แล้วใช้ FK/IK เพื่อสร้าง waypoint หรือเส้นทางที่ต้องการ

> ขั้น Generate ฝั่ง ROS2 จะรับข้อมูลเส้นทางมาตรวจสอบ joint limit และ safety แล้วแปลงให้เป็นชุด motion command ที่หุ่นสามารถทำงานได้

> ขั้น Repeat คืออัปโหลดหรือส่งชุดคำสั่งนั้นให้ Dobot MG400 ทำซ้ำเส้นทางแบบ offline execution จุดนี้ต่างจาก real-time control เพราะเป้าหมายของ Teach and Repeat คือการสอนเส้นทางและให้หุ่นทำซ้ำได้ ไม่ใช่ให้หุ่นตามมือแบบทันทีตลอดเวลา

เชื่อม:

> เพราะฉะนั้นเวลาอธิบายผล ถ้าเป็น real-time เราจะพูดเรื่อง response/queue มากขึ้น แต่ถ้าเป็น Teach and Repeat เราจะพูดเรื่อง path accuracy และการทำซ้ำของเส้นทางมากขึ้น

## หน้า 9: Result - Real-Time

พูด:

> หน้านี้เป็นตัวอย่างผลในโหมด real-time ครับ จุดประสงค์ของหน้านี้คือให้เห็นว่าระบบสามารถรับ input จากผู้ใช้ผ่าน VR/Unity แล้วส่งต่อจนหุ่นจริงขยับได้จริง

> แต่ผมอยากเน้นว่า real-time ใน prototype นี้เป็น real-time ในเชิง pipeline คือผู้ใช้ขยับแล้วระบบตอบสนองต่อเนื่อง แต่ฝั่ง robot hardware ยังถูกจำกัดด้วย command queue ของ Dobot เพราะไม่มี servo streaming command ดังนั้นผลที่เห็นอาจมี lag หรือไม่ smooth เท่าระบบ industrial real-time servo control

เชื่อม:

> หน้านี้จึงใช้ยืนยัน integration ว่า Unity, ROS2 และ Dobot เชื่อมกันครบ ส่วนความแม่นยำเราจะดูต่อใน layer error ที่ตามมาครับ

## หน้า 10: Result - Teach and Repeat

พูด:

> หน้านี้เป็นผลในโหมด Teach and Repeat ครับ โหมดนี้จะต่างจาก real-time ตรงที่เราไม่ได้บังคับให้หุ่นตามมือทุกเฟรม แต่ให้ผู้ใช้สอนเส้นทางก่อน แล้วระบบบันทึก/แปลงเส้นทางเป็นชุดคำสั่งเพื่อให้หุ่นทำซ้ำ

> ข้อดีของแนวทางนี้คือเหมาะกับงานซ้ำ ๆ ที่ไม่ต้องการให้ operator เขียน robot program แบบ manual ทุกจุด ผู้ใช้สามารถขยับใน environment เสมือนจริง แล้วเอาเส้นทางนั้นไป execute กับหุ่นจริง

> อย่างไรก็ตาม ความแม่นยำของ Teach and Repeat จะขึ้นกับหลายอย่าง เช่น calibration ระหว่างโลกเสมือนกับโลกจริง, sampling ของ waypoint, filter ที่ตัดจุดใกล้กัน, และข้อจำกัดการ execute ของ Dobot

เชื่อม:

> ต่อไปผมจะเข้าสู่หน้าวัด error โดยเริ่มจาก Unity-to-ROS ก่อน เพื่อดูว่าข้อมูลที่ Unity ส่งเข้ามาถูกแปลงใน ROS คลาดเคลื่อนเท่าไร

## หน้า 11: Error Unity <-> ROS

พูด:

> หน้านี้เป็นการเทียบระหว่างคำสั่งจาก Unity กับคำสั่งที่ ROS ได้รับและเตรียมส่งต่อครับ จุดนี้ยังไม่เกี่ยวกับหุ่นจริงโดยตรง แต่เป็นการเช็คว่า bridge และการแปลงข้อมูลระหว่าง Unity กับ ROS ทำให้เกิด error มากแค่ไหน

> ในฝั่ง joint angle ค่า max error อยู่ที่ J1 1.176 องศา, J2 0.803 องศา, J3 1.265 องศา และ J4 เป็น 0 องศา ส่วน MAE อยู่ประมาณ 0.09, 0.082, 0.22 และ 0 องศา

> ในฝั่งตำแหน่ง Cartesian ค่า max ของ vector อยู่ที่ 4.613 mm โดยแกนที่สูงสุดคือ Y ประมาณ 4.590 mm ส่วนค่าเฉลี่ยแบบ MAE ต่อแกนอยู่ประมาณ X 0.436 mm, Y 0.414 mm, Z 0.410 mm

> จุดที่ช่วยยืนยันว่า layer นี้ค่อนข้างแข็งแรงคือจากการ map 3 session ล่าสุด Unity sample สามารถ match กับ ROS ได้ 28,158 จาก 28,158 rows และ Unity target match ได้ 18,547 จาก 18,547 rows หรือคิดเป็น 100% ภายใน tolerance ที่ตั้งไว้ ดังนั้นปัญหาหลักไม่ได้อยู่ที่ข้อมูล Unity หายระหว่างทาง แต่เป็น peak จาก timing และ filtering บางช่วง

> ตรงนี้ตีความได้ว่า average ของการส่งข้อมูล Unity-to-ROS ยังอยู่ในระดับต่ำกว่า 0.5 mm ต่อแกน แต่จะมีบาง peak ที่สูงขึ้น ซึ่งมักเกิดในช่วงเปลี่ยนทิศทางเร็วหรือช่วงที่ filter/sampling ทำให้ target กับ command ไม่ตรง timestamp กันพอดี

เชื่อม:

> ถ้าดูเฉพาะค่าเฉลี่ย หน้านี้เป็นส่วนที่ช่วย support KPI coordinate error 0.410 mm แต่ถ้าดู peak เราจะเห็นว่าระบบยังต้องปรับ sampling และการ synchronize เวลาให้ดีขึ้น

## หน้า 12: Unity <-> ROS Peak 4.613 mm

พูด:

> หน้านี้เป็น visualization ของ peak error ใน layer Unity-to-ROS ครับ จุด peak อยู่ที่ 4.613 mm แถว row 453 ซึ่งแสดงให้เห็นว่าถึงค่าเฉลี่ยจะต่ำ แต่มีบางช่วงที่ trajectory จริงกับ target trajectory แยกออกจากกันชัดเจน

> ผมจะตีความ peak นี้เป็น transient error มากกว่าค่า steady-state accuracy เพราะมันเกิดในบาง row ไม่ได้เกิดตลอดเส้นทาง สาเหตุที่เป็นไปได้คือการ update ระหว่าง Unity กับ ROS ไม่ได้เข้ามาเป็นจังหวะเดียวกัน 100%, มีการ filter จุดที่ใกล้กัน, และ input จาก controller มีการเปลี่ยนเร็ว

> สิ่งที่ได้จากหน้านี้คือ Unity-to-ROS ไม่ใช่ bottleneck หลักในค่าเฉลี่ย แต่ยังมี peak ที่ต้องลดลงถ้าจะให้ระบบพร้อมใช้งานจริงมากขึ้น

เชื่อม:

> หลังจากนี้เราขยับจาก middleware layer ไปที่ hardware layer คือ ROS-to-Robot ซึ่งจะได้รับผลจากข้อจำกัดของ Dobot queue เพิ่มเข้ามาด้วย

## หน้า 13: Error ROS <-> Robot

พูด:

> หน้านี้เป็นการเทียบคำสั่งจาก ROS กับ feedback จากหุ่นจริงครับ อันนี้เป็น layer ที่สำคัญมาก เพราะเป็นช่วงที่คำสั่งออกจาก software ไปเจอกับ controller จริงของ Dobot MG400

> ในฝั่ง joint angle ค่า max อยู่ที่ J1 1.675 องศา, J2 1.828 องศา, J3 2.605 องศา และ J4 0.080 องศา ส่วน MAE ค่อนข้างต่ำ คือ J1 0.044, J2 0.046, J3 0.088 และ J4 0.009 องศา

> แต่ในฝั่ง Cartesian ค่า max vector ขึ้นไปถึง 12.307 mm โดยแกน Y สูงสุดประมาณ 12.290 mm ค่า MAE ต่อแกนอยู่ที่ X 0.178 mm, Y 0.247 mm, Z 0.163 mm

> จุดที่ต้องพูดให้ชัดคือ max 12.307 mm ไม่ควรถูกตีความว่า robot repeatability แย่เท่านั้น เพราะ Dobot spec repeatability เป็นการวัดเมื่อสั่งตำแหน่งเดิมและรอให้ settle แต่ข้อมูลของเราเป็นการเทียบ command กับ feedback ระหว่างระบบกำลังเคลื่อนที่ และหุ่นไม่ได้ให้ command ID ที่แม่นพอสำหรับจับคู่ feedback ทุกแถว ใน log จริง feedback command id เป็น 0 ทำให้ ROS-to-Robot ต้องอาศัยการ infer จากลำดับเวลาและ pose แทน hardware ID โดยตรง

เชื่อม:

> ดังนั้นหน้านี้บอกสองเรื่องพร้อมกัน คือค่าเฉลี่ยของ joint/position ไม่ได้สูงมาก แต่ peak error สูงเพราะ timing, queue, และการจับคู่ command-feedback ยังไม่สมบูรณ์

## หน้า 14: ROS <-> Robot Peak 12.307 mm

พูด:

> หน้านี้แสดง peak error ของ ROS-to-Robot ที่ 12.307 mm ครับ จุดนี้เป็นจุดที่ต้องอธิบายอย่างตรงไปตรงมา เพราะมันสูงกว่า KPI 0.5 mm แน่นอน

> เหตุผลหลักมี 3 ส่วนครับ หนึ่ง Dobot MG400 ในชุดคำสั่งที่เราใช้ไม่มี servo streaming command ดังนั้น ROS ส่ง target เข้า queue แล้วหุ่น execute ตามลำดับ ไม่ได้ตาม target ใหม่แบบทันที สอง การวัดของเราเป็น event log ที่ ROS command กับ robot feedback ไม่ได้ถูก sample พร้อมกันทุก row ทำให้บางแถวเป็นการเทียบตำแหน่งเป้าหมายกับตำแหน่งจริงขณะหุ่นยังวิ่งอยู่ สาม หุ่นไม่ได้ส่ง command ID กลับมาให้จับคู่แบบ 1:1 จึงต้องจับคู่จาก sequence/pose ซึ่งมีความคลาดเคลื่อนได้

> ถ้าจะพูดแบบสั้น คือ peak นี้เป็นตัวชี้ว่า prototype ยังมี bottleneck ที่ hardware execution และ measurement synchronization ไม่ใช่แค่ calculation error ของ Unity หรือ ROS อย่างเดียว

เชื่อม:

> จาก peak error นี้ เราจะไปดูผลรวมในเชิง path distance ว่าเมื่อให้เคลื่อนที่เส้นตรง 300 mm แล้วระยะทางจริงคลาดเคลื่อนเท่าไร

## หน้า 15: ความคลาดเคลื่อนของเส้นทางการเคลื่อนที่

พูด:

> หน้านี้เป็นการวัดความแม่นยำของระยะทาง โดยทดสอบให้ผู้ใช้และหุ่นเคลื่อนที่เส้นตรง 300 mm ทั้งหมด 10 รอบครับ

> ผลที่ได้คือระยะทางเฉลี่ยที่หุ่นเคลื่อนที่ได้ 298.125 mm ส่วนระยะที่คลาดเคลื่อนเฉลี่ย 2.579 mm และ standard deviation ของระยะคลาดเคลื่อนประมาณ 1.66 mm

> ถ้าแปลงตาม KPI ที่กำหนดว่า error ต้องไม่เกิน 0.5 mm ต่อความยาวเส้นทาง 100 mm ผลเฉลี่ย 2.579 mm ต่อ 300 mm จะเท่ากับประมาณ 0.860 mm ต่อ 100 mm ซึ่งยังเกิน target

> ตรงนี้ผมจะสรุปแบบตรงไปตรงมาว่า prototype ผ่านในด้านการเชื่อมต่อระบบและ interface response แต่ยังไม่ผ่านด้าน path distance accuracy ตามเกณฑ์ที่ตั้งไว้ สาเหตุหลักคือการ sampling/filtering, queue command ของ Dobot, calibration ระหว่างโลกจริงกับโลกเสมือน และวิธีเก็บ log/วัดผลที่ยังต้องแยก settled position ออกจาก tracking ระหว่างเคลื่อนที่

ปิดช่วง:

> ดังนั้น contribution ของงานนี้คือ framework ที่เชื่อม VR, Unity, ROS2 และ robot จริงให้ทำงานร่วมกันได้ และมีระบบวัดผลแยก layer ชัดเจน ส่วน limitation ที่เห็นจาก KPI จะเป็นทิศทางพัฒนาต่อ คือปรับ trajectory execution, ลด queue latency, sync timestamp ให้ดีขึ้น และเพิ่ม command ID สำหรับวัดผลให้ตรงกว่าเดิม

---

# คำตอบสำรองเวลาถูกถาม

## ถาม: ทำไม error สูง ทั้งที่หุ่น Dobot spec repeatability 0.05 mm?

ตอบ:

> ค่า repeatability ของหุ่นเป็นการวัดเมื่อสั่งตำแหน่งเดิมและรอให้หุ่น settle แล้วครับ แต่การวัดของเราเป็นระบบหลาย layer และหลาย timestamp โดยเฉพาะ ROS-to-Robot เราเทียบ command กับ feedback ระหว่างที่หุ่นกำลังเคลื่อนที่ รวมถึง Dobot ไม่ได้ส่ง command ID กลับมาชัดเจน ทำให้บาง row เป็น tracking error ระหว่างทาง ไม่ใช่ final repeatability ของตัวหุ่นโดยตรง

## ถาม: แล้วตัวเลข KPI เชื่อถือได้ไหม ถ้า log ยังมีปัญหา?

ตอบ:

> เชื่อถือได้ในฐานะ prototype evaluation ครับ แต่ต้องตีความตามขอบเขตของข้อมูล เราแยกแล้วว่าค่าเฉลี่ยบาง layer เช่น Unity-to-ROS อยู่ประมาณ 0.410 mm แต่ raw log แบบ event log ไม่ควรถูกใช้เป็น settled accuracy ตรง ๆ รุ่นต่อไปควรทำ logging ใหม่โดยใช้ timestamp กลาง, command ID แบบ end-to-end, และวัดเฉพาะช่วงที่หุ่นหยุดนิ่งหลัง command เพื่อให้ KPI แม่นขึ้น

## ถาม: ถ้า log ล่าสุดยาวมาก ทำไมไม่ใช้เป็นตัวหลัก?

ตอบ:

> log ล่าสุดใช้เป็นหลักฐานเรื่องระบบ logging ได้ครับ แต่ไม่เหมาะใช้เป็น KPI หลัก เพราะมันมี robot feedback ยาวประมาณ 1.44 ล้านแถว แต่มี Unity sample แค่ 26 แถว หมายความว่าหลัง input จาก Unity หยุดแล้ว feedback ยังถูกบันทึกต่อไปอีกนาน ถ้าเอาไปคำนวณ error ตรง ๆ จะเหมือนเอา feedback ปัจจุบันไปเทียบกับ command เก่ามาก จึงต้องใช้รอบ validation ที่ช่วง input-command-feedback สมดุลกว่าแทน

## ถาม: ทำไม latency KPI ผ่าน แต่ end-to-end latency ใน result สูงกว่า 500 ms?

ตอบ:

> KPI ข้อแรกในสไลด์วัด Controller-to-Virtual-Robot latency ซึ่งเป็น response ของ interface ผู้ใช้ อันนี้ได้ไม่เกิน 40 ms และผ่านเกณฑ์ 500 ms ส่วน end-to-end ไปถึงหุ่นจริงเป็นอีก metric หนึ่งที่สูงกว่า เพราะรวม ROS processing, network, และที่สำคัญคือ command queue ของ Dobot MG400 ซึ่งไม่ใช่ servo streaming controller

## ถาม: ถ้าจะทำให้ผ่าน KPI path accuracy ต้องแก้อะไร?

ตอบ:

> มี 4 เรื่องหลักครับ หนึ่ง ปรับ sampling time และ filter waypoint ไม่ให้ตัดหรือหน่วงเส้นทางมากเกินไป สอง ปรับ strategy การ execute เช่นส่งเป็น trajectory/offline path แทนการยิง target ทีละจุดถี่ ๆ สาม ปรับ speed/acceleration/CP ของ Dobot ให้เหมาะกับ path และสี่ ปรับระบบ logging ให้มี command ID กับ settled-window measurement เพื่อแยก error จริงของหุ่นออกจาก error ที่เกิดจากการจับคู่ข้อมูลผิดเวลา

## ถาม: สรุปงานนี้สำเร็จไหม?

ตอบ:

> สำเร็จในส่วน framework และ integration ครับ ระบบสามารถรับการเคลื่อนไหวจาก VR แสดงผลใน Unity ส่งผ่าน ROS2 และควบคุม Dobot MG400 จริงได้ รวมถึงมีการวัดผลแยก layer แต่ยังไม่สำเร็จเต็มตาม KPI ด้าน path accuracy ซึ่งเป็น limitation ที่พบจากการทดลองจริง และเป็นจุดที่เสนอไว้สำหรับ future improvement
