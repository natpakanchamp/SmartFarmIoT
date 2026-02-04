# Git Workflow Cheat Sheet: From Clone to Push

เอกสารฉบับนี้สรุปกระบวนการทำงานพื้นฐานของ Git สำหรับการเริ่มต้นทำงานกับ Repository จนถึงการส่งงานขึ้น Server

---

## 1. การเริ่มต้น (Initial Setup)
กรณีที่คุณต้องการคัดลอกโปรเจกต์จาก Remote (เช่น GitHub/GitLab) มายังเครื่องคอมพิวเตอร์ของคุณ

```bash
git clone <URL_ของ_Repository>
cd <ชื่อโฟลเดอร์โปรเจกต์>
```

### 2. การตรวจสอบสถานะ (Checking Status)
ก่อนจะเริ่มหรือหลังแก้ไขงาน ควรตรวจสอบสถานะของไฟล์เสมอ

```bash
git status
```

---

## 3. กระบวนการบันทึกและส่งงาน (The Core Workflow)
กระบวนการหลักประกอบด้วย 3 ขั้นตอนสำคัญ ดังนี้:

### Step 1: Stage changes
เลือกไฟล์ที่ต้องการเตรียมบันทึก การใช้ `.` หมายถึงเลือกทุกไฟล์ที่มีการเปลี่ยนแปลงในโฟลเดอร์นั้น

```bash
git add .
```

### Step 2: Commit changes
บันทึกการเปลี่ยนแปลงลงใน Local Repository พร้อมระบุข้อความอธิบาย (ควรเป็นข้อความที่สื่อความหมาย)

```bash
git commit -m "คำอธิบายการแก้ไขงานของคุณ"
```

### Step 3: Push changes
ส่งการเปลี่ยนแปลงจากเครื่องของคุณขึ้นไปยัง Remote Repository (Server)

```bash
git push origin <ชื่อ_branch>
# ตัวอย่างเช่น: git push origin main
```
