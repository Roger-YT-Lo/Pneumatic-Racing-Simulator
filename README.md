# Pneumatic Racing Simulator  
# 氣壓賽車模擬器  

## ⚠ 使用限制與版權聲明

本專案僅供學術展示與個人學習使用，**未經授權禁止用於商業用途、公開展示或轉載修改。**  
請勿將本專案內容或其衍生修改用於其他公開專案，違者將視為侵權行為追究。

A non-typical motion-based racing game that combines Unity physics with real pneumatic feedback, tilting the player’s seat to replicate G-forces.  
Made for **放視大賞 2025** | Created by **Roger YT.Lo**

這是一款非常規的動態賽車遊戲，結合 Unity 物理引擎與實體氣壓回饋，透過傾斜座椅模擬 G 力。  
作品參賽於 **2025 放視大賞**｜作者：**羅元廷 （Roger YT.Lo）**

---

## Project Overview | 專案概述

**EN** – The game links Unity to a custom Arduino-controlled pneumatic motion system.  
Players use a steering wheel and pedals while the game calculates real-time G-force and streams X/Y tilt angles to the seat.

**中文** – 本專案將 Unity 與自製 Arduino 控制的氣壓動感系統串接。  
玩家透過方向盤與踏板操作，遊戲即時計算 G 力並傳送 X／Y 傾斜角度至座椅。

---

## Hardware System Overview | 硬體系統概覽

**EN**

- USB serial 115 200 bps for real-time Unity control (portable to other engines).  
- *(See `block_diagram.png` for architecture).*  
- **Steering** AS5048A magnetic encoder + dual DC motors (manual two-stage force feedback)  
- **Throttle / Brake** Potentiometers via analog inputs  
- **Seat Motion** 4 pneumatic cylinders, each with 2 MOS-driven solenoid valves (push / pull)  
- **Sensors** MPU6050 for seat-tilt feedback  
- **MCU** SPI to AS5048A, I²C to MPU6050; executes seat logic from Unity  
- **Air pressure** ≈ 4 MPa  
- **Motion control** MOSFET on/off timing (non-PWM)

**中文**

- 使用 USB 串列（115 200 bps）與 Unity 通訊，可移植至其他引擎。  
- *結構圖見 `block_diagram.png`*  
- **方向盤** AS5048A 磁編碼器 + 兩顆 DC 馬達（手動兩段式力回饋）  
- **油門／煞車** 可變電阻類比輸入  
- **座椅動作** 4 支氣缸，每支以 2 顆 MOS 管驅動之電磁閥（推／拉）  
- **感測器** MPU6050 即時偵測傾角  
- **MCU** AS5048A 走 SPI，MPU6050 走 I²C；依 Unity 指令執行座椅動作  
- **供氣壓力** 約 4 MPa  
- **動作控制** MOSFET 開關定時（非 PWM）

---

## Development Log (Excerpt) | 開發紀錄（摘錄）

### Steering Control | 方向盤控制

| Tag | Change (EN) | 變更（中文） |
|-----|-------------|-------------|
| `new_v2` | Fixed >180° rotation bug | 修正旋轉超過 180° 錯誤 |
| `new_v2_unity` | Startup center-calibration; 2-level feedback; sends raw angle to Unity | 開機自動中心校正、兩段式回饋、傳送原始角度至 Unity |

### Pedal Input | 踏板輸入

| Tag | Change (EN) | 變更（中文） |
|-----|-------------|-------------|
| `new_v1` | Raw analog test | 類比值原始測試 |
| `new_v2` | 0-1 mapping with smoothing & dead-zone | 0–1 區間映射，含平滑與死區 |

### Input Integration | 輸入整合
- `input_control_combine_v2` – Merge throttle / brake and steering logic  
- `input_control_combine_v2` – 整合油門／煞車與方向盤邏輯  

### Seat Tilt Control | 座椅傾斜控制

| Tag | Change (EN) | 變更（中文） |
|-----|-------------|-------------|
| `unity_v5` | Full reset / pull-down (pin 10 = reset, pin 11 = emergency) | 完整重置／緊急下拉（10 號腳位 = 重置，11 號 = 緊急下拉） |

- Unity sends target X/Y angles.  
  Seat range: ±3.5° (X) / ±4.5° (Y).  
  Angle = (0,0) → seat holds still.

- Unity 傳送目標 X/Y 角度。  
  座椅可動範圍：±3.5°（X）/ ±4.5°（Y）。  
  當角度 = (0,0) 時座椅保持靜止。

---

## Pin Logic Summary | 腳位邏輯摘要

| Pin | Function (EN)                    | 功能（中文）               |
|-----|----------------------------------|----------------------------|
| 10  | Reset seat & set baseline        | 座椅歸零並設定基準         |
| 11  | Pull seat to bottom (emergency)  | 緊急下拉座椅               |

---

## Hardware Notes | 硬體備註

- **EN** – Potentiometers need current-limiting resistors. Solenoids require **24 V external power**. Manual push/pull/stop switches reserved. *Do not* PWM the force-feedback motors (no driver IC).  
- **中文** – 可變電阻須加限流電阻；電磁閥需 **24 V 外部供電**；保留手動推／拉／停止開關；**禁止**對力回饋馬達使用 PWM（無驅動 IC）。

---

## Demo Videos | 示範影片

- [Demo #1 – Tilt Simulation Test](https://youtu.be/FYp6wUs9AmY)  
- [Demo #2 – Pneumatic Control Showcase](https://youtu.be/WBgIm44x_N4)

---

## Contact | 聯絡方式

羅元廷 Roger YT.Lo — 台科大電子工程系 NTUST ECE  
Email: **roger020739@gmail.com**

---

## Affiliation | 製作團隊

Developed by **Poke Studio**  
mail: **poke.keyfix@gmail.com**  
Updated: **2025-04-10**
