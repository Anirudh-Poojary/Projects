# 2D 3-Axis CNC Printer using Arduino

This project implements a simple CNC-style 2D printer capable of converting digital images into pen-drawn vector outputs. It uses two DVD stepper motors for X and Y-axis movement and a servo motor for Z-axis pen-lift control. The system is built around the Arduino Uno and L293D motor drivers, with G-code-based control.

## 🛠️ Hardware Used
- Arduino Uno
- L293D motor driver
- Two DVD stepper motors (for X and Y axes)
- Servo motor (Z-axis pen lift)
- IR sensor (for home position detection, optional)
- Pen/pencil as drawing tool
- Power supply (9–12V DC)

## 🧠 Software Stack
- **Arduino IDE** — for uploading control code (`2D_printer_code.ino`)
- **Processing 3** — for sending G-code via serial (`GCTRL.pde`)
- **Inkscape** with **Gcodetools** plugin — for converting images into G-code
- **GCODES viewer** — optional tool to preview paths


