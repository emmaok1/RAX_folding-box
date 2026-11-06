---
name: robot-interface
description: Control the actual Dexmate Vega 1 U dual arms and coupled grippers.
---

Read ROBOT.md and live dexmate_status. Actions are synchronized 16-D rows, right
then left, 8 values each: absolute world xyz metres, quaternion wxyz, opening 0..1.
Each row advances 20 ms. Retain measured orientations for small translations.
Observe fresh RGB after motion; IK residual and physical tracking may differ
from commands. No object state or physics edits. Reset/evaluation/finish use
RAX lifecycle tools. The physical-lock success gate remains authoritative.
