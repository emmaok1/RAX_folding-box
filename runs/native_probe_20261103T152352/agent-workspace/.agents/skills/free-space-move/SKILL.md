---
name: free-space-move
description: Dexmate unladen free-space motion with explicit simulator-assisted collisions.
---
Read current teacher state and measured control feedback. Use dexmate_motion_plan with
current episode/step and the correct world TCP frame; preview does not move the robot.
Execute only a fresh accepted plan. Inspect measured goal_reached and stop reason separately
from command completion. Never bypass collision refusal with raw stepping. No deforming held
object, moving jaw, or intentional contact support. Plans currently have a 50-row whole-plan
limit; refuse/replan rather than silently remove limits. Tracking errors require diagnosis.
