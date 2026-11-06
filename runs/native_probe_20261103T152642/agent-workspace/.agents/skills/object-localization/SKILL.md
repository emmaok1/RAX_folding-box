---
name: object-localization
description: Privileged teacher localization from current simulator state, not RGB guesses.
---
Use dexmate_teacher_state and runtime/world/manifest.json. Runtime panel pose, midplane
outline and normal are world coordinates; body origin, COM, visible surface, grasp point
and virtual TCP are distinct. Check current episode/step. Static dimensions and memories
are not live poses. Teachers have privileged simulator information; student RGB contracts
are unchanged. Use images for inspection but never invent world coordinates from pixels.
