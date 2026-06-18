# childlink_v2.cmb mesh_id (mid) map — child Link equipment/hand variants

Derived 2026-06-18 by texture identification (tools/pica_texture dumps) + posed-geometry
classification (per-mesh bind-pose centroid/bbox) + in-game render sweep (REPL `linkmid only <n>`).

The CMB bakes EVERY hand-pose + held-equipment variant on distinct mesh_ids; OoT3D (like N64)
shows a state-dependent subset. N64 resolves the selection per-frame into
`player->leftHandType` / `rightHandType` / `sheathType` (PlayerModelType enum) + `currentShield`.
We translate those live values -> the matching mid set and push via SoH3D_GL_SetMidMask.

Textures: p_tex02 = Hylian shield face (blue crest); p_tex17 = DEKU shield face (orange swirl);
p_tex20 = sword sheath/hilt wood; p_tex22 = gold sword guard/ring; p_tex21 = Deku stick (green bands);
p_tex08 = slingshot/sword blade (blue/grey); p_tex07 = boomerang (round); p_tex04/p_tex18/p_tex19 = misc.

## BODY (always shown)
- 24 = full body skin (torso/legs/arms childlink_00/01)
- 26 = head + face + eyes + mouth (childlink_01/f00/f01, c_eye, c_mouth)
- 25 = far-LOD body (near-empty) -> NEVER show

## LEFT hand (bones 15,16) — Link is left-handed: the SWORD hand
- 0  = open empty hand          (LH_OPEN, idle default)
- 1  = closed empty hand        (LH_CLOSED, also bottle)
- 2  = holding rod forward      (LH_BGS / giant-knife-ish)
- 6  = holding DEKU STICK       (p_tex21)
- 7  = open variant
- 8  = holding BOOMERANG         (p_tex07; LH_BOOMERANG, child)
- 15 = misc item (p_tex25, bone15)
- 16 = SWORD in hand            (LH_SWORD; blue blade)

## RIGHT hand (bones 19,20) — the SHIELD hand
- 3  = open empty hand          (RH_OPEN, idle default)
- 4  = closed empty hand        (RH_CLOSED)
- 5  = DEKU SHIELD on arm       (RH_SHIELD + deku; p_tex17)
- 17 = misc (p_tex18)
- 18 = SLINGSHOT                (p_tex04; RH_BOW_SLINGSHOT, child)
- 19 = misc (p_tex19)
- 20 = misc (p_tex19+p_tex23)

## BACK / sheath (bone 21)
- 9  = Hylian shield + sword on back   (SHEATH_18 + hylian)
- 10 = Hylian shield on back           (SHEATH_19 + hylian)
- 11 = DEKU shield + sword on back     (SHEATH_18 + deku)  <- normal child loadout
- 12 = Deku shield + guard on back
- 13 = DEKU shield on back (no sword)  (SHEATH_19 + deku)
- 14 = sword on back, NO shield        (SHEATH_16)
- 21 = guard/ring only (empty sheath)  (SHEATH_17)

## WAIST (bones 23,24)
- 22 = item (bone23, p_tex27)
- 23 = scabbard, long (bone24, p_tex24) — sword holder on back; currently left OFF

## Policy (SoH3D_LinkComputeMidMask): body(24,26) + LH(leftHandType) + RH(rightHandType,currentShield)
##   + sheath(sheathType,currentShield). N64 keeps these self-consistent (sword-drawn => empty
##   sheath on back + sword in LH + shield on RH arm; stowed => open hands + shield+sword on back).
