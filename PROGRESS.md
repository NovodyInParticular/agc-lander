# AGC Lander — progress & plan

Real Luminary099 (Apollo 11 LM flight software) flies a simulated LM to the
lunar surface. Single Windows .exe (mingw-w64), raylib, Virtual AGC core in-process.

Read this file first when resuming work.

## Phase 1 — DSKY with real AGC output (DONE 2026-11-02)
- yaYUL builds Luminary099, verified word-for-word vs AP11ROPE.binsource.
- src/agc_host.c replaces yaAGC SocketAPI.c. MUST latch channel 07 (superbank).
- Self-test: `make test`, `make test-wine`.

## Phase 2 — peripherals & landing (DONE 2026-11-02)
Luminary099 lands the LM: `make test` runs tools/test_landing.sh (P63->P64->P65,
touchdown ~2.4 m/s, ~575 m from target, no alarms). Lab tool: build/linux/agc-lab.

Hard-won facts (each was a real bug):
- Host ChannelOutput MUST latch ch07 superbank (else banks 40-43 unreachable).
- DPS trim: ch12 bit9 = more +Q (pitch, +Y torque), bit11 = more +R (+Z).
  Engine below CG; thrust dir = (1, -gimbal_roll, gimbal_pitch).
- LR antenna: ch12 bit13 is a DRIVE command; software clears it on arrival
  (ch33 bit7 low); antenna must stay in pos 2.
- LR range: high scale 5.395 ft/count, low scale 1.079 (HISCALIM 460 = 2481.7 ft);
  ch33 bit9 = 1 high scale. Switch ~2500 ft.
- LR velocity: counts = 12288 + v_ft_s / scale, scales X -0.644, Y 1.212, Z 0.8668;
  beams = antenna axes, NB = Rx(alpha) Ry(beta) ANT (alpha 6, beta 24 pos1 / 0 pos2).
- Crew procedure matters: R51 V50N25 -> ENTER (skip fine align); V50N18 -> PRO
  (maneuver) then ENTER when done; DES ENG ARM; V99N62 -> PRO at TIG-5;
  V57E after LR lock (permit LR updates!); ENGINE STOP at contact light.
- Scenario: NASSP "Apollo 11 - 10 - PDI T-15m" LGC erasable (GPLv2), extracted by
  tools/extract_nassp.py into data/a11_pdi_lgc.c; GOJAM restart resumes P63.
  Truth state = LGC's own LM state vector (RCV+DELTA at TETLEM, scales 27/18/5/-1),
  attitude from CDUs + REFSMMAT. Site 0.6914N 23.7169E R=1735418 m.

### Original milestone notes
Milestones, each with a headless test in src/selftest*.c:

- [ ] 2a Symbol table from yaYUL listing -> build/agc_symbols.h; erasable poke
      helpers (SP/DP with binary scaling); padload writer.
- [ ] 2b IMU: stable member orientation (quaternion), gimbal angles
      (CDUX=OGA about X, CDUY=IGA about Y, CDUZ=MGA about Z; NB = Rx(o) Rz(m) Ry(i) SM),
      CDU pulses via PCDU/MCDU, gyro torque from ch 0177 (0.618"/pulse,
      bit15 sign, bits14-13 select 01=X 10=Y 11=Z, bit12 enable, bits11-1 count),
      coarse align ch 0174-0176 (2^-13 rev/pulse, only when ch012 bit4).
      Otherwise 0174-0176 drive FDAI error needles.
- [ ] 2c PIPA: SM-frame delta-V, 1 pulse = 1 cm/s (verify), PINC/MINC on 037-041.
- [ ] 2d RCS: ch05 bits1-8 = Q4U Q4D Q3U Q3D Q2U Q2D Q1U Q1D;
      ch06 bits1-8 = Q3A Q4F Q1F Q2A Q2L Q3R Q4R Q1L. 445 N each.
      Quads: Q1(+Z,-Y) Q2(-Z,-Y) Q3(-Z,+Y) Q4(+Z,+Y); U=thrust -X, D=+X,
      F=thrust -Z, A=+Z, L=+Y, R=-Y. LM axes: X up (yaw), Y right (pitch), Z fwd (roll).
- [ ] 2e DPS: ch011 bit13 on / bit14 off; THRUST counter 055 DINC -> POUT/MOUT
      pulses (~2.7 lbf each); ch014 bit4 thrust drive enable. Gimbal trim ch012.
- [ ] 2f Landing radar: RequestRadarData -> RNRAD (046), ch013 select bits,
      ch033 data-good bits (active low).
- [ ] 2g Pre-PDI state: padload (LUM69R2/PADLOADS.agc as reference),
      state vector, REFSMMAT, RLS, TLAND consistent with our world; ideal IMU
      (zero compensation terms). Then V37E63E -> P63 -> P64 -> P66 -> touchdown.
- [ ] 2h Crew inputs: ACA/RHC, ROD switch (P66), mode switches, abort.

## Phase 3 — visuals + cockpit (2026-11-02)
- Terrain, LM model, plume/dust, cameras, HUD done earlier.
- 3D LM cabin (src/cockpit.c geometry/render/picking, src/cockpit_panels.c artwork+logic):
  panels 1-6, 8, 11, 12, 14, 16 drawn live into a 4096 atlas (+ emission atlas), 3D levers,
  clickable switches/DSKY/DEDA, cabin sun shadow map, window fill light, floods, LPD etched on
  the CDR window (LOOKANGL = asin(unit(R-LAND).X), design eye 2.82,-0.66,0.20).
  Exterior lmmodel windows reshaped to match (t0 .03, t1 .86, y .48-1.00); docking window y -.80..-.52.
  Cockpit depth trick: gl_FragDepth = dist/6*0.03 (< any world log-depth beyond 1.15 m nearCut).
- FAR_PLANE now 2e8 (lunar orbit view).
- Lunar orbit camera (src/sky_extras.c): procedural globe, maria table, ray craters.
- Easter eggs (src/sky_extras.c): rare visitor flyby (first chance 6-20 min of flying,
  then every 15-35 min; --ufo N forces it); far-side 1945-style base at lat -84.6 lon 168.2
  in a permanently shadowed crater (orbit view lets you descend to 250 m only near it;
  --base ALT jumps there). No insignia by design.
- Fullscreen by default (F11), --windowed. F10 / --dual: second instance
  (--spectator) shows the outside view, fed by UDP 127.0.0.1:47011 (src/net.c, StatePkt in main.c).
  A window rule on the title "AGC Lander - Outside View" can pin it to a monitor.
- README.md = controls.

## Co-op + controllers + detail (2026-11-02)
- Two players (src/coop.c): host = CDR runs AGC; client = LMP (--join ADDR / menu). UDP 47012.
  Client actions are intercepted in agc_host_key/pro/set_rhc and lm_set_* (coop_role == CLIENT ->
  coop_forward), resent until acked (StatePkt.ack). Tested Linux<->Linux and Wine .exe -> Linux.
- Controllers (src/controls.c): keyboard + gamepad + raw GLFW flight stick (Extreme 3D Pro layout),
  Sony pads auto-mapped for any GUID (Bluetooth DualSense), raylib "available" != mapped gamepad.
- Bigger windows (s 0.03..1.10, y 0.46..1.00), eye 2.84/±0.66/0.30, rocks.c boulder field with
  shadows, lmmodel build_details() (seams, MESA, SEQ bay, LR antenna, flag, plaque, lights, rails).
- make install (~/.local + launcher), make dist (linux tar.gz with install.sh, windows zip).
- --demo: hands-off landing from high gate with camera cuts.

## Hints, crashes, full-screen view (2026-11-02)
- src/hints.c: plain-English hint box (H / menu row 7, saved in ~/.config/agc-lander.conf or %APPDATA%).
  Procedure hints come from crew.hint (synced in StatePkt); flying hints from LM state.
  Measured in agc-lab (`vb` command): W = +pitch = LM leans BACK (slows forward drift), D = leans right.
  ROD Up = slower descent.
- Crash: lm.c grades first footpad contact (LAND_* limits in lm.h; good <= 2.7 m/s because the
  engine cuts instantly and the LM free-falls 1.7 m from the contact light), topple > 40 deg.
  Crash latches engine stop, PGNS off, uneven strut collapse (lm.collapse[]). Effects in lmmodel.c
  (debris shards fx kind 4, fireball if fuel left), dust burst + camera shake in main.c.
  Results panel (Enter retry / Space hide). `--crash` cuts the engine at flight start (screenshots).
- DSKY floats bottom-right; 3D view always full width; DSKY default only in the full mission.
- DPS bell rebuilt: curved profile, blanket upper half, lip ring, inner wall to a closed throat.

## Next
- GitHub: public repo + v0.1.0 release (2026-11-02). GPLv2 (Virtual AGC, NASSP data).

## Phase 3 original notes
- 3D terrain around Tranquility Base, low sun (~10.8 deg), shadows, LM model,
  plume, RCS puffs, dust, Earth, cameras (window / chase / landing-site),
  FDAI, cross pointer, tape meters, DSKY, sounds.

## Reference scalings
- CDU: 2^-15 rev/count. Gyro: 2^-21 rev/pulse. Coarse: 2^-13 rev/pulse.
- MCT = 11.71875 us.
