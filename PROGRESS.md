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

## Graphics pass, terrain LOD, cabin sound (2026-11-15, branch quest-vr)
- Terrain never pops now. Three parts, all in terrain.c + the terrain shader in render.c:
  1. Geomorph: every chunk vertex carries its parent chunk's surface (position delta in
     `vertexTangent.xyz`, shadow in `.w`, normal xz in texcoord, albedo/fresh in texcoord2). The
     vertex shader blends toward it over the far 72-95 % of the level's range (`morph` uniform),
     and over FADE_TIME (0.6 s) when a chunk first appears (Node.fade). A parent hands over to its
     children only once its own fade is done. Camera jumps (> 400 m in a frame) set `settling`:
     chunks appear at once until the build queue drains. AGC_NOMORPH=1 = old hard switching.
  2. Per-pixel craters: the shader draws crater octaves 6000..6 m (terrain.c octaves[1..6], same
     hash, cells, radius, profile; OCT_* tables in render.c MUST match) for every octave the mesh
     has not taken over, with an exact parabolic-bowl sun shadow. Octave.gk decides the handover:
     geometry includes an octave when cell * gk >= 1.5 * spacing (gk 0.15 = about 10 vertices per
     cell). The shader weight is 1 - that test, blended by the morph factor, so mesh and shader
     trade places invisibly. Shadow drawing continues until 12-48 spacings per cell (vertex
     shadows are too coarse before that). Plane position comes from gl_VertexID (33 x 33 grid +
     skirts) and per-chunk `cellBase`/`cellFrac` uniforms. AGC_DBG=6 turns the crater pass off.
  3. Chunk-border normals use central differences (one sample outside the chunk): no lighting seams.
- Soil detail is soft-clipped (softplus) instead of clamped: no black speckle at the low Sun.
- LM: vertex colours are linear albedo (black 3 %, anodised skin = material 10, half metal,
  rough 0.6); bump gain per material, specular anti-aliasing from normal derivatives, sunlight on
  the vehicle held at 6.5 (regolith and rocks 9) so bright skins keep detail. Aft equipment bay:
  stand-off shield panels and gas tanks instead of the white fins.
- Post: tonemap now writes an LDR target with luma in alpha (blending OFF for that pass), then
  FXAA 3.11 + film grain + dither to the output. Sun: tight corona, faint six-point star, bloom
  input clamped at 260 so the disc does not fog the sky.
- Sound (src/sound.c, all synthesised, one raylib AudioStream callback): fans + 400 Hz inverter
  whine, DPS rumble following thrust, RCS knocks + hiss, switch and DSKY clicks, pulsed 3 kHz
  master alarm (stops after 12 s or on MASTER ALARM push), touchdown thud, crash. Heard only in the
  cockpit and DAC window views; M mutes. Not started in --shot/--audit/--spectator runs.
- AGC_SEQ=N with --shot NAME saves N consecutive frames (NAME-000.bmp ...), for motion checks.
- Still open in Phase 2: panel lettering is small/faint from the seat (LEG 1.28, EL glow 0.6);
  needs a size + overlap review per panel, then the Quest legibility check.

## 3D controls, hints on controls, moonwalk (2026-11-15, branch quest-vr, uncommitted)
- Sound: src/sound.c opens its own miniaudio device (the copy inside raylib) at the device's native
  rate, 1024-frame periods. The "audio cutting out" report was NOT the game: an `adb` server
  polls USB once a second and stalls one core ~21 ms each time; start it with `ADB_LIBUSB=1`
  (hotplug, no polling) or `adb kill-server` when done.
- Every control is modelled (cockpit.c build_controls: toggles with nut + bat handle + wicket
  guards, bar knobs, thumbwheels, push-button and DSKY/DEDA key caps, breaker plungers). The panel
  code registers them (Lever kinds in cockpit_int.h); legends ride on the caps via pc_lift().
  The mesh is rebuilt only when a control moves.
- Hints: hints_target() names the one control to use next; cockpit_panels.c rings it (green,
  pulsing, with an instruction), dsky.c rings the DSKY key (also on the 2D DSKY), and
  cockpit_overlay() points to it when it is out of view or hidden behind another panel.
- Panel 3's top rows (ENG ARM, PGNS mode) cannot be seen from the standing eye point: End now
  steps back and down ("duck") so they can; Home stands up again.
- Newly working switches: mission timer (start/stop/reset + slew wheels), event timer count
  up/down + slew + panel-3 reset, FLOOD ALL/POST, SIDE PNLS, ANUN/NUM BRT/DIM, DSKY LTG, the
  TEMP/PRESS, TEMP, HEATER, POWER and SIG STR monitor selectors, SUIT FAN / CABIN FAN (heard),
  and all 480 circuit breakers pull and push (wired: FLOOD, INTGL, SIDE PNLS, MISSION TIMER,
  EVENT TIMER, PRPLNT QTY, SUIT FAN 1/2, CABIN FAN CONT). Not wired: ABORT / ABORT STAGE,
  ENGINE START, RCS isolation, radar power, batteries, explosive devices.
- Cabin look from the user's reference photos (~/Downloads, LM interior): blue-grey panels and
  structure, cream overhead, yellow handholds, caution stripes round ABORT and START/STOP, AOT
  with its orange guard, metal forward hatch.
- Moonwalk (src/eva.c): after a good landing with the engine off, X opens the forward hatch
  (door swings in, the wall has a real opening), the crewman kneels, goes out and down the
  ladder; on the surface W A S D walk, Shift lope, Space jump (1.62 m/s2), F plants the flag
  (lmmodel_draw_flag), X at the foot of the ladder climbs back in.
- Debug switches: AGC_HINT=arm|auto|stop|alarm|rodup|roddown|<key letter>, AGC_CBOPEN=n,
  AGC_DUCK=1, AGC_EVA=hatch|crawl|ladder|walk|flag|auto (auto = parked LM, hatch opens itself).

### Plan agreed with the user 2026-11-15 (in this order)
1. DONE first pass: cabin realism from the photos; hatch, moonwalk, jump, flag.
2. DONE 2026-11-16: ascent and docking (see the section below).
3. Two builds: PC "high" (i9-9900K 8c/16t, RTX 2070 Super, 32 GB: more terrain detail,
   shadows, 4K) and Quest 3 (72 fps, cut-down shaders). One code base, a quality tier switch.
   DONE in part 2026-11-16 (render_quality() tiers, see the Quest section below); the PC-high
   tier's extra detail and on-headset tuning are still open.
4. DONE 2026-11-16: the Quest APK (org.agclander.quest) builds and runs on the Quest 3S.
- Moonwalk gaps closed 2026-11-16 evening (see "The gaps closed").

## Lift-off, ascent, rendezvous shortcut, docking (2026-11-16, branch quest-vr)
Plan step 2 is done. Everything is flown by the real Luminary099 except the skipped rendezvous.
- Scenario: NASSP "Apollo 11 - 11 - Lunar Liftoff T-1m" LGC state, extracted with
  `tools/extract_nassp.py SCN data/a11_liftoff_lgc.c a11_liftoff "Lunar Liftoff T-1m"` (the tool
  now takes a prefix). scenario_liftoff(keep_pose): GOJAM restart resumes P12's countdown (V06 N74).
  keep_pose=1 = the player's own landing (the L key): same spot and attitude, moon-fixed. Otherwise
  the LM is put at our landing target (NASSP's own site is on a crater wall in our terrain),
  lm_place_on_ground() (lowest pad touching) then lm_settle(4 s): physics only, the Moon turning
  (freezing the clock made the LM skid at 4.6 m/s and tumble). CDUs and CDUXD/YD/ZD are written
  from the real attitude so the DAP does not fire against the ground.
- CSM: from the LGC's own CSM state vector (RCVCSM+DELTACSM, VCVCSM+NUVCSM, TETCSM), propagated;
  src/csm.c (same integrator as the LM), model src/csmmodel.c (CSM axes: origin at the docking
  ring, +X out along the probe; target at y +0.66 z +0.10 matches the LM CDR window offset).
- APS + staging (lm.c): ENG ARM is 3-state (ARM_DES/ARM_ASC), ch11 ENGINE ON lights the armed
  engine, 15570 N, Isp 311. ABORT STAGE (B key / panel 1 button) = lm_abort_stage(): ch30 bit 4
  low, stages part. On the ground the ascent stage rests on the deck (4 contact points) until
  thrust lifts it (lm.liftoff); in flight they part at once and the descent stage falls and stays
  where it lands. After staging the body origin stays the model origin; the CG is lm.cg
  (1.35, 0, 0.04) and rigid_body() integrates the CG (identical to before when cg = 0: the landing
  test result did not change).
- P12 result: 16.7 x 91 km above the site radius (Apollo 11 17.4 x 86.5), 423 s burn, wedge to
  the CSM plane 0.04 deg, no alarms. ABORT STAGE during the descent: Luminary takes P71 and also
  reaches orbit (tested from 13.6 km: 16 x 223 km), then the same rendezvous/docking.
- src/ascent.c: phases (MP_COUNTDOWN .. MP_DOCKED), rendezvous shortcut (Eagle 40 m from Columbia,
  OUT OF Columbia's orbit plane with the same velocity: the line between them then stays fixed in
  inertial space; along-track placement drifted 0.03 m/s against an inertially held LM), the crew's
  docking DAP set-up keyed by watching the DSKY (V37E 00E, wait for P00, V48E, PRO, PRO with
  DBSELECT/ACC4OR2X/RHCSCALE cleared: 0.3 deg deadband, 2-jet X, fine stick), probe-and-drogue
  capture (33 cm, 10 deg, < 0.6 m/s), bounce, damage (> 0.45 m/s body contact), 8 s retract,
  hard dock. src/ascent_ui.c: docking readout and results panels.
- Keys in orbit: T/G along X, arrows sideways (docking-window screen axes: right = LM -Y), Q/E
  roll about the docking axis (Q lowers a positive roll error), W/S A/D tilt. Camera
  CAM_DOCK = overhead window + COAS reticle. Headset: left stick/grip/trigger translate.
- Tests (all in `make test`): test_ascent.sh, test_docking.sh (lab `dockpilot`: THC pulses + RHC
  only, through the real DAP), test_mission.sh (PDI -> landing -> liftoffkeep -> orbit -> dock).
- Audit: 55 views now (asc_* ascent stage above the descent stage, csm_* round Columbia); all 7
  texture maps load, 0 missing (also checked with the installed ~/.local/bin binary).
- Debug switches: AGC_AUTOCREW=1 (--start 4 with the auto crew), AGC_DOCK=metres (+AGC_DOCKCAM=n)
  starts lined up near Columbia, AGC_FAKE_RHC=p,y,r[,tx,ty,tz], AGC_DBGDOCK=1, AGC_DBGJETS=1,
  AGC_LMFOCUS=1 (chase camera on the LM only while docking). Lab: liftoff, liftoffkeep, stage,
  orbit, ascent:S, rndz, dock:M, dockpilot:S, thc:x,y,z.
- Still open: the PC-high tier's extra detail (plan step 3). The descent stage's crash, the flag
  blown over at lift-off and the APS's own sound were done 2026-11-16 evening.

## Quest session (2026-11-16, branch quest-vr: c631488, 9bdd4a1)
- render_quality() tiers (high PC / low headset; AGC_QUALITY and AGC_Q_* overrides) as shader
  #defines; the 2D UI as an OpenXR quad layer; hand tracking; menu pointer; controller models in
  the hands (src/vrhands.c) with the hinted button lit; per-eye frustum culling of terrain chunks
  and rock tiles; cockpit hull box-cut for leaning. Device 340YC10G9V07YY (Quest 3S). Unattended
  testing on the headset is impossible (it must be worn). Options file on the headset:
  /sdcard/Android/data/org.agclander.quest/files/agc-args.txt (always delete after a test run).
- Unresolved reports: seams that look like gaps, missing textures (not reproduced on the PC or
  the stand-in; screenshots from the headset wanted).

## The gaps closed (2026-11-16 evening, branch quest-vr)
User asked: flip W on PC and headset, and fix every gap in the game itself.
- W / stick forward now leans the LM forward (-pitch, as the real ACA and a flight sim): keyboard,
  gamepad, flight stick and the headset's right stick (controls.c, controls_vr.c). Hints and README
  follow. The 3D hand controller already tilted the right way (m3_ry is passive).
- Moonwalk, two crewmen (eva.c): the Commander and the LM Pilot. Playing alone the LMP is
  computer-run: out 9 s after the CDR is down (crawl, porch, ladder), keeps ~4 m ahead-right of the
  CDR (never under the LM), walks round blocks (veers when stuck), salutes the flag once, and climbs
  in FIRST when X is pressed at the ladder (the CDR waits, then follows; the hatch shuts behind the
  last man). Co-op: the joiner walks the LMP (CmdPkt carries an EvaNet, StatePkt both walkers and
  the flag; PROTO_VERSION 4). The LMP goes out only after the CDR. The spectator draws both.
- Figure (astro.c + lmmodel.c AP_* parts, materials 11 gold visor / 12 beta cloth): torso with
  chest unit and hoses, PLSS + OPS, helmet with visor, arms, legs by two-bone IK onto the drawn
  ground, overshoes; red stripes on the CDR (an Apollo 13+ convention, kept to tell the two apart).
  Poses: walk/lope gait, jump tuck, landing squat, ladder climb, porch crawl, flag planting,
  salute. First person draws only boots, shins and gloves (no gloves on the headset).
- Crew shadow map (render.c render_shadow2_*, CREW_SHADOW_GLSL in the terrain, LM and decal
  shaders, texture unit 7): follows the crew, sized to take in both and the flag, depth stretched
  for the ~10 m low-Sun shadows.
- Footprints (footprints.c + decal shader): multiplicative decals, the sole hollow and tread bars
  shaded against the Sun with self-shadowing; on landing of each hop (one boot) or jump (both);
  up to 900, cleared on a new flight. terrain_ground_height() = the finest mesh's surface (1 m
  grid, features < 1.5 m dropped, alternating diagonals): walkers, prints and the flag stand on it.
- Boulders: blocks >= 0.12 m within 30 m are cached; taller than STEP_UP (0.35 m) above the soles
  stop you at their edge + 0.32 m, lower ones are stepped onto (jumping onto a big one works).
- Headset hands: right grip takes the flag, letting go plants it under the hand; a grip on the
  ladder rail climbs in; in the cabin a grip on the hatch handle (by the knees) opens the hatch.
- Lift-off blast: a flag within 16 m goes over away from the stage (Apollo 11's did); further
  out it shakes. Descent-stage crash after an abort: impact speed recorded (lm.ds_impact_*; not in
  the save states), it tips over the way it was going and digs in; > 25 m/s it is drawn crushed;
  fireball if propellant is left, debris, a dust ring (lmmodel_draw_impact).
- Sound: the APS has its own voice (rough mid-band roar, combustion buzz, feed-line hiss, ignition
  cough), the staging pyros crack (on any ABORT STAGE), boot steps are felt through the suit.
- J skips the ascent burn: MS_INSERTION save state (snapgen_insertion: the LUNAR LIFT-OFF flight,
  auto crew, 2 s after the APS cutoff). Offered in P12 when lift-off was within 8 s of P12's TIG,
  after the vertical rise, no alarm, no keys. The descent stage keeps its own spot after the jump.
- Cockpit lettering: LEG 1.42 (PC) / 1.62 (headset), EL glow 0.85, glyphs drawn slightly bold.
- Looks from LM interior reference photos (cabin): cabin_wall map rebaked as flat painted
  sheets (0.45 x 0.30 m), screw rows along every seam, a raised frame and stringer with rivets (no
  more pillowed padding), K_WALL the lavender blue-grey, perforated cream overhead (procedural in
  the cabin shader). Exterior: lm_blanket rebaked shallow and crinkled (the low Sun had cut the old
  11-18 mm pillows into black/white pyramids), mylar bumpK 2.4 -> 1.5, black paint (material 3) is
  a satin charcoal with the panel map, a faint fill light on the LM (shadowed panels kept their
  seams instead of going flat black: the user's "missing textures"), the ascent stage roof's aft
  slope is shield panels. LM self-shadow bias is slope-scaled.
- Debug switches: AGC_EVA_LOOK=yaw,pitch, AGC_EVA_WALK=SECONDS (walk at the nearest big block),
  AGC_EVA_IN=SECONDS (press X), AGC_EVA_LOG=1 (phases), AGC_EVA_FLAG=east,south (plant near the LM),
  AGC_FOCUS_DS=1 (chase camera on the descent stage), AGC_SHADOW_ONLY=asc|desc, AGC_ABORT_AT=SECONDS.

## Holes in the LM, new skin maps (2026-11-23, branch quest-vr)
User's screenshots: black speckle on the aft equipment bay, flat grey "tiles", a hollow top.
- ROOT CAUSE: the LM is drawn with back faces culled, but many faces were wound inward, so
  from outside they were holes. prism() (and so every box()) built both end caps inward
  for counter-clockwise outlines; the crew compartment's main faces and both side walls
  were inward too. From above you looked into the ascent stage (the "U shape and two
  cylinders" screenshot); the black deck was the underside of a band 22 cm down; through
  the holes the crew compartment's black rear wall and the aft bay's front face (both at
  z = -1.32, exactly coplanar) fought pixel by pixel: the black speckle.
  Fixed: prism() orders any outline counter-clockwise and winds sides and caps outward;
  box() sorts its corners; cabin faces and side walls re-wound; docking window glass faces
  up; RCS nozzles got an inner wall + throat (open cones showed through); leg outrigger
  boxes were zero-thick and inside-out on the -y/-z legs; MESA plate moved inboard of the
  chamfer. tools/winding_check.py (with AGC_DUMP_LM) went from 371 visible inward
  triangles to 18 slivers. Rule: any new LM geometry must be wound outward - run it.
- lm_panel rebaked: irregular panels in staggered rows (2.52 m tile, no grid), per-panel
  tone, roughness and a slight tilt, screw rows along every edge. lm_foil rebaked: big
  irregular sheets of crumpled foil built from creased facets at three sizes (some sheets
  stretched into long wrinkles), sheet edges instead of a tape grid. The black tape strips
  over the descent stage are gone. Kapton colours copper-orange as in AS11-40-5927.
- Shader: anodised skin (material 10) metallic 0.85 / rough 0.40; foil rough 0.24; new
  material 13 black H-film blanket (descent top deck and its band); shadow lookups use the
  smooth surface normal; the reflected sky/ground horizon is softened by the normal's
  per-pixel change.
- Debug switches: AGC_DUMP_LM=FILE (every triangle as text), AGC_LM_NOCULL=1 (draw back
  faces too), AGC_DBG_MAT=1|x|y|z (flat colour per material, or 2 cm position bands).
- Perf finding (not acted on yet): at 2560x1440 HIGH GATE the world pass is ~21 ms on the
  2070 Super; gl_FragDepth (log depth) costs ~13 ms of it (AGC_Q_VZ=1 -> 8.7 ms).

## No fast-forward, no time warp: save states (2026-11-16, branch quest-vr)
Goal from the user: every scenario starts at once, the full mission feels shorter without
skipping time. Commits ca7e66f, 3268d90, 8769df0 and the scenarios commit after them.
- src/snapshot.c/.h: tagged, versioned sections (AGC, LM, CSM, CREW, MISN, SCEN), each module has
  ONE serialiser for save and load plus a layout fingerprint (a blob whose field list no longer
  matches is refused); rope CRC checked. AGC section: all of agc_t except the rope, the engine's
  hidden statics via agc_engine_hidden_io() (vendor patch in agc_engine.c, CountCDUX/Y/Z hoisted),
  the host's key queue, pending counter pulses, mct_debt, RHC, RR dither. Function-local statics
  in crew/csm/ascent moved to file scope. ANY NEW STATE that sim_advance reads must be added to
  its module's *_snap(), then `make snapshots`.
- agc-selftest: round trip (crew typing; mid-P63): hostile wipe (lift-off LGC, erasable XORed,
  engine statics and queues poisoned), restore, 60 s, byte compare. Mutation-tested: catches
  7 of 9 deliberately dropped fields; the 2 it cannot catch (pend_total, RHC counts) are
  self-correcting / overwritten. Also: every embedded milestone must load (stale = test failure)
  and the 150 M one must land.
- src/terrain_shape.c: the ground's shape without raylib (lab command `terrain`). Boulders are
  generated there too (rocks.c draws the same list, same RNG order: the old field is unchanged)
  and the physics surface includes them (terrain_rock_height, 4 m grid, r >= 0.3 m).
- `make snapshots` = agc-lab snapgen: the game's start + automatic crew from NASSP PDI-15 over the
  terrain; milestones TIG-30, RADAR (data good, before V57), HIGH GATE (2600 m), 150 M, a 1 Hz
  track, and two scenario flights; tools/embed_bin.py --multi -> data/snapshots.c (checked in).
- Menu: POWERED DESCENT/HIGH GATE/FINAL 500 FT load TIG-30/HIGH GATE/150 M (FINAL 500 FT keeps the
  P66 takeover). GS_FASTFWD, PgUp/PgDn, --warp, --alt, HUD "TIME x" removed.
- Advance to next event: J (PC), NEXT EVENT button on the headset display (point + trigger).
  Targets TIG-30, RADAR, HIGH GATE, offered only when milestones.c finds the flight on the
  canonical track (30 m, 0.3 m/s, 1.5 deg, program, flashing display, engine arm, PGNS mode, 16
  AGC flag words, no alarm, no keys going in) and the canonical crew does nothing before the
  milestone. Fade + Mission Control line. Debug: AGC_PRESS_J=SECONDS. tools/test_advance.sh.
- Scenarios (menu, under SCENARIOS): 1202 ALARMS = RADAR + agc_host_set_rr_dither(6400) (RR CDUs
  CDUT 035 / CDUS 036 counted up and down, each count steals an MCT: Luminary overflows by itself,
  1202 after V57, 1201 with V16N68); Houston's GO 20 s after each alarm (crew.c), the auto crew
  does V05N09E then RSET/KEY REL. LOW FUEL OVER THE BOULDERS = its own flight retargeted onto
  West crater's east rim (1500 blocks, ~8% of the rim covered), saved at 150 m with 100 s of
  hover fuel; the generator picks a spot where an upright LM's bell/base hits a block and trims
  the aim, so the computer's own landing is CRASH_BOULDER. Houston calls 60 / 30 s to bingo.
  ANOTHER SITE = retargeted 7 km downrange (Site 2 features in terrain_shape.c), from high gate.
  Retargeting = moving P63's LAND (stable-member, B24) by the same offset as RLS, like Apollo 12's
  N69 update (changing RLS alone does nothing once P63 has run IGNALG; V37E63E re-selection stalls
  on the astronaut clock). The canonical flight lands ~520 m E / 250 m N of its aim (navigation
  error in the NASSP state); the scenario flights aim that much short. tools/test_scenarios.sh.
- Found, not changed: the canonical touchdown point is on a 12.8 deg slope (it settles to ~12.6
  deg); the results grade only first contact.
- Possible next: an ascent milestone (insertion) so J can skip the 7-minute P12 burn; more
  scenarios from the same machinery.

## Hint walkthrough: checklists and key caps (2026-11-23, branch quest-vr)
- hints.c builds a checklist per phase: title, up to ~6 steps, each ticked (done), current
  (pulsing arrow, bold) or still to do. Steps are ticked off from the live state, so the
  list is right after a save-state load too. Checklists: pre-burn (6 steps), V57 key by key
  (VERB, 5, 7, ENTR), program alarms, lift-off (ABORT STAGE, ENG ARM ASC, PRO), take over
  and land (PGNS ATT HOLD, ROD, lean), moonwalk (flag via eva_flag_planted()).
- Key caps on the right of each step say what to press: keyboard keys on the PC
  ([F7], [P], [V] [5] [7] [ENTER], [UP] [DOWN], [W] [A] [S] [D]); in the headset the
  controller buttons instead (TRIGGER, B, L STICK) and the matching button on the
  controller model glows. When the auto crew is flying, no step is "current".
- New hint target HT_PGNS_ATTHOLD rings the PGNS mode switch (pointing down) in the cabin.
- Fix: the V57 hint now clears as soon as Luminary's LRINH flag (FLGWRD11 bit 8, set by
  LRON) is set, so keying V57 yourself ends it (before it only watched the crew's v57_done).
- Checked with --shot in scenarios 0, 3, 4, 5, 6 (chase and CDR views); make test passes,
  snapshots unchanged.
- Seen, not acted on: from the CDR seat in P63 the window shows the surface far below as a
  soft grey blur (both at 105,000 ft and 27,000 ft).

## Next
- GitHub: public repo + v0.1.0 release (2026-11-02). GPLv2 (Virtual AGC, NASSP data).

## Phase 3 original notes
- 3D terrain around Tranquility Base, low sun (~10.8 deg), shadows, LM model,
  plume, RCS puffs, dust, Earth, cameras (window / chase / landing-site),
  FDAI, cross pointer, tape meters, DSKY, sounds.

## Reference scalings
- CDU: 2^-15 rev/count. Gyro: 2^-21 rev/pulse. Coarse: 2^-13 rev/pulse.
- MCT = 11.71875 us.
