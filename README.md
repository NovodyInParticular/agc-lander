# AGC Lander

Land Apollo 11's Lunar Module with the **real flight software**, then fly it home. Luminary099,
the program that flew Eagle to Tranquility Base and back up to Columbia, runs here on an emulated
Apollo Guidance Computer (Virtual AGC core). The game simulates the vehicle around it: the IMU, the
landing radar, the descent and ascent engines, the stage separation, the RCS jets, the Moon, the
CSM and the crew procedures.

Single executable for Windows (`agc-lander.exe`) and Linux.

## Starting

Run `agc-lander.exe`. The game opens **fullscreen** (F11 switches to a window and back).

Pick a scenario with **Up/Down** and press **Enter**:

| Scenario | What happens |
|---|---|
| Full mission | P63 waiting at PDI −15 min. You run the real crew checklist. |
| Powered descent | 30 s before ignition: the AGC and the crew have run the checklist. PRO at V99. |
| High gate | 8,500 ft, just before P64 pitches over, with Tranquility Base coming into view. |
| Final 500 ft | 150 m up in P64; then it's yours in P66. |
| Lunar lift-off | Tranquility Base, one minute before ascent: P12, orbit, then docking with Columbia. |

Under **SCENARIOS**:

| Scenario | What happens |
|---|---|
| 1202 alarms | Radar data good at 29,000 ft, as on Apollo 11: the rendezvous radar's two CDUs, powered out of phase, count up and down 6,400 times a second each and steal the computer's time. Luminary's executive overflows by itself: 1202 and 1201 alarms, software restarts, and the guidance keeps flying. V05N09E shows the code, RSET clears the light, Houston calls GO. |
| Low fuel over the boulders | 500 ft, the LOW LEVEL light on, about 100 s of hovering left, and Luminary steering for West crater's boulder-strewn rim. The spot it has picked puts a block under the descent stage: take over in P66 and stop short or fly over. Houston calls 60 and 30 seconds. |
| Another site | Luminary was given a landing site 7 km downrange (an update to its target, as later missions uplinked): high gate over new ground with an old crater, a fresh blocky one and a crater chain. |

Boulders are solid: a block taller than the descent engine's bell clearance (about half a metre
above the footpads) or the stage's base under the LM breaks the descent stage.

Every scenario starts at once, with no fast-forwarding and no time warp. The later
ones begin from save states of one real flight: the AGC, the crew and the vehicle exactly as they
were at that moment of a descent the computer flew from PDI −15 min (`make snapshots` makes them).

During the long quiet stretches (the coast to ignition, the braking burn, the approach to high
gate) **J** skips to the next event: 30 s to ignition, landing radar data good, high gate. It is
offered only when nothing needs doing before then and your flight is still the one that save state
continues: same orbit, attitude, switches and computer state (programs, flags), no alarm. Fly it
differently, or leave something undone, and the time is yours to fly in full.
The ascent has one too: once P12 has the ascent stage climbing (lift-off on time, nothing
alarming), **J** jumps to orbit insertion; your descent stage stays where you left it.

New to this? Start with **Final 500 ft** and leave **plain-English hints** on (the last menu
entry, or **H** in flight). A box at the top of the screen tells you what to do next: which key to
press, which way to lean, when to cut the engine.

## Landing (and crashing)

Every touchdown is graded on a results screen: vertical speed, sideways speed, tilt, distance
from the target and fuel left. **Enter** flies the same scenario again, **Space** hides the panel.

| Result | Vertical speed | Sideways speed | Tilt |
|---|---|---|---|
| Good landing | up to 2.7 m/s (8.9 ft/s) | up to 0.6 m/s | up to 6° |
| Hard landing | up to 4.0 m/s (13 ft/s) | up to 2.0 m/s | up to 20° |
| Crash | faster | faster | more, or tipping past 40° afterwards |

Cutting the engine at the blue CONTACT light drops the LM the last 1.7 m, so a gentle 1 m/s
descent touches down at about 2.6 m/s. A crash folds the legs, throws debris, and if there is
propellant left and the impact is hard, the hypergolic propellants go up in a fireball.

## Lift-off and docking

After a landing (yours, or the LUNAR LIFT-OFF scenario), press **L** on the results panel: the
game skips the 21 hours on the surface and loads the computer exactly as Apollo 11's crew had it
one minute before lift-off (from NASSP's mission state), wherever your LM is standing.

1. **Lift-off.** The DSKY counts down in P12 (V06 N74, R1 = minutes and seconds to go). When it
   flashes **V99**, do what Aldrin called out: **B** (ABORT STAGE fires the bolts between the
   stages), **F7** (ENG ARM: ASC), then **P** (PRO). The ascent engine lights at ignition time;
   the descent stage stays behind as the launch pad, and its gold foil blows off.
2. **Ascent.** P12 flies a ten-second vertical rise, the pitch-over, and a seven-minute burn into
   an orbit about 17 x 90 km (Apollo 11: 17.4 x 86.5 km), steering into Columbia's orbital plane.
   Nothing to do but watch.
3. **Rendezvous.** The real one took 3 1/2 hours of burns (CSI, CDH, TPI, braking). **Enter** on the
   orbit panel skips to the end of it: Eagle station-keeping 40 m (130 ft) from Columbia, nearly
   lined up. The crew keys P00 and V48 for you (0.3 deg deadband, 2-jet translation).
4. **Docking.** Tab to the **DOCKING WINDOW** view: the Commander's overhead window, with the COAS
   sight. Columbia's docking target sits in the sight when the probe is on the drogue axis. Close at
   0.1-0.3 m/s; capture needs the probe within 33 cm of the axis, the two axes within 10 deg, and
   under 0.6 m/s. The readout on the right shows range, rate, offsets, drift, angle and roll.

| Key (in orbit) | Action |
|---|---|
| T / G | Translate along +X (toward Columbia) / back |
| Arrow keys | Translate sideways (as seen through the docking window) |
| W / S, A / D | Tilt (pitch, roll) |
| Q / E | Turn about the docking axis (yaw) |
| Gamepad | Right stick / bumpers: translation, as in the landing |
| Headset | Left stick + left grip / trigger: translation; right grip + right stick sideways: yaw |

**Aborting the landing.** ABORT STAGE (**B**, or the guarded button on panel 1) during the
descent does what it did in the real LM: the stages separate, the ascent engine is armed, and
Luminary switches to P71 and flies the ascent stage back to orbit. From there, rendezvous and
docking work the same way.

Hitting the drogue or Columbia too fast (above 0.6 / 0.45 m/s) breaks something: **Enter** then
backs off for another approach. A hard dock ends the mission.

## Controls

### General
| Key | Action |
|---|---|
| Tab | Next camera: chase, **cockpit**, 16 mm DAC, tower, cinematic, **lunar orbit**, **docking window** |
| F1 | Help overlay |
| F2 | Automatic crew on/off |
| F3 | 16 mm film look |
| F4 | Floating DSKY on/off (shown in the full mission; the cockpit has its own) |
| F5 / Pause | Pause |
| F6 | HUD on/off |
| H | Plain-English hints on/off (remembered between runs) |
| M | Cabin sound on/off (heard in the cockpit and 16 mm window views; outside views are silent, as in vacuum) |
| F10 | Outside view on your **second monitor** (press again to close it) |
| F11 | Fullscreen / window |
| J | Skip to the next event (only when nothing needs doing; see above) |
| Esc | Back to the menu |
| Right-drag, mouse wheel | Orbit and zoom the outside cameras |

### Flying
| Key | Action |
|---|---|
| W / S | Pitch: W leans the LM forward, S leans it back (rotational hand controller) |
| A / D | Roll |
| Q / E | Yaw |
| Up / Down | ROD switch in P66: each click is −1 / +1 ft/s rate of descent |
| Space | ENGINE STOP (press it at the LUNAR CONTACT light) |
| F7 | Engine arm: descent engine (or the ascent engine, after landing) |
| F8 | PGNS mode control: AUTO / ATT HOLD |
| B | ABORT STAGE (separates the stages; in flight it also arms the ascent engine and Luminary takes P71) |
| L | After landing: lift off (21 hours later, P12) |

### Joysticks and gamepads

You don't need one: the keyboard flies the whole landing. A stick is the most authentic way to fly, though.
In the real LM the Commander's **right hand held the ACA**, a pistol-grip attitude controller, and his
**left hand held the TTCA**, a T-handle for RCS translation that was also the descent-engine throttle in manual.

Controllers are detected automatically and hot-plug. A message shows which one was found.

**Flight stick (e.g. Logitech Extreme 3D Pro) = the ACA**
| Control | Action |
|---|---|
| Stick left/right | Roll |
| Stick forward/back | Pitch (forward leans the LM forward) |
| Twist | Yaw |
| Hat up / down | ROD switch (P66) |
| Hat left / right | Translate left / right |
| Trigger (1, hold) | DSKY PRO, e.g. enable ignition at V99 |
| Thumb button (2) | ENGINE STOP |
| Buttons 3 / 4 | ROD down / up |
| Button 5 | DSKY ENTR |
| Button 6 | Next camera |

**Gamepad (PS5 DualSense, Xbox, ...)**
| Control | Action |
|---|---|
| Left stick | Pitch / roll (forward leans the LM forward) |
| L2 / R2 | Yaw left / right |
| Right stick | Translate (TTCA) left/right, forward/back |
| L1 / R1 | Translate down / up (R1 = +X ullage) |
| D-pad up / down | ROD switch (P66) |
| Cross (A) | ENGINE STOP |
| Circle (B), hold | DSKY PRO |
| Square (X) | DSKY ENTR |
| Triangle (Y) | Next camera |
| Options (Start) | Pause |

**PS5 DualSense over Bluetooth** works too. To pair it: hold **PS + Create** until the light bar flashes, then
pair "DualSense Wireless Controller" in your Bluetooth settings (on Omarchy: the Bluetooth item in the
system menu, or `bluetui`). Any Sony pad is mapped automatically, whichever driver version the kernel
reports.

You can use the stick and the gamepad at the same time. Rotation works as rate command in PGNS
ATT HOLD (F8), the mode Armstrong flew the final approach in.

### DSKY (the computer)
| Key | DSKY key |
|---|---|
| V | VERB |
| N | NOUN |
| 0–9 | digits |
| = / - | + / − |
| Enter | ENTR |
| C or Backspace | CLR |
| R | RSET |
| P (hold) | PRO |
| K | KEY REL |

You can also click the DSKY keys with the mouse, on the side panel or in the cockpit.

### The crew procedure (Full mission)
1. R51 asks V50 N25: press **ENTR** (skip the fine align).
2. V50 N18: press **PRO** to start the auto maneuver, then **ENTR** when it's done.
3. Arm the descent engine (**F7**, or the ENG ARM switch on panel 3).
4. V99 N62 at TIG −5 s: press **PRO** to enable ignition.
5. After landing-radar lock, key **V57E** to let the radar update the computer.
6. Hit **ENGINE STOP** when the blue LUNAR CONTACT light comes on.

F2 hands all of this to the automatic crew, which works the way Armstrong and Aldrin did.

## The cockpit

Tab to **COCKPIT**. You stand at the Commander's station of the LM cabin.

| Input | Action |
|---|---|
| Right-drag | Look around |
| Mouse wheel | Zoom |
| Shift + wheel, or L | Lean toward the window |
| Home | Look out of the window |
| End | Step back and look down at the lower panels (again: stand up) |
| X | After landing, engine off: open the hatch and go outside; outside, at the foot of the ladder: climb back in |
| W A S D, Shift, Space, F | On the surface: walk, lope, jump, plant the flag (right-drag or arrow keys to look) |
| (moonwalk) | Playing alone, the LM Pilot follows you out, keeps you company and climbs in first; in co-op the joining player walks him. Boulders taller than a step block your way; you leave boot prints |
| (headset, moonwalk) | Squeeze the right grip to take the flag, let go to plant it where your hand is; grab the ladder to climb in; in the cabin, grab the hatch handle by your knees to go out |
| F9 | Swap to the LM Pilot's station |
| Left click | Operate switches, knobs, buttons and the DSKY |

Hovering the mouse over a control shows its name and position. Controls that the simulation does
not model still move, and their tooltip says "(not modelled)".

**Switches that work:**
- ENG ARM and PGNS MODE CONTROL (panel 3)
- RATE OF DESCENT and ENGINE STOP (panel 5)
- PLUS X (ullage)
- MASTER ALARM (press it to silence)
- FLOOD and ANUN/NUM dimmers (panel 5/4), and the INTEGRAL dimmer and LAMP TEST (panel 3)
- X-POINTER SCALE, RATE SCALE and MODE SEL (landing radar or PGNS altitude on the tape)
- Event timer start/stop/reset

**Instruments:** FDAIs driven by the IMU and the AGC's error needles, the X-pointer, the altitude
and altitude-rate tapes, T/W and thrust, propellant quantity, the event and mission timers,
caution and warning lights (DES QTY lights at low fuel), the DSKY, the AGS DEDA, and circuit
breaker panels 11 and 16.

**Landing Point Designator:** the angle scale is etched on the Commander's window. Read the
LPD angle from DSKY R1 in P64 and look along that line on the glass to see where the computer is
landing. The scale lines up from the design eye position, so leaning in moves it, just as in the
real LM.

## Second monitor

Press **F10**, or start with `--dual`. A second window opens on your other monitor and shows the
outside (chase) view, live, while you fly from the cockpit on the main screen. Tab in that window
cycles its cameras, and Esc closes it.

On tiling window managers (Hyprland and others), add a window rule for the title
`AGC Lander - Outside View` to put it fullscreen on the monitor you want.
On Windows, it opens on the monitor the game is not on.

## Two players (co-op)

The real LM flew with two crew, and so can you:
- **Commander (host):** runs the real AGC and the vehicle, and flies (hand controller, ROD switch, ENGINE STOP).
- **LM Pilot (joins):** gets the right-hand station. They run the DSKY, the AGS DEDA and the panels, and work
  ENG ARM, PGNS mode and their own ENGINE STOP. Their key presses and switch throws go to the host's
  computer, so you're both flying one Lunar Module. Their hand controller and ROD inputs are added to the
  Commander's, so agree who flies.

**Host:** in the menu, set **TWO PLAYERS: HOST** (Enter), then start any scenario. Or run `agc-lander --host`.
**Join:** choose **JOIN A GAME AS LM PILOT**, type the host's IP address and press Enter. Or run
`agc-lander --join 192.168.1.50`.

- Both players need the same version of the game.
- It uses **UDP port 47012** on the host. On a LAN that usually just works; on Windows allow the
  firewall prompt; on Linux with ufw: `sudo ufw allow 47012/udp`. Over the internet the host
  must forward UDP 47012 on their router.
- The host owns time: pause and skipping ahead are the Commander's.
- Esc on the LM Pilot's side leaves the game.

## Lunar orbit view

Tab to **LUNAR ORBIT** for the whole Moon: maria, ray craters and the day/night terminator.
Right-drag pans over the surface, Ctrl + right-drag turns the heading, the wheel changes altitude,
and Home takes you back over Eagle.

## Installing

**Windows:** unzip and run `agc-lander.exe`.

**Linux / Omarchy:** unpack `agc-lander-linux-x86_64.tar.gz` and run `./install.sh`. That installs
to `~/.local` and adds **AGC Lander** to the app launcher (Super + Space on Omarchy). From source,
run `make install`.

## Command-line options

`--windowed`, `--monitor N`, `--dual`, `--host`, `--join ADDRESS`, `--demo`, `--start 0-3` (skip the menu), `--camera N`

## Building

`make` (Linux), `make windows` (mingw-w64), `make test` (AGC self-test, an automatic landing, a P12 ascent,
a docking flown with the hand controllers, and the whole mission from PDI to hard dock).
