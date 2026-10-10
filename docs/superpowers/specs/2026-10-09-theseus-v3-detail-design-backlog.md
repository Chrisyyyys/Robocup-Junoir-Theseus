# Theseus V3: what still needs detailed design (9 Oct 2026, updated 10 Oct)

**Updated on 10 Oct (sixth round):** O4 (the arm drawing) and O5's order sheet are done, O6 to O9 changed with the spring's legs (a straight rear leg on an M3 x 6 bolt, a forward leg on a 2 mm pin in the arm, a printed arbor), O8's sleeve was re-sized, and O14 is new; rows touched that day say **10 Oct**. Written on 9 Oct, after the omni mount was designed ([mechanical design](2026-10-07-theseus-v3-mechanical-design.md), section 4 and Figure 12). It lists everything in the V3 design that is **not yet detailed enough to print, buy or wire**, and what each item needs from you. Nothing here is a new requirement: the items come from the flags in the design (**not designed**, **[placeholder]**, **[estimate]**, **[measure]**, **[proposal]**), from the open list in section 13, and from what the omni work turned up. The omni module itself is now designed down to its screws; the omni items below are what that design still leaves open.

How to read it:

- **P1** = settle before the part is printed or bought (a wrong guess means a reprint or a wrong purchase). **P2** = needed before the robot can run a whole course. **P3** = refinement, can follow the first robot.
- **Who**: *you* = a measurement, a purchase or a decision only you can make; *me* = a design task I can do in the model and the spec; *both* = I design it after you supply an input.
- Status words: **none** = nothing drawn; **placeholder** = a box or a guess stands in; **estimate** = read from a photo or inferred; **decided** = chosen, unmeasured.

## 1. Omni module (section 4): what the mount design leaves open

| # | Item | What the detail design must give | Status | Pri | Who |
|---|---|---|---|---|---|
| O1 | Wheel bore and hub | **Buy one Nexus 14145 and measure it**: depth of the 12 mm hole (the design needs it through, or at least 9 mm from each face for the bearing and the screw head), its tolerance, how a 604 bearing (12 mm outer) seats in it, hub width, whether the face has a rim that catches the screw head. The sleeve length (12.6 mm) and the 5 mm bearing seat come from this | **decided**; the datasheet gives only "axle hole 12 mm" | P1 | you (buy and measure), then me |
| O2 | Bearing retention | how the two outer rings stay in a smooth bore: press fit plus retaining compound (Loctite 641), or a circlip groove if the wheel allows one; whether the inner rings are clamped hard enough by the screw (head, washer, sleeve, spacer, arm) | **proposal** | P1 | both |
| O3 | Axle thread in the arm | M4 thread 2.9 mm deep in a 3 mm aluminium bar (about four turns): pull-out and loosening test, or a thicker arm end, or a threaded insert; thread-locker grade. **Access:** the screw is turned from below with a ball-end 3 mm key at an angle, because the silver module sits 1.6 cm inboard of the wheel at the same height: check this again when the module is chosen (H2) | **decided** | P2 | both (bench) |
| O4 | Arm drawing and manufacture | **10 Oct: drawn.** [Figure 13](v3-figures/fig13_omni_arm.png) and [omni_arm.dxf](v3-figures/omni_arm.dxf): 6061-T6 bar 12 x 3 mm, 58 mm long, bore **5.05 H8** reamed (the tube is 5.00), M4 x 0.7 tapped through (drill 3.3), the **arc slot** (8 mm radius, 3.3 mm wide, round ends; a mill, or drill-and-file), a **2 mm seat hole** for the spring's pin, all with positions, edge distances (1.0 mm of bar over the seat hole, 2.1 mm above and below the slot), 4.9 g, and the way to file the ride height (slot ends set it: 0.2 mm off = 1.1 mm of height; leave the rest end 0.1 mm short). A PETG copy for the first fit only (it twists the wheel 2.2 degrees under load; aluminium 0.14). **Still open: who cuts it** (a laser or waterjet service takes the DXF; or saw, drill and file) | **drawn** | P1 | you (who cuts it) |
| O5 | Torsion spring | **10 Oct: order sheet written** ([Figure 14](v3-figures/fig14_omni_spring.png)): music wire 1.6 mm, mean coil 10 mm (OD 11.6), **right-hand, 4.07 turns**, straight tangent legs (rear 10 mm to its load point and 12 in all, forward 11 and 12), free angle 203.6 degrees, 8.6 N.mm per degree, 107 N.mm at 12.4 degrees of opening and 378 N.mm at 44.0 (+-10 %), stress relief, 3 pieces. The legs have their supports (rear on the adjuster bolt, forward on a 2 mm pin in the arm, no notch needed) and the coil its arbor (spec 4.3). **Still open: a supplier and the lead time, and the life test** (1055 MPa at full travel is 48 % of the wire's strength with the load opening the coil: run 10 000 cycles of the spring alone, torque to stay within 5 %); keep the compression-spring fallback (4.3 N preload, 15 N at full travel) in mind | **order sheet ready**, no supplier | P1 | you (supplier, test) |
| O6 | Preload adjuster and set-up procedure | **10 Oct: an M3 x 6 hex bolt in an M3 x 5.7 insert in the pillar's tail**, under the straight rear leg (one turn = 0.55 N at the axle; the leg lies over the middle of the head with 2 mm to spare): the turning access (5.5 mm socket from the side), a force-travel measurement at the axle with a spring scale (2.4 N preload, 8.5 N at 2.5 cm), a mark for the set position. With a spring that is a little stiffer or softer than ordered the bolt takes up the preload (+-1 mm of head height is +-1.1 N) | **decided** | P2 | me (procedure), you (bench) |
| O7 | Pivot hardware | tube material (brass or steel, 5 / 3.1 x 13.5 mm, tolerance of the length), grease, a 1 mm washer on the ear side to take up the arm's 1.2 mm of sideways float, the M3 x 5.7 insert in the ear, the M3 x 25 pivot screw and its thread-locker; **10 Oct: the arbor** (printed 7.4 / 5.2 x 9 mm sleeve on the tube under the coil: print it standing on its end; drawn 5.2 mm inside because a printed hole comes out 0.1 to 0.2 mm small: it must slide on the 5.00 tube without rattling, ream it to 5.1 if it binds) | **decided** | P2 | me |
| O8 | Stop screw | thread in the printed ear (M3 into a 2.6 mm pilot; an insert if the thread strips), thread-locker, access with a 2.5 mm hex key from inboard. **Wear, 10 Oct:** the screw is threaded along its whole length and its threads bear on the aluminium slot ends: about 170 MPa at the thread crests at rest (13 N), about 400 MPa in the 70 N impact case (aluminium yields at 240), assuming 0.5 mm of crest contact **[estimate]**: a small dent that spreads the load, **accepted for the first robot (D31)**. If the stair test shows the rest end creeping: a thin sleeve (brass or steel tube **3.6 mm outside**, as long as the pivot tube, slot 3.9 mm wide: 0.4 mm from the coil, 0.8 mm from the forward leg); a 4 mm sleeve or a shoulder screw no longer fits, and neither would the 3.0 mm sleeve of the 9 Oct note (it would not clear the thread). Or hard-anodise the arm after machining | **decided**, sleeve optional | P2 | me |
| O9 | Pillar, tail and ear in the tub CAD | fillets and ribs, print orientation (the pillar is a tall thin wall; the stop load of up to 70 N should not act across layers), supports for the pocket, heat-set insert holes (the ear's, and since 10 Oct the tail's for the adjuster bolt: the tail is 8.5 mm wide, y 0.35 to 1.2); the opening's edges | **in the model as plain blocks** | P2 | me |
| O10 | Ride height and belly | procedure to measure and trim the rest height (file or shim the slot end), tolerance stack (wheel diameter, slot, prints: about +-1 mm), a check that the belly line (z 3.5) still clears the stairs | **none** | P2 | both |
| O11 | Dirt and debris | the wheel and its bearings run in an open bay under the body: decide whether the maze floor's dust and hair need a shield or a brush; **not designed** | **none** | P3 | me |
| O12 | Omni terrain, marginal cases | the 30 degree stairs up case has **0.20 cm** against a 0.2 cm threshold (no margin, as on 8 Oct); the Dangerous Zone 2 cm bump on a 25 degree ramp **fails** as in rev 3 (D8). A real stair test decides | **known** | P2 | you (test) |
| O13 | Wi-Fi antenna in its new place | the antenna moved to -22 degrees on the right bumper's recess wall (it overlapped that wall at -17.5): 8 degrees from the bumper switch and 5.6 mm under the USB-C socket. Range test with the robot assembled | **proposal, untested** | P3 | you (test) |
| O14 | Seat pin and arbor (10 Oct) | the **2 mm stainless pin** in the arm (slip fit in a 2.0 H7 hole with retaining compound, driven in from the inboard face, 3.5 mm out, 2.5 mm in the bar) and the **printed arbor**: check that the leg stays on the pin through the travel (it turns with the arm, so it should not slide; if it walks off, a 0.3 mm groove across the pin), that the coil does not bind on the PETG arbor (a drop of grease) and that the legs lie within 1.6 mm of their end turns' planes | **in the model** | P2 | both (bench) |

## 2. Parts with no holder or no geometry yet

| # | Item | What is missing | Status | Pri | Who |
|---|---|---|---|---|---|
| H1 | Front floor port FP (TCS34725 board, 20.3 mm) | pocket in the floor at (7.5, +3.5), face at z 2.8, two M2.5 posts (holes 15.24 mm apart), cable up, a light shield (D26) | **none** | P1 | me |
| H2 | Silver module SM | the part itself is not chosen (silver bench test), so its holder cannot be drawn (D26) | **placeholder box** | P1 | you (choose), then me |
| H3 | Dropper unit hold-down | the two snap tabs or thumbscrews on the seats; wear of the half-lap rebates (100 lift-outs) | **none** | P2 | me |
| H4 | Hopper and channel joints | the two snap tabs of each hopper flange, the channel's snap detent, the two-half split of the open-trough channel (not redrawn), a 1 mm TPU pad if the kits bounce | **proposal** | P2 | me |
| H5 | Plate hub and coupling | how the kit plate sits on the N20's 3 mm shaft (friction plus an M2 set screw), the hub; the 5 mm hub with flats for the 28BYJ-48 variant | **none** | P1 | me |
| H6 | N20 cartridge | face plate, two M1.6 screws, the pocket's snap tabs; face-hole spacing **[measure]** | **estimate** | P1 | both |
| H7 | Battery tray and the battery | model and size not chosen: the removal path of section 3.3 depends on it; strap slots, connector side, front stop, the 1 mm foam pad, a charge port (not designed) | **placeholder 7.0 x 3.5 x 2.5 cm** | P1 | you (choose), then me |
| H8 | Control deck | recesses for the power switch (1.3 x 1.9 cm, or the breaker), the start button, two status LEDs; the parts are not chosen | **placeholder** | P2 | both |
| H9 | USB-C service socket | the panel-mount part, how it is held flush in the wall, the right-angle plug and cable to J12 | **placeholder** | P2 | both |
| H10 | Bumpers | hinge pin and its wall socket, return spring, microswitch (part not chosen) and its pocket; the CAD has the plates and switch pockets, not the hinge or the spring | **rev 3 text only** | P2 | both |
| H11 | Rear nub | M5 nylon bolt, PTFE or ball tip, jam nut, boss thread | **boss only** | P3 | me |
| H12 | Drive wheel hubs and tyres | hub on the 4 mm D-shaft with the M3 set screw; the 80 mm tyre material (the 2 cm stair test depends on its friction) | **solid cylinder** | P1 | both |
| H13 | Camera cages | print orientation, the removal path in the Fusion report (not run), cable routing, the lid humps; all rest on an **estimated** lens block (six caliper readings: Figure 11) | **estimate** | P1 | you (measure), then me |
| H14 | ToF mounts | plug shaft and cable strain relief, the 200 mm cables, access to the two screws per board | **geometry only** | P2 | me |
| H15 | Lid | hook strain and retention (test print), finger notches, the optional lid switch | **calc only** | P3 | me |
| H16 | Handle and victim LED | post-to-bar joint (two M3), the LED's wires and connector at the bar (D19) | **geometry only** | P2 | me |
| H17 | GIGA posts and header tails | check the header solder tails against the floor modules; M3 inserts | **geometry only** | P2 | me |
| H18 | Wiring harness | about 15 connectors between the frame and the tub (ToF, cameras, N20, control deck): routes, strain relief, labels, the frame lying beside the tub on its 200 mm cables | **none** | P2 | me |
| H19 | Main PCB (shield) | outline (it starts about 20 mm from the GIGA's connector edge), connector positions, the front-edge and outboard-strip rule; out of scope of the mechanical design but nothing can be wired without it | **rules only** | P2 | you (PCB), me (rules) |

## 3. Measurements, purchases and choices only you can make

| # | Item | Why it matters | Pri |
|---|---|---|---|
| M1 | **Buy the Nexus 14145 omni wheel** (long lead time) and measure its bore, width and mass | the whole omni mount rests on its datasheet; O1 and O2 | P1 |
| M2 | Six camera caliper readings (Figure 11) and the board thickness | cage and lid hump | P1 |
| M3 | Which version of the VL53L0X board 3317 you own (an older board has another outline) | the ToF pockets | P1 |
| M4 | Pololu 20D face-hole spacing and the 3499 encoder outline | cradle, face plate | P1 |
| M5 | N20 face-hole spacing | N20 face plate | P1 |
| M6 | Printer bed size | whether the tub and frame split along y = 0 | P1 |
| M7 | Battery model and size | tray, strap, removal path | P1 |
| M8 | Power switch or breaker size; start button; LEDs; USB-C panel socket | control deck, socket hole | P2 |
| M9 | GIGA connector, J14 and reset and boot button positions (read from a picture, +-1 mm); BNO055 hole positions | USB-C plug clearance, antenna cable | P2 |
| M10 | Bearings: the BOM says "size TBD, 4 existing + 5 spare": confirm that the stock is 604-2RS (4 x 12 x 4 mm), and which M4 and M3 screws, inserts and thread-locker you have | omni parts list (Figure 12) | P1 |
| M11 | Weigh the printed parts and the bought parts | every mass is a placeholder; the lift limit (1.58 m/s^2 now) and the spring preload scale with it | P2 |

## 4. Electronics and firmware changes the mechanics need

| # | Item | Pri |
|---|---|---|
| E1 | Qwiic multiplexer: the TCA9548A has 8 channels for the nine ToF boards, and the BOM counts 7 ToF boards as required against 9 in the design: a second mux or another way (XSHUT or addresses) | P1 |
| E2 | Floor sensors 7.5 cm ahead: silver is read 75 mm earlier, so `overNextTile` becomes TILE_MM/2 - 75 mm and the reading is latched | P2 |
| E3 | ToF positions moved: `TOF_SIDE_OUT_MM` 72.8 to 60.0, `TARGET_SIDE_GAP_MM` 67.2 to 80.0; the toed-out front readings were not recomputed after the 9 Oct move | P2 |
| E4 | IMU in **IMU mode** (no magnetometer) until it is calibrated in place (the V2 heading loss) | P2 |
| E5 | PWM ramp 0.3 s (the lift limit is 1.58 m/s^2, a 0.25 s ramp to 0.3 m/s needs 1.2); the front ToF readings need IMU-pitch gating (the nose pitches 17.4 degrees coming down a 2 cm riser) | P2 |
| E6 | Dropper control: the plate must park within +-1.5 mm (aim +-0.5 mm) approaching from one side every time; homing and a parking method with the N20's encoder are **not designed** | P1 |
| E7 | Radio test mode: chosen at power-up, never started in a run (rules 4.1.1) | P3 |

## 5. Bench tests that can change the design

Chute with 20 cubes at 32 degrees and real kits (friction, bounce, a printed bore); plate parking repeatability from both sides; 2 cm stair test with the real tyres (the one real risk of 2WD); 5 kg hang from the handle and frame-screw pull-out; 100 lid cycles; 100 dropper-unit lift-outs; a printed cradle for each motor; one ToF pocket printed with a board fitted; one camera cage fitted; **the omni: spring force against travel, spring life (10 000 cycles), axle-thread pull-out, the seat pin's fit, the slot end after the first landings (dent), ride height, then the stair test**; Wi-Fi range with the robot assembled; the N20 against the 28BYJ-48 (rev 3 test 7).

## 6. Suggested order

1. **This week, you:** order the omni wheel and the torsion spring (O1, O5, M1), measure the camera and the battery (M2, M7), check the ToF board version (M3), and tell me the printer bed size (M6). These have lead times and decide reprints.
2. **Me, in parallel:** the arm drawing (O4, **done 10 Oct**) and the spring order sheet (O5, **done 10 Oct**), FP and SM holders (H1, H2 once the module is chosen), the plate hub (H5), dropper hold-down (H3) and hopper joints (H4), the bumper hinge (H10), wheel hubs (H12).
3. **First prints:** one drive cradle, one ToF pocket, one camera cage, the omni pillar and ear on a short piece of floor with a PETG arm: the cheap ones that settle fits.
4. **After the first robot runs:** the spring life test, the stair test, ride height (O10), dirt (O11), the harness and the shield PCB (H18, H19).

## Decisions still waiting for you

D8 (the Dangerous Zone corner case that fails), D27 (the Nexus 14145 as the omni wheel), D28 (the aluminium arm and the mount of section 4), D29 (the layout shifts the real wheel forced: omni axle x 6.85, wheel 3 mm left of centre, floor port y 3.5, antenna at -22 degrees); the earlier D1 to D7 of rev 3 are unchanged.
