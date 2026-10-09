Scratch scripts from the chute redesign of 8 Oct 2026 (spec section 6.2). They are a record of how the final numbers were found, not a check, and they were written against the
first-design parameters: they may need small changes to run against the final v3_params4.

head.py        parametric hopper and channel head on chute_geometry's convex-piece helpers (bore width and height, trough style, slot and void width, lead-ins)
design_run.py  runs a head in MuJoCo for a list of friction values, e.g.
               PYTHONPATH="C:/Users/christopher.shu/pl4" python design_run.py "W=1.8,H=1.6,ceil_extra=0.2,slot=1.6,void=1.6,trough=floor,trough_size=2.3,damp=0.3,par=0.2" 100 29 0.15,0.3,0.45,0.6
               (run from this folder with v3-checks on the path; arguments: design string, kits per friction value, seed, friction list)
dbg2.py        a debugging run of one design

What the sweeps showed (kits 10.3 mm, 1.2 g, random turn up to +-29 degrees; friction, mass and impact softness are assumptions):
- a 15 mm bore with the trough cut down to the bore floor took 120 of 120 kits at mu 0.2, 0.35 and 0.5 with the plate parked exactly, also with bouncy impacts (damping ratio 0.3); 14.5 and 14.0 mm bores took only 91 and 96 of 120;
- with the plate parked 2.5 mm off, kits wedged diagonally in the 14.5 mm slot (narrower than the cube's 14.57 mm face diagonal), so the slot and the hopper void became 16 mm;
- with bouncy impacts a 16 mm bore still wedged tumbling kits between its walls (a cube in a general attitude is up to its 17.84 mm space diagonal across), so the bore became 18 mm;
- a radial slot (114.9 degrees on the ring) was tried first and rejected (eight pockets at 30 degrees do not fit between slots 130 degrees apart, and it still jammed at friction 0.35 to 0.45);
- a channel extended back behind the slot centre gave a meaningless result (the kit sat on the tube corners) and was dropped.
The final design and its results are in chute_dynamics.py and results/chute_dynamics.txt.
