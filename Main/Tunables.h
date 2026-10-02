#ifndef tunables_h
#define tunables_h
// Values that used to be compile-time constants / literals and are now runtime-tunable
// from the web tuner (see tuner/index.html and Tuning.ino).
// Defaults live in Tunables.cpp and are identical to the values that were hard-coded before.

struct PidGains {
  double kp, ki, kd;
};

struct TuneState {
  PidGains center;   // fwd(): wall-follower centering
  PidGains scale;    // fwd(): encoder distance scaling
  PidGains gyro;     // fwd(): heading hold (currently commented out in fwd())
  PidGains climb;    // fwd(): heading hold while on a ramp
  PidGains turn;     // absoluteturn()
  PidGains parallel; // obstacleavoidance(): PARALLEL step
  PidGains wiggle;   // obstacleavoidance(): WIGGLE step
  float colBlack;    // black tile: clear-channel / baseline ratio below this
  int   colSilver;   // silver tile: red channel above this
  float colWhite;    // white tile: clear-channel / baseline ratio above this
};

// One row of the parameter table the web tuner edits (table itself lives in Tuning.ino).
// Defined here, not in Tuning.ino, so Arduino's auto-generated prototypes can see the type.
enum TuneType : unsigned char { TT_DOUBLE, TT_FLOAT, TT_INT };
struct TuneParam {
  const char* name;   // e.g. "center.kp"
  const char* group;  // heading shown in the tuner UI
  const char* desc;   // one-line help (no double quotes)
  void* ptr;          // variable being edited
  TuneType type;
  double lo, hi, step; // allowed range and UI nudge size
};

extern TuneState tune;
extern int SENSOR_OFFSET_MM[8]; // Distance.ino, per-sensor offset in mm, index 1..7
extern float clear;             // Main.ino, color-sensor baseline clear value

#endif
