// SuperTeam shared types/constants.
// Lives in a header (not a .ino) on purpose: the Arduino builder concatenates
// .ino files and auto-inserts function prototypes at the top of the merged
// file, so struct types and macros used across .ino boundaries must come from
// a header included before everything else.
#ifndef superteam_h
#define superteam_h

// ingredient bitmask (matches the five single-colour ingredient targets)
#define ING_RED    0x01
#define ING_YELLOW 0x02
#define ING_GREEN  0x04
#define ING_BLUE   0x08
#define ING_BLACK  0x10

#endif
