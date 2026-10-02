"""Turn the Arduino sketch in Main/ into one C++ file, the way the Arduino IDE does.

The Arduino IDE glues every .ino file of a sketch together (the main sketch first,
then the others in alphabetical order), adds `#include <Arduino.h>` at the top and
generates a prototype for every function so functions can be called before they
are defined.  This module does the same so the robot code compiles unmodified on
a PC.  It also appends the simulator "probe" (sim/core/sim_probe.inc), which gives
the simulator read access to the sketch's globals (robot state, map, position).

Nothing in Main/ is changed.
"""

import os
import re

FUNC_SKIP_FIRST_WORDS = {
    "if", "while", "for", "switch", "else", "do", "return", "struct", "class",
    "enum", "union", "namespace", "typedef", "using", "template", "extern",
}


def blank_comments_strings_preproc(src):
    """Return a copy of src where comments, string/char literals and preprocessor
    lines are replaced by spaces (newlines are kept) so positions stay identical."""
    out = list(src)
    i, n = 0, len(src)
    at_line_start = True
    while i < n:
        c = src[i]
        if c == "\n":
            at_line_start = True
            i += 1
            continue
        if at_line_start and c in " \t":
            i += 1
            continue
        if at_line_start and c == "#":
            # preprocessor line, honour backslash continuations
            while i < n and src[i] != "\n":
                if src[i] == "\\" and i + 1 < n and src[i + 1] == "\n":
                    out[i] = " "
                    i += 2
                    continue
                out[i] = " "
                i += 1
            continue
        at_line_start = False
        if c == "/" and i + 1 < n and src[i + 1] == "/":
            while i < n and src[i] != "\n":
                out[i] = " "
                i += 1
            continue
        if c == "/" and i + 1 < n and src[i + 1] == "*":
            out[i] = out[i + 1] = " "
            i += 2
            while i < n and not (src[i] == "*" and i + 1 < n and src[i + 1] == "/"):
                if src[i] != "\n":
                    out[i] = " "
                i += 1
            if i < n:
                out[i] = out[i + 1] = " "
                i += 2
            continue
        if c in "\"'":
            quote = c
            out[i] = " "
            i += 1
            while i < n and src[i] != quote:
                if src[i] == "\\":
                    out[i] = " "
                    i += 1
                if i < n and src[i] != "\n":
                    out[i] = " "
                i += 1
            if i < n:
                out[i] = " "
                i += 1
            continue
        i += 1
    return "".join(out)


def _split_signature(sig):
    """Return (text_before_first_paren, params_text) or None."""
    p = sig.find("(")
    if p < 0:
        return None
    depth = 0
    for j in range(p, len(sig)):
        if sig[j] == "(":
            depth += 1
        elif sig[j] == ")":
            depth -= 1
            if depth == 0:
                tail = sig[j + 1:].strip()
                tail = re.sub(r"\b(const|noexcept|override|final)\b", "", tail).strip()
                if tail:
                    return None
                return sig[:p], sig[p + 1:j]
    return None


def _has_default_args(params):
    depth = 0
    for ch in params:
        if ch in "(<[{":
            depth += 1
        elif ch in ")>]}":
            depth -= 1
        elif ch == "=" and depth == 0:
            return True
    return False


def find_function_definitions(clean):
    """Yield (sig_start, sig_text) for every top-level function definition."""
    depth = 0
    stmt_start = 0
    results = []
    for i, c in enumerate(clean):
        if c == "{":
            if depth == 0:
                sig = clean[stmt_start:i]
                text = " ".join(sig.split())
                parts = _split_signature(text) if text else None
                if parts:
                    head, params = parts
                    words = head.replace("*", " * ").replace("&", " & ").split()
                    if (len(words) >= 2 and "=" not in head
                            and words[0] not in FUNC_SKIP_FIRST_WORDS
                            and re.fullmatch(r"[A-Za-z_]\w*", words[-1])
                            and "operator" not in head):
                        start = stmt_start + (len(sig) - len(sig.lstrip()))
                        results.append((start, text, _has_default_args(params)))
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                stmt_start = i + 1
        elif c == ";" and depth == 0:
            stmt_start = i + 1
    return results


def find_enum(clean, src, name):
    m = re.search(r"\benum\s+" + re.escape(name) + r"\s*\{", clean)
    if not m:
        return []
    body_start = m.end()
    body_end = clean.find("}", body_start)
    body = clean[body_start:body_end]
    names = []
    for part in body.split(","):
        part = part.strip()
        if not part:
            continue
        if "=" in part:
            return []  # explicit values: give up, probe falls back to numbers
        names.append(part)
    return names


def ino_files(sketch_dir):
    main_name = os.path.basename(os.path.normpath(sketch_dir)) + ".ino"
    files = sorted(f for f in os.listdir(sketch_dir) if f.endswith(".ino"))
    if main_name in files:
        files.remove(main_name)
        files.insert(0, main_name)
    return files


def cpp_files(sketch_dir):
    return sorted(os.path.join(sketch_dir, f) for f in os.listdir(sketch_dir)
                  if f.endswith((".cpp", ".c")))


def merge_sketch(sketch_dir, out_cpp, probe_path, state_names_header, display_root=None):
    """Write the merged C++ file. Returns a dict with info about what was found."""
    files = ino_files(sketch_dir)
    if not files:
        raise SystemExit("no .ino files found in " + sketch_dir)
    display_root = display_root or os.path.dirname(os.path.abspath(sketch_dir))

    chunks = []
    for f in files:
        path = os.path.join(sketch_dir, f)
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        if not text.endswith("\n"):
            text += "\n"
        chunks.append((path, text))

    prototypes = []
    moves_limit = -1  # the "iterator >= N" limit that sends the robot home
    void_fns = []  # functions with signature `void f()`, used to name RTOS threads
    first_def = None  # (chunk index, line number)
    enums = {}
    for ci, (path, text) in enumerate(chunks):
        clean = blank_comments_strings_preproc(text)
        for start, sig, has_defaults in find_function_definitions(clean):
            if first_def is None:
                first_def = (ci, text.count("\n", 0, start))
            name = sig[:sig.find("(")].split()[-1]
            if re.fullmatch(r"(static\s+)?void\s+\w+\s*\(\s*(void)?\s*\)", sig):
                void_fns.append(name)
            if has_defaults:
                continue  # like the Arduino IDE: no prototype for functions with default arguments
            if name in ("setup", "loop"):
                continue
            prototypes.append(sig + ";")
        m = re.search(r"\biterator\s*>=\s*(\d+)", clean)
        if m and moves_limit < 0:
            moves_limit = int(m.group(1))
        for enum_name in ("RobotState", "Steps"):
            if enum_name not in enums:
                found = find_enum(clean, text, enum_name)
                if found:
                    enums[enum_name] = found

    def rel(p):
        return os.path.relpath(p, display_root).replace("\\", "/")

    out = ["// Generated by sim/tools/sketch.py from the .ino files in " + rel(sketch_dir) + ". Do not edit.\n",
           "#include <Arduino.h>\n"]
    for ci, (path, text) in enumerate(chunks):
        lines = text.splitlines(keepends=True)
        if first_def is not None and first_def[0] == ci:
            k = first_def[1]
            out.append('#line 1 "%s"\n' % rel(path))
            out.extend(lines[:k])
            out.append("// ---- prototypes generated by the simulator (the Arduino IDE does the same) ----\n")
            for p in prototypes:
                out.append(p + "\n")
            out.append('#line %d "%s"\n' % (k + 1, rel(path)))
            out.extend(lines[k:])
        else:
            out.append('#line 1 "%s"\n' % rel(path))
            out.extend(lines)
    out.append('#line 1 "sim_probe.inc"\n')
    out.append('#include "%s"\n' % probe_path.replace("\\", "/"))

    os.makedirs(os.path.dirname(out_cpp), exist_ok=True)
    with open(out_cpp, "w", encoding="utf-8") as fh:
        fh.write("".join(out))

    with open(state_names_header, "w", encoding="utf-8") as fh:
        fh.write("// Generated by sim/tools/sketch.py: enum names found in the sketch.\n")
        for enum_name in ("RobotState", "Steps"):
            names = enums.get(enum_name, [])
            fh.write("#define SIM_HAVE_%s %d\n" % (enum_name.upper(), 1 if names else 0))
            fh.write("static const char* const SIM_%s_NAMES[] = {%s};\n" % (
                enum_name.upper(), ", ".join('"%s"' % n for n in names) or '"?"'))
            fh.write("static const int SIM_%s_COUNT = %d;\n" % (enum_name.upper(), len(names)))
        fh.write("static void (*const SIM_VOID_FNS[])() = {%s};\n" % (", ".join(void_fns) or "nullptr"))
        fh.write("static const char* const SIM_VOID_FN_NAMES[] = {%s};\n" % (
            ", ".join('"%s"' % n for n in void_fns) or '"?"'))
        fh.write("static const int SIM_VOID_FN_COUNT = %d;\n" % len(void_fns))
        fh.write("#define SIM_CODE_MOVES_LIMIT %d\n" % moves_limit)
    return {"ino": files, "prototypes": prototypes, "enums": enums, "void_fns": void_fns}


if __name__ == "__main__":
    import sys
    info = merge_sketch(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    print("merged", info["ino"], "with", len(info["prototypes"]), "prototypes")
