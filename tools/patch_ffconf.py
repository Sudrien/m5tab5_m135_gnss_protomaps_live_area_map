# Patch a vendored copy of IDF's FatFs ffconf.h to enable exFAT.
#
# Run from CMakeLists.txt, not by hand - see the MAP_FATFS_EXFAT block there.
# It lives in a file rather than inline in CMake because the second half is a
# per-line regex rewrite, and CMake's regex has no multiline mode: doing it
# there means matching newlines by hand and running the substitution twice to
# catch adjacent lines, which is the kind of clever that breaks quietly.
#
# Usage: patch_ffconf.py <path to ffconf.h>
# Exits non-zero with a message on any doubt. A patch that half-applied is
# worse than one that did not: the symptom is exFAT volumes still refusing to
# mount, hours later, with nothing pointing back here.
#
# Ported from tools/enable_exfat.sh in m5tab5_esp_idf_usb_host_example.

import re
import sys

if len(sys.argv) != 2:
    sys.exit("usage: patch_ffconf.py <ffconf.h>")

path = sys.argv[1]

try:
    text = open(path).read()
except OSError as e:
    sys.exit("cannot read %s: %s" % (path, e))


def set_ff(text, name, want):
    """Set an FF_* option to a literal value, then prove it took.

    A regex that matches nothing is not an error to re.sub, so without the
    check afterwards a failed patch looks exactly like a successful one.
    """
    pat = re.compile(r'^#define[ \t]+%s[ \t].*$' % name, re.MULTILINE)
    if not pat.search(text):
        sys.exit("no %s line in %s - IDF's FatFs layout may have changed"
                 % (name, path))
    text = pat.sub('#define %s\t%s' % (name, want), text)

    check = re.search(r'^#define[ \t]+%s[ \t]+(\S+)' % name, text, re.MULTILINE)
    if not check or check.group(1) != want:
        sys.exit("%s is %r after patching, expected %r"
                 % (name, check.group(1) if check else None, want))
    return text


# exFAT itself.
text = set_ff(text, 'FF_FS_EXFAT', '1')

# exFAT volumes routinely exceed 32-bit sector addressing. This is also what
# brings GPT partition tables in: FatFs reads them in find_volume(), inside
# #if FF_LBA64, so a GPT drive is unreadable without it regardless of the
# filesystem on the partition.
text = set_ff(text, 'FF_LBA64', '1')

# Neither the SD nor the MSC disk IO layer implements TRIM, and FatFs calls it
# during exFAT operations. Left on, it surfaces as ESP_ERR_INVALID_RESPONSE.
text = set_ff(text, 'FF_USE_TRIM', '0')

# ffconf.h maps most FF_* options straight onto Kconfig symbols:
#
#     #define FF_USE_LABEL    CONFIG_FATFS_USE_LABEL
#
# Kconfig emits nothing at all for a bool that is off - not 0, nothing - so
# FF_USE_LABEL expands to a bare undeclared identifier. Stock IDF never trips
# over it, because the only uses sit in code that compiles solely when exFAT is
# on. Turning exFAT on is exactly what exposes them:
#
#     ff.c: In function 'dir_read':
#     ffconf.h:55: error: 'CONFIG_FATFS_USE_LABEL' undeclared
#
# Every option defined this way carries the same hazard, so guard them all
# rather than waiting for each new exFAT code path to light one up. Behaviour
# is unchanged: set stays set, unset becomes the 0 it always meant.
pattern = re.compile(
    r'^#define[ \t]+(FF_[A-Z0-9_]+)[ \t]+(CONFIG_[A-Z0-9_]+)[ \t]*$',
    re.MULTILINE)

guarded = []


def guard(m):
    ff, cfg = m.group(1), m.group(2)
    guarded.append(ff)
    return ('#ifdef %s\n#define %s\t%s\n#else\n#define %s\t0\n#endif'
            % (cfg, ff, cfg, ff))


text = pattern.sub(guard, text)

open(path, 'w').write(text)

print("ffconf: FF_FS_EXFAT=1 FF_LBA64=1 FF_USE_TRIM=0")
if guarded:
    print("ffconf: guarded %d Kconfig bools: %s"
          % (len(guarded), ", ".join(guarded)))
