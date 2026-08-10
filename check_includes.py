#!/usr/bin/env python3
"""Pre-flight: symbols used without a header that plausibly declares them.

Deliberately has no blanket exemptions - an earlier version treated
M5Unified.h as covering everything, which meant it silently passed the very
bug it was written to catch.
"""
import re, sys, os

NEEDS = {
    r'\bWiFi\.':                  'WiFi.h',
    r'\bWL_CONNECTED\b':          'WiFi.h',
    r'\bHTTPClient\b':            'HTTPClient.h',
    r'\bM5\.':                    'M5Unified.h',
    r'\bSD_MMC\b':                'SD_MMC.h',
    r'\bps_malloc\b':             'Arduino.h',
    r'\besp_timer_get_time\b':    'esp_timer.h',
    r'\bxTaskCreatePinnedToCore\b':'task.h',
    r'\bxQueue\w+\b':             'queue.h',
    r'\bxSemaphore\w+\b':         'semphr.h',
    r'\bconfigTime\b':            'Arduino.h',
    r'\bgmtime_r\b':              'time.h',
    r'\bsntp_get_sync_status\b':  'esp_sntp.h',
    r'\bmbedtls_\w+':             'mbedtls',
    r'\besp_fill_random\b':       'esp_random.h',
    r'\besp_efuse_mac_get_default\b': 'esp_mac.h',
    r'\bWebServer\b':             'WebServer.h',
    r'\bDNSServer\b':             'DNSServer.h',
}

# Running this bare used to check nothing at all and exit 0 - a green light
# that meant "no files were examined". Default to the whole directory so that
# cannot happen again.
files = sys.argv[1:]
if not files:
    here = os.path.dirname(os.path.abspath(__file__))
    files = sorted(os.path.join(here, n) for n in os.listdir(here)
                   if n.endswith(('.c', '.cpp', '.h', '.ino')))
    print(f'no files given; checking all {len(files)} sources in {here}\n')

bad = 0
for f in files:
    src = open(f).read()
    body = re.sub(r'//.*', '', src)
    body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)
    body = re.sub(r'#include[^\n]*', '', body)
    incs = ' '.join(re.findall(r'#include\s*[<"]([^>"]+)[>"]', src))

    miss = []
    for pat, hdr in NEEDS.items():
        if re.search(pat, body) and hdr not in incs:
            miss.append(f'{pat.strip(chr(92)+"b")} needs {hdr}')
    if miss:
        bad += 1
        print(f'{os.path.basename(f):18s} {"; ".join(miss)}')
    else:
        print(f'{os.path.basename(f):18s} ok')
sys.exit(1 if bad else 0)
