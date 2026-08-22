// Compiled-in z0 world map.
//
// The boot screen used to get its backdrop by rendering z0/0/0 out of the
// pmtiles archive, which meant the backdrop could not appear until the card
// was mounted and the archive opened - two of the slowest steps of boot, and
// the two most likely to be the thing that has gone wrong. A boot screen that
// only has a picture once everything already works is a boot screen with no
// picture on the boots that matter.
//
// So the outline is in flash instead. Natural Earth 1:110m land and lakes,
// public domain, Mercator-projected and simplified to the pixel grid it is
// drawn on: about 15 KB of coordinates and no dependency on storage, PSRAM,
// the network or the archive. It is available from the first line of setup().
//
// This is deliberately not a replacement for the map engine. There are no
// roads, no names, no zoom and no fix marker - it is a backdrop, drawn once
// per repaint, and the real map paints over it as soon as it has tiles.
#pragma once

#include <stdint.h>

// Paint the world across the given rectangle of the panel.
//
// Scaled to cover: the world is drawn as a square of side max(w, h) centred
// on the rectangle, so the shorter axis is cropped rather than letterboxed.
// At 1280x720 that is the middle 720 rows of a 1280-wide world, which loses
// the poles - Mercator stretches them into meaninglessness and there is no
// coastline up there anybody recognises.
//
// `dark` picks the night palette, matching the map's own two-palette scheme
// so a night boot does not flash a bright screen at 3 am.
//
// Nothing is allocated: one row is composed at a time in a static buffer and
// pushed. Safe to call before the card is mounted, and safe to call when
// PSRAM is full.
void worldmap_draw(int x0, int y0, int w, int h, bool dark);

// The sea colour of the palette above, for callers that need to match it -
// clearing a strip, or filling the part of a panel the map has not reached.
uint16_t worldmap_sea(bool dark);
