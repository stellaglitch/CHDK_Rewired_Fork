#ifndef BEND_PREVIEW_H
#define BEND_PREVIEW_H

// Live preview of the bend matrix and the experimental chain - an
// approximation drawn over the live view. Implementation and its limits are
// in core/bend_preview.c.

// Whether there is anything to preview: the switch is on and the matrix or the
// chain would change a picture.
int  bend_preview_wanted(void);

// One step of the preview, from gui_redraw(). Alternates between building a
// frame and painting half of it; force paints the whole of the last frame at
// once, for a screen that has just been erased. Returns non-zero if it drew
// anything, so the caller knows to repaint the layers above it.
int  bend_preview_draw(int force);

// Called from gui_redraw() whenever the preview is not being drawn. Takes it
// off the screen if it was on - unless wiped says the screen has already been
// cleared, by a shot - and gives the heap back if it is switched off.
void bend_preview_idle(int wiped);

// Raw R, G, B of a daylight-white surface, green = 1000, from the port's DNG
// colour matrix; all zero if there is none. Shared with the capture path
// (raw.c), which hands it to the experimental engine.
void bend_preview_neutral(unsigned *out);

#endif
