/****************************************************************
 * $ID: colorbar.c     Sun, 23 Dec 2007 21:01:11 +0800  mhfan $ *
 *                                                              *
 * Description:                                                 *
 *                                                              *
 * Maintainer:  ∑∂√¿ª‘(MeiHui FAN)  <mhfan@hhcn.com>            *
 *                                                              *
 * Copyright (C)  2007~2008  HHTech                             *
 *   www.hhcn.com, www.hhcn.org                                 *
 *   All rights reserved.                                       *
 *                                                              *
 * This file is free software;                                  *
 *   you are free to modify and/or redistribute it   	        *
 *   under the terms of the GNU General Public Licence (GPL).   *
 ****************************************************************/


static inline void memset32(uint32_t* ptr, uint32_t val, int32_t siz)
{
    for (siz >>= 4; 0 < siz--; ptr += 4) {
	ptr[0] = val;	ptr[1] = val;	ptr[2] = val;	ptr[3] = val;
    }
}


#define	UYVY_WHITE		(0xEB80EB80)
#define	UYVY_YELLOW		(0xD292D210)
#define	UYVY_CYAN		(0xAA10AAA6)
#define	UYVY_GREEN		(0x91229136)
#define	UYVY_MAGENTA		(0x6ADE6ACA)
#define	UYVY_RED		(0x51F0515A)
#define	UYVY_BLUE		(0x296E29F0)
#define	UYVY_BLACK		(0x10801080)


#ifdef	CONFIG_ITU656_OUTPUT
#ifdef	NTSC
#define	FIELD1_VBL_START	(1)
#define	FIELD1_VBL_FINISH	(20)
#define	FIELD2_VBL_START	(264)
#define	FIELD2_VBL_FINISH	(283)
#define	FIELD1_START_LINE	(4)
#define	FIELD2_START_LINE	(266)

#define	LINES_PER_VBL		(20)	// XXX: 19.5
#define	LINES_PER_FIELD		(243)
#define	LINES_PER_FRAME		(525)

#define	LINE_BLANK_BYTES	(268)
#define	FRAMES_PER_SECOND	(30)

#elif	defined(PAL)

#define	FIELD1_VBL_START	(624)
#define	FIELD1_VBL_FINISH	(23)
#define	FIELD2_VBL_START	(311)
#define	FIELD2_VBL_FINISH	(336)
#define	FIELD1_START_LINE	(1)
#define	FIELD2_START_LINE	(313)

#define	LINES_PER_VBL		(24)	// XXX: 24.5
#define	LINES_PER_FIELD		(288)
#define	LINES_PER_FRAME		(625)

#define	LINE_BLANK_BYTES	(280)
#define	FRAMES_PER_SECOND	(25)

#else
#error  "Please define `NTSC' or `PAL'!"
#endif//NTSC

#ifdef	ACTIVE_VIDEO_ONLY
#undef	LINES_PER_FRAME
#undef	FIELD1_VBL_START
#undef	FIELD2_VBL_START
#undef	FIELD1_VBL_FINISH
#undef	FIELD2_VBL_FINISH
#undef	FIELD1_START_LINE
#undef	FIELD2_START_LINE
#undef	LINE_BLANK_BYTES
#undef	LINES_PER_VBL

#define	FIELD1_VBL_START	(0)
#define	FIELD2_VBL_START	(0)
#define	FIELD1_VBL_FINISH	(0)
#define	FIELD2_VBL_FINISH	(0)
#define	FIELD1_START_LINE	(1)
#define	FIELD2_START_LINE	(289)
#define	LINE_BLANK_BYTES	(0)
#define	LINES_PER_VBL		(0)

#define	LINES_PER_FRAME		(FRAME_ACTIVE_LINES)
#define	SAV_BYTES		(0)
#define	EAV_BYTES		(0)

#else// XXX:
#define	SAV_BYTES		(4)
#define	EAV_BYTES		(4)
#endif//ACTIVE_VIDEO_ONLY

#define	BITS_PER_PIXEL		(16)
#define	PIXELS_PER_LINE		(720)

#define	FRAME_ACTIVE_LINES	(LINES_PER_FIELD << 1)
#define	LINE_ACTIVE_BYTES	(PIXELS_PER_LINE << 1)
#define	BYTES_PER_PIXEL		((BITS_PER_PIXEL + 7) >> 3)
#define	LINE_CONTROL_BYTES	(SAV_BYTES + LINE_BLANK_BYTES + EAV_BYTES)
#define	BYTES_PER_LINE		(LINE_CONTROL_BYTES + (PIXELS_PER_LINE << 1))
#define BYTES_PER_FIELD		(BYTES_PER_LINE * LINES_PER_FIELD)
#define BYTES_PER_FRAME		(BYTES_PER_LINE * LINES_PER_FRAME)

static inline unsigned* itu656_line(unsigned* ptr, int init)
{
/*
 * Offsets for ITU656/CCIR601 frame line definition:
 *
 * |<--EAV-->|<----BLANKING---->|<--SAV-->|<--------ACTIVE FIELD-------->|
 *      4          268/280           4                  1440
 */
    short i;
#ifdef	ACTIVE_VIDEO_ONLY
    short V = 0;
#else
    static short L = 0, V = 1, F = 1;

    switch (++L) {
    // F-digital field identification:
    case FIELD1_START_LINE : F = 0; break;
    case FIELD2_START_LINE : F = 1; break;
    // V-digital field blanking:
    case FIELD1_VBL_START  : V = 1; break;
    case FIELD1_VBL_FINISH : V = 0; break;
    case FIELD2_VBL_START  : V = 1; break;
    case FIELD2_VBL_FINISH : V = 0; break;
    // finished a frame:
    case LINES_PER_FRAME   : L = 0; break;
    }

    // End Active Video (EAV):
    *ptr++ = F ? (V ? 0xF10000FF : 0xDA0000FF) :
		 (V ? 0xB60000FF : 0x9D0000FF);

    // Blanking Code:
    if (init) for (i = (LINE_BLANK_BYTES >> 2) + 1; --i; )
	*ptr++ = UYVY_BLACK;
    else ptr += (LINE_BLANK_BYTES >> 2);

    // Start Active Video (SAV):
    *ptr++ = F ? (V ? 0xEC0000FF : 0xC70000FF) :
		 (V ? 0xAB0000FF : 0x800000FF);
#endif//ACTIVE_VIDEO_ONLY

    if (init) {
	if (V) {	// Video Blanking Line:
	    for (i = (LINE_ACTIVE_BYTES >> 2) + 1; --i; )
		*ptr++ = UYVY_BLACK;
	} else {	// Active Video Line Code:
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_WHITE;
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_YELLOW;
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_CYAN;
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_GREEN;
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_MAGENTA;
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_RED;
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_BLUE;
	    for (i = ((LINE_ACTIVE_BYTES >> 2) >> 3) + 1; --i; )
		*ptr++ = UYVY_BLACK;
	}
    } else ptr += (LINE_ACTIVE_BYTES >> 2);

    return ptr;
}
#endif//CONFIG_ITU656_OUTPUT

#ifdef	CONFIG_FB_COLORBAR
void __init draw_colorbar(struct fb_info* fbi)
{
    u16 s, r, w, h;
    u32* pixels = (u32*)fbi->screen_base;
    struct fb_var_screeninfo* var = &fbi->var;

#define HORIZON_LINES		(2)
#define CURSOR_PIXELS		(32)

    if (var->nonstd/* == FB_PIXEL_YUV*/) {
	for (s = (var->xres >> 1) + 1, h = HORIZON_LINES + 1; --h; )
	    for (w = s; --w; )	*pixels++ = UYVY_BLUE;	// margin

	s = (var->xres >> 4) + 1, r = ((var->xres - ((s - 1) << 4)) >> 1);
	for (h = var->yres - (HORIZON_LINES << 1) + 1; --h; ) {
	    for (w = s; --w; )	*pixels++ = UYVY_RED;
	    for (w = s; --w; )	*pixels++ = UYVY_GREEN;
	    for (w = s; --w; )	*pixels++ = UYVY_BLUE;
	    for (w = s; --w; )	*pixels++ = UYVY_BLACK;
	    for (w = s; --w; )	*pixels++ = UYVY_WHITE;
	    for (w = s; --w; )	*pixels++ = UYVY_YELLOW;
	    for (w = s; --w; )	*pixels++ = UYVY_MAGENTA;
	    for (w = s; --w; )	*pixels++ = UYVY_CYAN;
				 pixels += r;
	}

	for (s = (var->xres >> 1) + 1, h = HORIZON_LINES + 1; --h; )
	    for (w = s; --w; )	*pixels++ = UYVY_YELLOW;// margin
    }	else

    if (var->bits_per_pixel <  9) /*dtrace*/; else
    if (var->bits_per_pixel < 17) {
	for (s = (var->xres >> 1) + 1, h = HORIZON_LINES + 1; --h; )
	    for (w = s; --w; )	*pixels++ = 0x001f001f;		// Blue margin

	s = (var->xres >> 4) + 1, r = ((var->xres - ((s - 1) << 4)) >> 1);
	for (h = var->yres - (HORIZON_LINES << 1) + 1; --h; ) {
	    for (w = s; --w; )	*pixels++ = 0xf800f800;		// Red
	    for (w = s; --w; )	*pixels++ = 0x07e007e0;		// Green
	    for (w = s; --w; )	*pixels++ = 0x001f001f;		// Blue
	    for (w = s; --w; )	*pixels++ = 0x00000000;		// Black
	    for (w = s; --w; )	*pixels++ = 0xffffffff;		// White
	    for (w = s; --w; )	*pixels++ = 0xffe0ffe0;		// R + G
	    for (w = s; --w; )	*pixels++ = 0x07ff07ff;		// G + B
	    for (w = s; --w; )	*pixels++ = 0xf81ff81f;		// B + R
				 pixels += r;
	}

	for (s = (var->xres >> 1) + 1, h = HORIZON_LINES + 1; --h; )
	    for (w = s; --w; )	*pixels++ = 0xffe0ffe0;		// Yellow
    }	else

    if (var->bits_per_pixel < 25) {
	for (s = (var->xres >> 2) + 1, h = HORIZON_LINES + 1; --h; ) {
	    for (w = s; --w; )		*pixels++ = 0x00ff0000,	// Blue margin
		*pixels++ = 0x0000ff00, *pixels++ = 0xff0000ff;
#ifndef	CONFIG_DELTA_LCD
continue;
#endif
					--h;			// even line:
	    for (w = s; --w; )		*pixels++ = 0x0000ff00,	// Green
		*pixels++ = 0xff0000ff, *pixels++ = 0x00ff0000;
	}

	s =   (var->xres >> 5) + 1;
	r = (((var->xres - ((s - 1) << 5)) * 3) >> 2);
	for (h = var->yres - (HORIZON_LINES << 1) + 1; --h; ) {
#if 0
	    unsigned t = 0xff - h;
	    for (w = s; --w; )		    *pixels++ = 0x01000001 * t,
		*pixels++ = 0x00010000 * t, *pixels++ = 0x00000100 * t;
	    for (w = s; --w; )		    *pixels++ = 0x00000100 * t,
		*pixels++ = 0x01000001 * t, *pixels++ = 0x00010000 * t;
	    for (w = s; --w; )		    *pixels++ = 0x00010000 * t,
		*pixels++ = 0x00000100 * t, *pixels++ = 0x01000001 * t;
	    for (w = s; --w; )		    *pixels++ = 0x00000000 * t,
		*pixels++ = 0x00000000 * t, *pixels++ = 0x00000000 * t;
	    for (w = s; --w; )		    *pixels++ = 0x01010101 * t,
		*pixels++ = 0x01010101 * t, *pixels++ = 0x01010101 * t;
	    for (w = s; --w; )		    *pixels++ = 0x01000101 * t,
		*pixels++ = 0x01010001 * t, *pixels++ = 0x00010100 * t;
	    for (w = s; --w; )		    *pixels++ = 0x00010100 * t,
		*pixels++ = 0x01000101 * t, *pixels++ = 0x01010001 * t;
	    for (w = s; --w; )		    *pixels++ = 0x01010001 * t,
		*pixels++ = 0x00010100 * t, *pixels++ = 0x01000101 * t;
#else
	    for (w = s; --w; )		*pixels++ = 0xff0000ff,	// Red
		*pixels++ = 0x00ff0000, *pixels++ = 0x0000ff00;
	    for (w = s; --w; )		*pixels++ = 0x0000ff00,	// Green
		*pixels++ = 0xff0000ff, *pixels++ = 0x00ff0000;
	    for (w = s; --w; )		*pixels++ = 0x00ff0000,	// Blue
		*pixels++ = 0x0000ff00, *pixels++ = 0xff0000ff;
	    for (w = s; --w; )		*pixels++ = 0x00000000,	// Black
		*pixels++ = 0x00000000, *pixels++ = 0x00000000;
	    for (w = s; --w; )		*pixels++ = 0xffffffff,	// White
		*pixels++ = 0xffffffff, *pixels++ = 0xffffffff;
	    for (w = s; --w; )		*pixels++ = 0xff00ffff,	// R + G
		*pixels++ = 0xffff00ff, *pixels++ = 0x00ffff00;
	    for (w = s; --w; )		*pixels++ = 0x00ffff00,	// G + B
		*pixels++ = 0xff00ffff, *pixels++ = 0xffff00ff;
	    for (w = s; --w; )		*pixels++ = 0xffff00ff,	// B + R
		*pixels++ = 0x00ffff00, *pixels++ = 0xff00ffff;
#endif
					 pixels += r;
#ifndef	CONFIG_DELTA_LCD
continue;	// The colorbar sequence is according the first line.
#endif
					--h;			// even line:

#if 0
	    t = 0xff - h;
	    for (w = s; --w; )		    *pixels++ = 0x00010000 * t,
		*pixels++ = 0x00000100 * t, *pixels++ = 0x01000001 * t;
	    for (w = s; --w; )		    *pixels++ = 0x01000001 * t,
		*pixels++ = 0x00010000 * t, *pixels++ = 0x00000100 * t;
	    for (w = s; --w; )		    *pixels++ = 0x00000100 * t,
		*pixels++ = 0x01000001 * t, *pixels++ = 0x00010000 * t;
	    for (w = s; --w; )		    *pixels++ = 0x00000000 * t,
		*pixels++ = 0x00000000 * t, *pixels++ = 0x00000000 * t;
	    for (w = s; --w; )		    *pixels++ = 0x01010101 * t,
		*pixels++ = 0x01010101 * t, *pixels++ = 0x01010101 * t;
	    for (w = s; --w; )		    *pixels++ = 0x01010001 * t,
		*pixels++ = 0x00010100 * t, *pixels++ = 0x01000101 * t;
	    for (w = s; --w; )		    *pixels++ = 0x01000101 * t,
		*pixels++ = 0x01010001 * t, *pixels++ = 0x00010100 * t;
	    for (w = s; --w; )		    *pixels++ = 0x00010100 * t,
		*pixels++ = 0x01000101 * t, *pixels++ = 0x01010001 * t;
#else
	    for (w = s; --w; )		*pixels++ = 0x00ff0000,	// Blue
		*pixels++ = 0x0000ff00, *pixels++ = 0xff0000ff;
	    for (w = s; --w; )		*pixels++ = 0xff0000ff,	// Red
		*pixels++ = 0x00ff0000, *pixels++ = 0x0000ff00;
	    for (w = s; --w; )		*pixels++ = 0x0000ff00,	// Green
		*pixels++ = 0xff0000ff, *pixels++ = 0x00ff0000;
	    for (w = s; --w; )		*pixels++ = 0x00000000,	// Black
		*pixels++ = 0x00000000, *pixels++ = 0x00000000;
	    for (w = s; --w; )		*pixels++ = 0xffffffff,	// White
		*pixels++ = 0xffffffff, *pixels++ = 0xffffffff;
	    for (w = s; --w; )		*pixels++ = 0xffff00ff,	// B + R
		*pixels++ = 0x00ffff00, *pixels++ = 0xff00ffff;
	    for (w = s; --w; )		*pixels++ = 0xff00ffff,	// R + G
		*pixels++ = 0xffff00ff, *pixels++ = 0x00ffff00;
	    for (w = s; --w; )		*pixels++ = 0x00ffff00,	// G + B
		*pixels++ = 0xff00ffff, *pixels++ = 0xffff00ff;
#endif
					 pixels += r;
	}

	for (s = (var->xres >> 2) + 1, h = HORIZON_LINES + 1; --h; ) {
	    for (w = s; --w; )		*pixels++ = 0xff00ffff,	// Yellow
		*pixels++ = 0xffff00ff, *pixels++ = 0x00ffff00;
#ifndef	CONFIG_DELTA_LCD
continue;
#endif
					--h;			// even line:
	    for (w = s; --w; )		*pixels++ = 0xffff00ff,	// B + R
		*pixels++ = 0x00ffff00, *pixels++ = 0xff00ffff;
	}

#ifdef	CONFIG_DELTA_LCD
    /* 
     * For Delta LCD(e.g.: A025DL01), the RGB sequence is:
     *	R-G-B for  odd line, and G-B-R for even line.
     *
     * For best visual effect, the recommented RGB sequence is:
     *  G-B-R for  odd line, and B-R-G for even line.
     */
    pixels = (u32*)fbi->screen_base;
    for (s = (var->xres * 3) >> 2, h = var->yres + 1; --h; ) {	u32 lva;
#if 0
	for (w = s, lva = *pixels; --w; ) {
	    u32 nva = pixels[1];    // shift the line data  left:
	    *pixels++ = (lva >>  8) | (nva << 24);	lva = nva;
	}   *pixels++ =  lva >>  8;
#else
	for (w = s + 1,  lva = 0; --w; ) {
	    u32 nva =*pixels;	    // shift the line data right:
	    *pixels++ = (nva <<  8) | (lva >> 24);	lva = nva;
	}
#endif
    }
#endif//CONFIG_DELTA_LCD

#if 0
{   unsigned char* ptr = (unsigned char*)fbi->screen_base +
	((var->xres >> 1) + 10) * 3;
    for (h = var->yres + 1; --h; ) {
	if (h % 2)  ptr[2] = 0xff, ptr[1] = 0x00, ptr[3] = 0x00;
	else	    ptr[2] = 0x00, ptr[1] = 0x00, ptr[3] = 0xff;
	ptr[37] = ptr[38] = ptr[39] = 0x00;
	ptr += var->xres * 3;
    }
}
#endif
    } else if (var->bits_per_pixel < 33) {
	for (s = (var->xres >> 0) + 1, h = HORIZON_LINES + 1; --h; )
	    for (w = s; --w; )	*pixels++ = 0x000000ff;		// Blue margin

	s = (var->xres >> 3) + 1, r = ((var->xres - ((s - 1) << 3)) >> 0);
	for (h = var->yres - (HORIZON_LINES << 1) + 1; --h; ) {
	    for (w = s; --w; )	*pixels++ = 0x00ff0000;		// Red
	    for (w = s; --w; )	*pixels++ = 0x0000ff00;		// Green
	    for (w = s; --w; )	*pixels++ = 0x000000ff;		// Blue
	    for (w = s; --w; )	*pixels++ = 0x00000000;		// Black
	    for (w = s; --w; )	*pixels++ = 0xffffffff;		// White
	    for (w = s; --w; )	*pixels++ = 0x00ffff00;		// G + R
	    for (w = s; --w; )	*pixels++ = 0x0000ffff;		// B + G
	    for (w = s; --w; )	*pixels++ = 0x00ff00ff;		// R + B
				 pixels += r;
	}

	for (s = (var->xres >> 0) + 1, h = HORIZON_LINES + 1; --h; )
	    for (w = s; --w; )	*pixels++ = 0x00ffff00;		// Yellow
    } else {
	unsigned size = fbi->var.yres * fbi->fix.line_length / 2;

	memset32(pixels, 0x00000000, size);	// Black/
	pixels = (void*)((unsigned char*)pixels + size);

	memset32(pixels, 0xffffffff, size);	// White/
	pixels = (void*)((unsigned char*)pixels + size);

	//dtrace;
    }

    BUG_ON((char*)pixels != fbi->screen_base +
	    fbi->var.yres * fbi->fix.line_length);

    // draw a white cursor on top-left corner
    pixels = (u32*)fbi->screen_base;
    for (s = CURSOR_PIXELS; --s; ) {
	memset32(pixels, 0xffffffff, CURSOR_PIXELS);
	pixels = (u32*)((unsigned char*)pixels + fbi->fix.line_length);
    }
}
#else
#define draw_colorbar(...)
#endif//CONFIG_FB_COLORBAR

// vim:sts=4:ts=8:
