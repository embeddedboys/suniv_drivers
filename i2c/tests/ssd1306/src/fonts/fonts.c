/*
 * `Soft' font definitions
 *
 *    Created 1995 by Geert Uytterhoeven
 *    Rewritten 1998 by Martin Mares <mj@ucw.cz>
 *
 *	2001 - Documented with DocBook
 *	- Brad Douglas <brad@neruo.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <string.h>
#include <stddef.h>
#include "ssd1306_font.h"

#define ARRAY_SIZE(n) (sizeof(n)/sizeof(n[0]))

static const struct font_desc *fonts[] = {
    &font_vga_8x8,
    &font_vga_8x16,
    &font_vga_6x11,
    &font_7x14,
    &font_sun_8x16,
    &font_sun_12x22,
    &font_10x18,
    &font_acorn_8x8,
    &font_pearl_8x8,
    &font_mini_4x6,
    &font_6x10,
};

#define num_fonts ARRAY_SIZE(fonts)

/**
 *	find_font - find a font
 *	@name: string name of a font
 *
 *	Find a specified font with string name @name.
 *
 *	Returns %NULL if no font found, or a pointer to the
 *	specified font.
 *
 */

const struct font_desc *find_font(const char *name)
{
   unsigned int i;

   for (i = 0; i < num_fonts; i++)
      if (!strcmp(fonts[i]->name, name))
	  return fonts[i];
   return NULL;
}


/**
 *	get_default_font - get default font
 *	@xres: screen size of X
 *	@yres: screen size of Y
 *      @font_w: bit array of supported widths (1 - 32)
 *      @font_h: bit array of supported heights (1 - 32)
 *
 *	Get the default font for a specified screen size.
 *	Dimensions are in pixels.
 *
 *	Returns %NULL if no font is found, or a pointer to the
 *	chosen font.
 *
 */

const struct font_desc *get_default_font(int xres, int yres, unsigned font_w,
					 unsigned font_h)
{
    int i, c, cc;
    const struct font_desc *f, *g;

    g = NULL;
    cc = -10000;
    for(i=0; i<num_fonts; i++) {
	f = fonts[i];
	c = f->pref;
#if defined(__mc68000__)
#ifdef CONFIG_FONT_PEARL_8x8
	if (MACH_IS_AMIGA && f->idx == PEARL8x8_IDX)
	    c = 100;
#endif
#ifdef CONFIG_FONT_6x11
	if (MACH_IS_MAC && xres < 640 && f->idx == VGA6x11_IDX)
	    c = 100;
#endif
#endif
	if ((yres < 400) == (f->height <= 8))
	    c += 1000;

	if ((font_w & (1 << (f->width - 1))) &&
	    (font_h & (1 << (f->height - 1))))
	    c += 1000;

	if (c > cc) {
	    cc = c;
	    g = f;
	}
    }
    return g;
}
