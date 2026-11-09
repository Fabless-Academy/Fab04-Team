#pragma once
#include "nc_opengl_interface.h"
void nc_opengl_draw_text(Font *font, const char *text, float x, float y, float scale, float color[3], int sw, int sh, struct gl_font_program p);
