#pragma once
struct viewport { int x; int y; int width; int height; };
struct gl_npu_program { int dummy; };
struct gl_font_program { int dummy; };
typedef struct { int dummy; } Font;
void glViewport(int x, int y, int w, int h);
void nc_opengl_draw_rectangle(float x, float y, float w, float h, float color[4], struct gl_npu_program p);
void nc_opengl_draw_line(float sx, float sy, float ex, float ey, int lane_class, float color[4], struct gl_npu_program p);
