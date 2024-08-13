#pragma once


#pragma once

static const char* skeleton_vshader =
#ifndef __EMSCRIPTEN__
    "#version 330"
#else
    "#version 300 es"
#endif
R"glsl(
layout (location=0) in vec4 v_position;
layout (location=1) in vec3 v_color;

out vec3 v2f_color;

uniform mat4 modelview_projection_matrix;

void main()
{
    v2f_color = v_color;
    gl_Position  = modelview_projection_matrix * v_position;
}

)glsl";


static const char* skeleton_fshader =
#ifndef __EMSCRIPTEN__
    "#version 330\n"
#else
    "#version 300 es\n"
#endif
R"glsl(
precision mediump float;

in vec3 v2f_color;

out vec4 f_color;

void main()
{
    f_color = vec4(v2f_color, 1.0);
}

)glsl";
