#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;

attribute vec4 a_position;
attribute vec3 a_normal;
varying vec3 v_normal;
//! [0]
void main()
{
    gl_Position = mvp_matrix * a_position;
	v_normal = a_normal;
}
//! [0]
