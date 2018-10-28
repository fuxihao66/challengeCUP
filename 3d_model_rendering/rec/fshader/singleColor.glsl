#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform vec4 ver_color;
varying vec3 v_normal;
//! [0]
void main()
{
	vec3 lightDir = vec3(0.58, 0.58, 0.58);
	//vec3 diffuseColor = = ver_color.rgb * clamp(dot(v_normal, lightDir), 0, 1);
	vec3 diffuseColor = = v_normal;
    gl_FragColor = vec4(diffuseColor, 1.0);
}
//! [0]


