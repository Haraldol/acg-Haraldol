#version 120

// see the GLSL 1.2 specification:
// https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.1.20.pdf

uniform bool is_reflection; // variable of the program
varying vec3 normal; // normal vector pass to the rasterizer and fragment shader

void main()
{
    normal = vec3(gl_Normal);// set normal and pass it to fragment shader

    // "gl_Vertex" is the *input* vertex coordinate of triangle.
    // "gl_Vertex" has type of "vec4", which is homogeneious coordinate
    float x0 = gl_Vertex.x/gl_Vertex.w;// x-coord
    float y0 = gl_Vertex.y/gl_Vertex.w;// y-coord
    float z0 = gl_Vertex.z/gl_Vertex.w;// z-coord
    if (is_reflection) {
        vec3 nrm = normalize(vec3(0.4, 0.0, 1.0)); // normal of the mirror
        vec3 org = vec3(-0.3, 0.0, -0.5); // point on the mirror
        // wite code to change the input position (x0,y0,z0).
        // the transformed position (x0, y0, z0) should be drawn as the mirror reflection.
        //
        // make sure the occlusion is correctly computed.
        // the mirror is behind the armadillo, so the reflected image should be behind the armadillo.
        // furthermore, make sure the occlusion is correctly computed for the reflected image.
        // Here we calculate the reflection of a point in a plane by doing the following:
        // searching for the scale value t = d-e/|n| 
        // then 
        float scalar_nrm_org   = nrm[0]*org[0] + nrm[1]*org[1] + nrm[2]*org[2];
        float scalar_nrm_point = nrm[0]*x0 + nrm[1]*y0 + nrm[2]*z0;
        float scalar_nrm_nrm   = nrm[0]*nrm[0] + nrm[1]*nrm[1] + nrm[2]*nrm[2];
        float distance = scalar_nrm_org - scalar_nrm_point;
        float scale_factor_t   = distance/scalar_nrm_nrm; // Distance divided by normal
        x0 = (x0 + 2*scale_factor_t*nrm[0]);
        y0 = (y0 + 2*scale_factor_t*nrm[1]);
        // The z0 coordiantes scale factor is not multiplied by 2, because we want to make it appear as close to the mirror as possible
        // Otherwise some parts of the armadillo renders outside the bounding box.
        z0 = (z0 + 1*scale_factor_t*nrm[2]); 
    }
    // do not edit below

    // "gl_Position" is the *output* vertex coordinate in the
    // "canonical view volume (i.e.. [-1,+1]^3)" pass to the rasterizer.
    gl_Position = vec4(x0, y0, -z0, 1);// opengl actually draw a pixel with *maximum* depth. so invert z
}
