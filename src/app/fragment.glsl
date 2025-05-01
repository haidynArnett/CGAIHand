precision highp float;              //// set default precision of float variables to high precision

varying vec2 vUv;                   //// screen uv coordinates (varying, from vertex shader)
uniform vec2 iResolution;           //// screen resolution (uniform, from CPU)
uniform float iTime;                //// time elapsed (uniform, from CPU)

const vec3 CAM_POS = vec3(-0.35, 1.0, -3.0);

float sdf2(vec3 p);
float sdfSmoothUnion(float a, float b, float k);
float sdfBlob(vec3 p, vec3 c, int num_x, int num_y, int numz);

#define PI 3.1415926538

/////////////////////////////////////////////////////
//// sdf functions
/////////////////////////////////////////////////////
//// sphere: p - query point; c - sphere center; r - sphere radius
float sdfSphere(vec3 p, vec3 c, float r)
{
    return length(p - c) - r;
}

//// plane: p - query point; h - height
float sdfPlane(vec3 p, float h)
{
    return p.y - h;
}

//// box: p - query point; c - box center; b - box half size (i.e., the box size is (2*b.x, 2*b.y, 2*b.z))
float sdfBox(vec3 p, vec3 c, vec3 b)
{
    vec3 d = abs(p-c) - b;
    return length(max(d, 0.0)) + min(max(d.x, max(d.y, d.z)), 0.0);
}

//// Vertical Capped Cylinder p - query point; h - height; r - radius from https://iquilezles.org/articles/distfunctions/
float sdfCappedCylinder( vec3 p, float h, float r )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - vec2(r,h);
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

// Octahedron - exact from https://iquilezles.org/articles/distfunctions/
float sdOctahedron( vec3 p, float s )
{
  p = abs(p);
  float m = p.x+p.y+p.z-s;
  vec3 q;
       if( 3.0*p.x < m ) q = p.xyz;
  else if( 3.0*p.y < m ) q = p.yzx;
  else if( 3.0*p.z < m ) q = p.zxy;
  else return m*0.57735027;
    
  float k = clamp(0.5*(q.z-q.y+s),0.0,s); 
  return length(vec3(q.x,q.y-s+k,q.z-k)); 
}

/////////////////////////////////////////////////////
//// operations
/////////////////////////////////////////////////////
float sdfIntersection(float s1, float s2)
{
    return max(s1, s2);
}

float sdfUnion(float s1, float s2)
{
    return min(s1, s2);
}

float sdfSubtraction(float s1, float s2)
{   
    return max(s1, -s2);
}

float sdfSmoothUnion(float a, float b, float k) {
    return -k * log2(exp2(-a/k) + exp2(-b/k));
}

vec3 translatePoint(vec3 p, vec3 t) {
    return p + t;
}

vec3 rotatePointX(vec3 p, float theta) {
    mat3 rot = mat3(1, 0, 0,
                    0, cos(theta), sin(theta),
                    0, -sin(theta), cos(theta));
    return rot * p;
}
vec3 rotatePointY(vec3 p, float theta) {
    mat3 rot = mat3(cos(theta), 0, -sin(theta),
                    0, 1, 0,
                    sin(theta), 0, cos(theta));
    return rot * p;
}
vec3 rotatePointZ(vec3 p, float theta) {
    mat3 rot = mat3(cos(theta), sin(theta), 0,
                    -sin(theta), cos(theta), 0, 
                    0, 0, 1);
    return rot * p;
}
vec3 rotatePointXYZ(vec3 p, vec3 c, float theta_x, float theta_y, float theta_z) {
    p = translatePoint(p, -c);
    p = rotatePointX(p, theta_x);
    p = rotatePointY(p, theta_y);
    p = rotatePointZ(p, theta_z);
    p = translatePoint(p, c);
    return p;
}

/////////////////////////////////////////////////////
//// sdf hand calculation
/////////////////////////////////////////////////////

// struct Joint {
//     Joint parent;
//     vec3 offset;
//     vec3 abs_pos;
//     vec3 rotation;
//     float s;
// }

// Joint make_joint(Joint parent, vec3 offset, vec3 rotation) {
//     Joint joint = Joint(parent, offset, vec3(0.), rotation, 0.);
//     joint.abs_pos = joint.offset;
//     if (joint.parent != NUll) {
//         joint.abs_pos += joint.parent.abs_pos;
//     }
// }

float smoothing_factor = 0.03;

// helpers to build hand
float base_knuckle_rad = 0.07;
float thumb(vec3 p, vec2 lengths, vec2 rots)
{
    float knuckle_rad = 0.01;
    float finger_rad = 0.06;

    vec3 p1 = rotatePointX(p, rots.x);
    float s = sdfSphere(p1, vec3(0.), base_knuckle_rad);
    s = sdfSmoothUnion(s, sdfCappedCylinder(p1 - vec3(0., lengths.x, 0.), lengths.x, finger_rad), smoothing_factor);

    vec3 p2 = rotatePointX(p1 - vec3(0., lengths.x * 2., 0.), rots.y);
    s = sdfSmoothUnion(s, sdfSphere(p2, vec3(0.), knuckle_rad), smoothing_factor);
    s = sdfSmoothUnion(s, sdfCappedCylinder(p2 - vec3(0., lengths.y, 0.), lengths.y, finger_rad), smoothing_factor);
    s = sdfSmoothUnion(s, sdfSphere(p2, vec3(0., lengths.y * 2., 0.), finger_rad), smoothing_factor);
    return s;
}

float finger(vec3 p, vec3 lengths, vec3 rots) {
    float knuckle_rad = 0.01;
    float finger_rad = 0.05;

    vec3 p1 = rotatePointX(p, rots.x);
    float s = sdfSphere(p1, vec3(0.), base_knuckle_rad);
    s = sdfSmoothUnion(s, sdfCappedCylinder(p1 - vec3(0., lengths.x, 0.), lengths.x, finger_rad), smoothing_factor);

    vec3 p2 = rotatePointX(p1 - vec3(0., lengths.x * 2., 0.), rots.y);
    s = sdfSmoothUnion(s, sdfSphere(p2, vec3(0.), knuckle_rad), smoothing_factor);
    s = sdfSmoothUnion(s, sdfCappedCylinder(p2 - vec3(0., lengths.y, 0.), lengths.y, finger_rad), smoothing_factor);

    vec3 p3 = rotatePointX(p2 - vec3(0., lengths.y * 2., 0.), rots.z);
    s = sdfSmoothUnion(s, sdfSphere(p3, vec3(0.), knuckle_rad), smoothing_factor);
    s = sdfSmoothUnion(s, sdfCappedCylinder(p3 - vec3(0., lengths.z, 0.), lengths.z, finger_rad), smoothing_factor);
    s = sdfSmoothUnion(s, sdfSphere(p3, vec3(0., lengths.z * 2., 0.), finger_rad), smoothing_factor);

    return s;
}

float sdf(vec3 p)
{
    p -= vec3(0., 1.0, 1.);
    p = rotatePointXYZ(p, vec3(0.), 0., 0., iTime);

    float s = 0.;

    // Thumb
    p = rotatePointZ(p, PI / 2.2);
    s = sdfUnion(s, thumb(
        rotatePointZ(p - vec3(0.0, 0.5, 0.), -PI / 3.),
        vec2(0.12, 0.09),
        vec2(0.1, 0.1)
    ));
    p = rotatePointZ(p, -PI / 2.2);

    // Fingers
    s = sdfUnion(s, finger(
        p - vec3(0.3, 0.3, 0.),
        vec3(0.15, 0.13, 0.1),
        vec3(0.1, 0.1, 0.1)
    ));

    s = sdfUnion(s, finger(
        p - vec3(0.1, 0.4, 0.),
        vec3(0.15, 0.13, 0.1),
        vec3(0.1, 0.1, 0.1)
    ));

    s = sdfUnion(s, finger(
        p - vec3(-0.1, 0.4, 0.),
        vec3(0.15, 0.13, 0.1),
        vec3(0.1, 0.1, 0.1)
    ));

    s = sdfUnion(s, finger(
        p - vec3(-0.3, 0.3, 0.),
        vec3(0.12, 0.11, 0.08),
        vec3(0.1, 0.1, 0.1)
    ));

    // Palm
    // p = rotatePointX(p, PI / 2.);
    // s = sdfUnion(s, sdfCappedCylinder(p, 0.06, 0.5));
    // p = rotatePointX(p, -PI / 2.);

    return s;
}

/////////////////////////////////////////////////////
//// ray marching
/////////////////////////////////////////////////////

//// ray marching: origin - ray origin; dir - ray direction 
float rayMarching(vec3 origin, vec3 dir)
{
    float s = 0.0;
    for(int i = 0; i < 100; i++)
    {
        vec3 p = origin + s * dir;
        float sdf = sdf(p);
        s += sdf;
        if (sdf < 0.0 || s > 20.0) {
            break;
        }
    }
    
    return s;
}

/////////////////////////////////////////////////////
//// normal calculation
/////////////////////////////////////////////////////

//// normal: p - query point
vec3 normal(vec3 p)
{
    float s = sdf(p);          //// sdf value in p
    float dx = 0.01;           //// step size for finite difference

    float nx = sdf(vec3(p.x + dx, p.y, p.z)) - s;
    float ny = sdf(vec3(p.x, p.y + dx, p.z)) - s;
    float nz = sdf(vec3(p.x, p.y, p.z + dx)) - s;
    return normalize(vec3(nx, ny, nz));
}

/////////////////////////////////////////////////////
//// Phong shading
/////////////////////////////////////////////////////

vec3 phong_shading(vec3 p, vec3 n)
{
    //// background
    if(p.z > 10.0){
        return vec3(0.9, 0.6, 0.2);
    }

    //// phong shading
    vec3 lightPos = vec3(4.*sin(iTime), 4., 4.*cos(iTime));  
    vec3 l = normalize(lightPos - p);               
    float amb = 0.1;
    float dif = max(dot(n, l), 0.) * 0.7;
    vec3 eye = CAM_POS;
    float spec = pow(max(dot(reflect(-l, n), normalize(eye - p)), 0.0), 128.0) * 0.9;

    vec3 sunDir = vec3(0, 1, -1);
    float sunDif = max(dot(n, sunDir), 0.) * 0.2;

    //// shadow
    float s = rayMarching(p + n * 0.02, l);
    if(s < length(lightPos - p)) dif *= .2;

    vec3 color = vec3(1., 1., 1.);

    return (amb + dif + spec + sunDif) * color;
}

/////////////////////////////////////////////////////
//// main function
/////////////////////////////////////////////////////

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    vec2 uv = (fragCoord.xy - .5 * iResolution.xy) / iResolution.y;         //// screen uv
    vec3 origin = CAM_POS;                                                  //// camera position 
    vec3 dir = normalize(vec3(uv.x, uv.y, 1));                              //// camera direction
    float s = rayMarching(origin, dir);                                     //// ray marching
    vec3 p = origin + dir * s;                                              //// ray-sdf intersection
    vec3 n = normal(p);                                                     //// sdf normal
    vec3 color = phong_shading(p, n);                                       //// phong shading
    fragColor = vec4(color, 1.);                                            //// fragment color
}

void main() 
{
    mainImage(gl_FragColor, gl_FragCoord.xy);
}