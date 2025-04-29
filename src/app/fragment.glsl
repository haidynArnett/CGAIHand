precision highp float;              //// set default precision of float variables to high precision

varying vec2 vUv;                   //// screen uv coordinates (varying, from vertex shader)
uniform vec2 iResolution;           //// screen resolution (uniform, from CPU)
uniform float iTime;                //// time elapsed (uniform, from CPU)

const vec3 CAM_POS = vec3(-0.35, 1.0, -3.0);

vec2 sdf2(vec3 p);
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
//// boolean operations
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

/////////////////////////////////////////////////////
//// sdf calculation
/////////////////////////////////////////////////////

//// sdf: p - query point
vec2 sdf(vec3 p)
{
    float s = 0.;

    //// 1st object: plane
    float plane1_h = -0.1;
    
    //// 2nd object: sphere
    vec3 sphere1_c = vec3(-2.0, 1.0, 0.0);
    float sphere1_r = 0.25;

    //// 3rd object: box
    vec3 box1_c = vec3(-1.0, 1.0, 0.0);
    vec3 box1_b = vec3(0.2, 0.2, 0.2);

    //// 4th object: box-sphere subtraction
    vec3 box2_c = vec3(0.0, 1.0, 0.0);
    vec3 box2_b = vec3(0.3, 0.3, 0.3);

    vec3 sphere2_c = vec3(0.0, 1.0, 0.0);
    float sphere2_r = 0.4;

    //// 5th object: sphere-sphere intersection
    vec3 sphere3_c = vec3(1.0, 1.0, 0.0);
    float sphere3_r = 0.4;

    vec3 sphere4_c = vec3(1.3, 1.0, 0.0);
    float sphere4_r = 0.3;

    //// calculate the sdf based on all objects in the scene
    
    float plane1_sdf = sdfPlane(p, plane1_h);
    float sphere1_sdf = sdfSphere(p, sphere1_c, sphere1_r);
    float box1_sdf = sdfBox(p, box1_c, box1_b);
    float box_sphere_sdf = sdfSubtraction(sdfBox(p, box2_c, box2_b), sdfSphere(p, sphere2_c, sphere2_r));
    float sphere_sphere_sdf = sdfIntersection(sdfSphere(p, sphere3_c, sphere3_r), sdfSphere(p, sphere4_c, sphere4_r));
    
    s = plane1_sdf;
    s = sdfUnion(s, sphere1_sdf);
    s = sdfUnion(s, box1_sdf);
    s = sdfUnion(s, box_sphere_sdf);
    s = sdfUnion(s, sphere_sphere_sdf);
    
    int material_id = 0;
    if (s == plane1_sdf) {
        material_id = 0;
    } else if (s == sphere1_sdf) {
        material_id = 1;
    } else if (s == box1_sdf) {
        material_id = 2;
    } else if (s == box_sphere_sdf) {
        material_id = 3;
    } else if (s == sphere_sphere_sdf) {
        material_id = 4;
    }
    //// your implementation ends

    return vec2(s, material_id);
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
        float sdf = sdf(p).x;
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
    float s = sdf(p).x;          //// sdf value in p
    float dx = 0.01;           //// step size for finite difference

    float nx = sdf(vec3(p.x + dx, p.y, p.z)).x - s;
    float ny = sdf(vec3(p.x, p.y + dx, p.z)).x - s;
    float nz = sdf(vec3(p.x, p.y, p.z + dx)).x - s;
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

    vec3 color = vec3(1.0, 1.0, 1.0);

    int material_id = int(sdf(p).y);
    switch(material_id) {
        case 0:
            color = vec3(0.1, 0.8, 0.1);
            break;
        case 1:
            color = vec3(1.0, 0.05, 0.05);
            break;
        case 2:
            color = vec3(0.0, 0.9, 0.1);
            break;
        case 3:
            color = vec3(0.0, 0.1, 0.95);
            break;
        case 4:
            color = vec3(0.7, 0.7, 1.0);
            break;
    }

    return (amb + dif + spec + sunDif) * color;
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

float sdfSmoothUnion(float a, float b, float k) {
    return -k * log2(exp2(-a/k) + exp2(-b/k));
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