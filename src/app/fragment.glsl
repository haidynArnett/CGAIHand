precision highp float;              //// set default precision of float variables to high precision

varying vec2 vUv;                   //// screen uv coordinates (varying, from vertex shader)
uniform vec2 iResolution;           //// screen resolution (uniform, from CPU)
uniform float iTime;                //// time elapsed (uniform, from CPU)
uniform float iTimeDelta;
uniform float iFrame;
uniform sampler2D iChannel0;
const vec3 CAM_POS = vec3(-0.35, 0.0, -2.0);
bool initialized = false;

vec2 screen_to_xy(vec2 coord) {
    return (coord - 0.5 * iResolution.xy) * 2.0 / iResolution.y;
}

float remap01(float inp, float inp_start, float inp_end) {
    return clamp((inp - inp_start) / (inp_end - inp_start), 0.0, 1.0);
}
float dist_sqr(vec2 a, vec2 b) {
    vec2 diff = a - b;
    return dot(diff, diff);
}


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
float base_knuckle_rad = 0.02;
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

float sdf_hand(vec3 p) {
    p = rotatePointXYZ(p, vec3(0.), -PI / 3., 0., PI / 2.);

    // Thumb
    p = rotatePointZ(p, PI / 2.2);
    float s = thumb(
        rotatePointZ(p - vec3(0.0, 0.5, 0.), -PI / 3.),
        vec2(0.12, 0.09),
        vec2(0.1, 0.1)
    );
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


    p = rotatePointX(p, PI / 2.);
    s = sdfSmoothUnion(s, sdfCappedCylinder(p, 0.06, 0.5), 0.04);
    p = rotatePointX(p, -PI / 2.);
    return s;
}

struct Particle {
    vec3 pos;
    vec3 pos_prev;
    vec3 vel;
    float inv_mass;
    bool is_fixed;
    float radius; // Added radius field for different particle sizes
    vec3 color;   // Color of the particle (RGB)
};

// Simulation constants
const float damp = 0.4;
const float collision_dist = 0.2;
const float ground_collision_dist = 0.15;
const vec3 gravity = vec3(0.0, -100.0, 0.0);

// Define n_rope rope particles and add one extra "mouse particle".
const int MAX_PARTICLES = 20;
// const int MAX_SPRINGS = 20;

Particle particles[MAX_PARTICLES];

const int initial_particles = 20;
int n_particles = initial_particles; // TODO: for now

// https://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
float rand(vec2 co){
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

void init_state(void){
    // particles[0].pos = vec3(-2.0, 1.0, 0.0);
    // particles[1].pos = vec3(-1.0, 2.0, 0.0);
    // particles[2].pos = vec3(0.0, 2.0, 0.0);
    // particles[3].pos = vec3(1.0, 2.0, 0.0);
    // particles[4].pos = vec3(2.0, 2.0, 0.0);
    // particles[5].pos = vec3(1.5, 2.0, 0.1);

    for (int i = 1; i < initial_particles; i++) {
        float start = float(i) / float(initial_particles) * 2.0;
        float random = sin(rand(vec2(float(i), float(i))));
        float random2 = sin(rand(vec2(float(i+1), float(i+1))));

        particles[i].pos  = vec3(random, start, random2);
        particles[i].pos_prev = particles[i].pos;
        particles[i].vel = vec3(0.0);
        particles[i].inv_mass = 1.0;
        particles[i].is_fixed = false;
        particles[i].radius = 0.03;
        particles[i].color = vec3(1.0, 0.0, 0.0);
    }
}

bool is_initializing() {
    return iTime < 0.06 || iFrame < 2.;
}
// // Load rope particles from the previous frame and update the mouse particle.
void load_state() {
    //0,0: (num_particles, num_springs, selected_particle)

    vec4 data = texelFetch(iChannel0, ivec2(0, 0), 0);
    n_particles = int(data.x);

    // Load other particles
    for (int i = 1; i < n_particles; i++) {
        vec4 pos_0 = texelFetch(iChannel0, ivec2(i, 0), 0);
        vec4 vel_0 = texelFetch(iChannel0, ivec2(i, 1), 0);
        vec4 extraData = texelFetch(iChannel0, ivec2(i, 2), 0);

        particles[i].pos = pos_0.xyz;
        particles[i].vel = vel_0.xyz;
        particles[i].inv_mass = 1.0; // default mass
        particles[i].is_fixed = false;
        particles[i].radius = extraData.x > 0.0 ? extraData.x : 0.1; // Default to 0.1 if not set
        particles[i].color = extraData.yzw; // Read color data
    }
}

float collision_constraint(vec3 a, vec3 b, float collision_dist){
    // Compute the distance between two particles a and b.
    // The constraint is defined as L - L0, where L is the current distance between a and b
    // and L0 = collision_dist is the minimum distance between a and b.

    float dist = length(a - b);
    if(dist < collision_dist){
        //// Your implementation starts
        return dist - collision_dist;
        //// Your implementation ends
    }
    else{
        return 0.0;
    }
}

vec3 collision_constraint_gradient(vec3 a, vec3 b, float collision_dist){
    // Compute the gradient of the collision constraint with respect to a.

    float dist = length(a - b);
    if(dist <= collision_dist){
        //// Your implementation starts
        if (a == b) {
            return vec3(0.0);
        }
        vec3 grad = normalize(a - b);
        return grad;
        //// Your implementation ends
    }
    else{
        return vec3(0.0);
    }
}

void solve_collision_constraint(int i, int j, float collision_dist, float dt){
    // Use the sum of particle radii as the collision distance
    float combinedRadius = particles[i].radius + particles[j].radius;
    
    // Compute the collision constraint for particles i and j.
    float numer = 0.0;
    float denom = 0.0;

    //// Your implementation starts
    vec3 grad = collision_constraint_gradient(particles[i].pos, particles[j].pos, combinedRadius);
    //// Your implementation ends
    numer = -collision_constraint(particles[i].pos, particles[j].pos, combinedRadius);
    denom = length(collision_constraint_gradient(particles[i].pos, particles[j].pos, combinedRadius)) * particles[i].inv_mass +
            length(collision_constraint_gradient(particles[j].pos, particles[i].pos, combinedRadius)) * particles[j].inv_mass;

    //PBD if you comment out the following line, which is faster
    denom += (1. / 1000.) / (dt * dt);

    if (denom == 0.0) return;
    float lambda = numer / denom;
    particles[i].pos += lambda * particles[i].inv_mass * grad;
    particles[j].pos -= lambda * particles[j].inv_mass * grad;
}

// float phi(vec3 p){
//     const float PI = 3.14159265359;
//     //let's do sin(x)+0.5
//     // return p.y - (0.1 * sin(p.x * 2. * PI) - 0.5);
//     return p.y - 0.0;
// }


vec3 hand_sdf_gradient(vec3 p){
    float dx = 0.01;
    float s = sdf_hand(p);
    return normalize(vec3(sdf_hand(vec3(p.x + dx, p.y, p.z)) - s, sdf_hand(vec3(p.x, p.y + dx, p.z)) - s, sdf_hand(vec3(p.x, p.y, p.z + dx)) - s));
}

float ground_constraint(vec3 p, float ground_collision_dist){
    if(sdf_hand(p) < ground_collision_dist){
        //// Your implementation starts
        return sdf_hand(p) - ground_collision_dist;
        //// Your implementation ends
    }
    else{
        return 0.0;
    }    
}

vec3 ground_constraint_gradient(vec3 p, float ground_collision_dist){
    // Compute the gradient of the ground constraint with respect to p.

    if(sdf_hand(p) < ground_collision_dist){
        //// Your implementation starts
        return hand_sdf_gradient(p);
        //// Your implementation ends
    }
    else{
        return vec3(0.0, 0.0, 0.0);
    }
}

void solve_ground_constraint(int i, float ground_collision_dist, float dt){
    // Compute the ground constraint for particle i.
    float numer = 0.0;
    float denom = 0.0;

    //// Your implementation starts
    vec3 grad = ground_constraint_gradient(particles[i].pos, ground_collision_dist);
    numer = -ground_constraint(particles[i].pos, ground_collision_dist);
    denom = length(ground_constraint_gradient(particles[i].pos, ground_collision_dist)) * particles[i].inv_mass;

    //// Your implementation ends

    //PBD if you comment out the following line, which is faster
    denom += (1. / 1000.) / (dt * dt);

    if (denom == 0.0) return;
    float lambda = numer / denom;
    particles[i].pos += lambda * particles[i].inv_mass * grad;
}

void solve_constraints(float dt) {
    for (int i = 1; i < n_particles; i++) {
        solve_ground_constraint(i, ground_collision_dist, dt);
    }
    for (int i = 1; i < n_particles; i++) {
        for (int j = i + 1; j < n_particles; j++) {
            solve_collision_constraint(i, j, collision_dist, dt);
        }
    }
    //// Your implementation ends
}


float sdf(vec3 p)
{
    float s = 0.0;
    // s = sdfPlane(p, 0.0);
    s = sdfSphere(p, particles[0].pos, particles[0].radius);
    for (int i = 1; i < n_particles; i++) {
        s = sdfSmoothUnion(s, sdfSphere(p, particles[i].pos, particles[i].radius), 0.05);
    }
    s = sdfUnion(s, sdf_hand(p));
    // s = sdfSphere(p, vec3(0., 2., 0.), 0.5);
    // p -= vec3(0., 1.0, 1.);

    // Palm

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


vec3 render_scene(vec2 pixel_xy) {
    // float phi = phi(pixel_xy);
    // if(phi < 0.0) {
    //     col =  vec3(122, 183, 0) / 255.; // ground color
    // }
    // else{
    //     col = vec3(229, 242, 250) / 255.; // background color
    // }
    
    float pixel_size = 2.0 / iResolution.y;
    
    // If still initializing, return the background color.
    // if (is_initializing()) {
    //     return vec3(0.9, 0.6, 0.2);
    // }

    // // Render rope particles - modified to use per-particle radius and color
    // {
    //     for (int i = 0; i < n_particles; i++){
    //         float dist = length(pixel_xy - particles[i].pos);
    //         float radius = particles[i].radius;
    //         //TODO: what is this??
    //         float effect = remap01(dist, radius, radius - pixel_size);
    //         if (effect > 0.0) {
    //             // Use particle's color instead of a fixed color
    //             col = mix(col, particles[i].color, effect);
    //         }
    //     }
        
    // }
    
    // Render All springs
    // {
    //     float min_dist = 1e9;

    //     if(iMouse.z == 1.){
    //         min_dist = dist_to_segment(pixel_xy, particles[0].pos, particles[selected_particle].pos);
    //     }

    //     for (int i = 1; i < n_springs; i++) {
    //         int a = springs[i].a;
    //         int b = springs[i].b;
    //         min_dist = min(min_dist, dist_to_segment(pixel_xy, particles[a].pos, particles[b].pos));
    //     }

    //     const float thickness = 0.01;
        
    //     col = mix(col, vec3(0.3, 0.3, 0.3), 0.25 * remap01(min_dist, thickness, thickness - pixel_size));
    // }


    // TODO: merge this with particles
    // screen uv
    vec2 uv = pixel_xy;
    vec3 origin = CAM_POS;                                                  //// camera position 
    vec3 dir = normalize(vec3(uv.x, uv.y, 1));                              //// camera direction
    float s = rayMarching(origin, dir);                                     //// ray marching
    vec3 p = origin + dir * s;                                              //// ray-sdf intersection
    vec3 n = normal(p);                                                     //// sdf normal
    vec3 color = phong_shading(p, n);                                       //// phong shading
    // fragColor = vec4(color, 1.);    
    return color;
}

vec4 output_color(vec2 pixel_ij){
    int i = int(pixel_ij.x);
    int j = int(pixel_ij.y);
    
    if(j == 0){
        // (0,0): (num_particles, num_springs, selected_particle)
        if(i==0){
            // return vec4(float(n_particles), float(n_springs), float(selected_particle), float(current_add_particle));
            return vec4(float(n_particles), 0.0, 0.0, 0.0);
            // return vec4(1.0, 0.0, 0.0, 0.0);
        }
        else if(i < n_particles){
            //a particle
            return vec4(particles[i].pos, 0.0);
        }
        else{
            return vec4(0.0, 0.0, 0.0, 1.0);
        }
    }

    else if(j == 1){
        if(i < n_particles){
            // return vec4(float(springs[i].a), float(springs[i].b), springs[i].restLength, springs[i].inv_stiffness);
            return vec4(particles[i].vel, 0.0);
        }
        else{
            return vec4(0.0, 0.0, 0.0, 1.0);
        }
    }
    else if(j == 2){
        // Store additional particle data (radius and color)
        if(i < n_particles){
            return vec4(particles[i].radius, particles[i].color);
        }
        else{
            return vec4(0.0, 0.0, 0.0, 1.0);
        }
    }
    else{
        vec2 pixel_xy = screen_to_xy(pixel_ij);
        vec3 color = render_scene(pixel_xy);
        return vec4(color, 1.0);
    }
}


// Main function
void main() {
    vec2 pixel_ij = vUv * iResolution.xy;
    int pixel_i = int(pixel_ij.x);
    int pixel_j = int(pixel_ij.y);

    if(is_initializing()){
        init_state();
    }
    else{
        load_state();
        if (pixel_j == 0) {
            if (pixel_i >= n_particles) return;

            float actual_dt = min(iTimeDelta, 0.02);
            const int n_steps = 5;
            float dt = actual_dt / float(n_steps);

            for (int i = 0; i < n_steps; i++) {
                // Update rope particles only; skip updating the mouse particle since it's fixed.
                for (int j = 0; j < n_particles; j++) {
                    if (!particles[j].is_fixed)
                        particles[j].vel += dt * gravity;
                    particles[j].vel *= exp(-damp * dt);
                    particles[j].pos_prev = particles[j].pos;
                    particles[j].pos += dt * particles[j].vel;
                }
                solve_constraints(dt);
                // Update velocities for rope particles only.
                for (int j = 0; j < n_particles; j++) {
                    if (!particles[j].is_fixed){
                        particles[j].vel = (particles[j].pos - particles[j].pos_prev) / dt;
                        // initialized = true;
                    }
                }
            }
        }
    }


    gl_FragColor = output_color(pixel_ij);
    // mainImage(gl_FragColor, gl_FragCoord.xy);
    if(initialized){
        gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
    }
}