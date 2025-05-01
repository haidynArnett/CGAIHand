precision highp float;              //// set default precision of float variables to high precision

varying vec2 vUv;                   //// screen uv coordinates (varying, from vertex shader)
uniform vec2 iResolution;           //// screen resolution (uniform, from CPU)
uniform float iTime;                //// time elapsed (uniform, from CPU)
uniform float iTimeDelta;
uniform float iFrame;
uniform sampler2D iChannel0;
const vec3 CAM_POS = vec3(-0.35, 1.0, -1.0);
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

 vec3[5] lengths = vec3[5](
    vec3(0.12, 0.09, 0.),
    vec3(0.15, 0.13, 0.1),
    vec3(0.15, 0.13, 0.1),
    vec3(0.15, 0.13, 0.1),
    vec3(0.12, 0.11, 0.08)
);
const vec3[5] open_rotations = vec3[5](
    vec3(0.1, 0.1, 0.),
    vec3(0.1, 0.1, 0.1),
    vec3(0.1, 0.1, 0.1),
    vec3(0.1, 0.1, 0.1),
    vec3(0.1, 0.1, 0.1)
);
const vec3[5] closed_rotations = vec3[5](
    vec3(1.2, 1.2, 0.),
    vec3(1., 1., 1.),
    vec3(1., 1., 1.),
    vec3(1., 1., 1.),
    vec3(1., 1., 1.)
);

float sdf_hand(vec3 p)
{
    vec3[5] rotations;
    for (int i = 0; i < 5; ++i) {
        float k1 = (cos(iTime) + 1.) / 2.;
        float k2 = 1. - k1;
        rotations[i] = k1 * open_rotations[i] + k2 * closed_rotations[i];
    }

    p -= vec3(0., 1.0, 1.);
    p = rotatePointXYZ(p, vec3(0.), -PI / 3., 0., PI / 2.);

    // // Thumb
    p = rotatePointZ(p, PI / 1.8);
    float s = thumb(
        rotatePointZ(p - vec3(0.0, 0.5, 0.), -PI / 6.),
        lengths[0].xy,
        rotations[0].xy
    );
    p = rotatePointZ(p, -PI / 1.8);

    // // Fingers
    s = sdfUnion(s, finger(
        p - vec3(0.3, 0.3, 0.),
        lengths[1],
        rotations[1]
    ));

    s = sdfUnion(s, finger(
        p - vec3(0.1, 0.4, 0.),
        lengths[2],
        rotations[2]
    ));

    s = sdfUnion(s, finger(
        p - vec3(-0.1, 0.4, 0.),
        lengths[3],
        rotations[3]
    ));

    s = sdfUnion(s, finger(
        p - vec3(-0.3, 0.3, 0.),
        lengths[4],
        rotations[4]
    ));


    // Palm
    p = rotatePointX(p, PI / 2.);
    float palm_s = sdfCappedCylinder(p, 0.06, 0.5);
    palm_s = sdfSubtraction(palm_s, sdfBox(p, vec3(-0.65, 0., 0.), vec3(0.2, 0.1, 0.8)));
    palm_s = sdfSubtraction(palm_s, sdfBox(p, vec3(0., 0., -0.5), vec3(0.5, 0.1, 0.1)));
    s = sdfSmoothUnion(s, palm_s, 0.05);
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
const float damp = 0.2;
const float collision_dist = 0.2;
const float ground_collision_dist = 0.05;
const float GRAVITY_FACTOR = 200.0;
const vec3 gravity = vec3(0.0, -GRAVITY_FACTOR, 0.0);

const int MAX_PARTICLES = 20;
// const int MAX_SPRINGS = 20;

Particle particles[MAX_PARTICLES];

const int initial_particles = 20;
int n_particles = initial_particles; // TODO: for now

// https://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
float rand(vec2 co){
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 random_init_position(int i) {
    float random = sin(rand(vec2(float(i), float(i))));
    // return vec3(0.0 + 0.05 * random, 2.0, 1.0 + 0.05 * random2);
    return vec3(0.5, 2.0 + random * 0.1, 1.0);
}

void init_state(void){
    for (int i = 1; i < initial_particles; i++) {
        particles[i].pos  = random_init_position(i);
        particles[i].pos_prev = particles[i].pos;
        particles[i].vel = vec3(0.0);
        particles[i].inv_mass = 1.0;
        particles[i].is_fixed = false;
        particles[i].radius = 0.03;
        particles[i].color = vec3(1.0, 1.0, 1.0);
    }
}

bool is_initializing() {
    return iTime < 0.06 || iFrame < 2.;
}
void load_state() {
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
    float dist = length(a - b);
    if(dist < collision_dist){
        return dist - collision_dist;
    }
    else{
        return 0.0;
    }
}

vec3 collision_constraint_gradient(vec3 a, vec3 b, float collision_dist){
    float dist = length(a - b);
    if(dist <= collision_dist){
        if (a == b) {
            return vec3(0.0);
        }
        vec3 grad = normalize(a - b);
        return grad;
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

    vec3 grad = collision_constraint_gradient(particles[i].pos, particles[j].pos, combinedRadius);
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

vec3 hand_sdf_gradient(vec3 p){
    float dx = 0.01;
    float s = sdf_hand(p);
    return normalize(vec3(sdf_hand(vec3(p.x + dx, p.y, p.z)) - s, sdf_hand(vec3(p.x, p.y + dx, p.z)) - s, sdf_hand(vec3(p.x, p.y, p.z + dx)) - s));
}

float ground_constraint(vec3 p, float ground_collision_dist){
    if(sdf_hand(p) < ground_collision_dist){
        return sdf_hand(p) - ground_collision_dist;
    }
    else{
        return 0.0;
    }    
}

vec3 ground_constraint_gradient(vec3 p, float ground_collision_dist){
    // Compute the gradient of the ground constraint with respect to p.

    if(sdf_hand(p) < ground_collision_dist){
        //// Your implementation starts
        return hand_sdf_gradient(p) * GRAVITY_FACTOR;
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

    vec3 grad = ground_constraint_gradient(particles[i].pos, ground_collision_dist);
    numer = -ground_constraint(particles[i].pos, ground_collision_dist);
    denom = length(ground_constraint_gradient(particles[i].pos, ground_collision_dist)) * particles[i].inv_mass;

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
        return vec3(0.1, 0.1, 0.1);
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

    vec3 color = vec3(0.0);
    if(sdf(p) == sdf_hand(p)){
        color = vec3(1., 1., 1.);
    }
    else{
        // Create a rainbow gradient based on position
        float r = 0.5 + 0.5 * sin(p.x * 3.0 + iTime);
        float g = 0.5 + 0.5 * sin(p.y * 4.0 + iTime * 1.3);
        float b = 0.5 + 0.5 * sin(p.z * 5.0 + iTime * 0.7);
        color = vec3(r, g, b);
    }

    return (amb + dif + spec + sunDif) * color;
}

/////////////////////////////////////////////////////
//// main function
/////////////////////////////////////////////////////

vec3 render_scene(vec2 pixel_xy) {
    float pixel_size = 2.0 / iResolution.y;
    // screen uv
    vec2 uv = pixel_xy;
    vec3 origin = CAM_POS;                                                  //// camera position 
    vec3 dir = normalize(vec3(uv.x, uv.y, 1));                              //// camera direction
    float s = rayMarching(origin, dir);                                     //// ray marching
    vec3 p = origin + dir * s;                                              //// ray-sdf intersection
    vec3 n = normal(p);                                                     //// sdf normal
    vec3 color = phong_shading(p, n);                                       //// phong shading
    return color;
}

vec4 output_color(vec2 pixel_ij){
    int i = int(pixel_ij.x);
    int j = int(pixel_ij.y);
    
    if(j == 0){
        if(i==0){
            return vec4(float(n_particles), 0.0, 0.0, 0.0);
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

                    if (particles[j].pos.y < 0.0) {
                        particles[j].pos = random_init_position(j);
                        particles[j].vel = vec3(0.0);
                    }
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
    if(initialized){
        gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
    }
}