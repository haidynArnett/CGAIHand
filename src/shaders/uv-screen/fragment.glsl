varying vec2 vUv;

uniform float uTime;
uniform vec2 uResolution;

void main()
{
  gl_FragColor = vec4(vUv.x, vUv.y, 0.5 + 0.5 * sin(uTime * 2.), 1.0);
}
