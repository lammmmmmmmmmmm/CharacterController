## Camera
- Camera rotation speed/damping
- Camera follow speed/damping
- Look ahead
- [ ] Support Cinemachine both v2.x and v3.x
- [x] Mouse smooth
- Zoom

## Input
- [x] New input system
- [ ] Joystick controls speed

## Jump
- [ ] Hold to jump higher
- [x] Jump buffer
- [x] Coyote time
- [ ] Limit air control
- [x] Ceiling check
- [x] Ground check
- [x] Max height
- [x] Time to reach max height
  - Animation curve
- [x] Gravity
- Having a fall curve is quite complicated, since player can fall from different heights.
  
## Movement
- [ ] Turn acceleration
- [x] Max speed
- [x] Multiple speeds
  - Walk
  - Run
- Since we are using multiple speeds, the acceleration and deceleration should be linear.
  - If we want to use curves, we have to make different acceleration and deceleration curves for each speed.

## Physics Interaction
- [x] Slope
- [x] Stairs
- [x] Uneven ground
- [ ] Wall jump
- [x] Moving platform
- [x] Rotating platform
- [ ] Fluid
- [x] Trampoline

## Juice
- Particle effects
  - [x] Landing dust
  - [x] Jump dust
  - [ ] Running dust
- Squash and stretch
  - Landing
  - Jump
- Animation
  - Lean while running