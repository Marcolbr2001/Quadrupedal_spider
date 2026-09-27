#include "Legs.h"

// ==================================================
// === SERVO MAP ===
// ==================================================
// Leg i, joint j -> PCA9685 channel. See PINOUT.md for the full table.
// It is NOT sequential: on the board the first six connectors go to the
// RIGHT-hand legs and the last six to the LEFT-hand ones. The triples stay
// whole, only which leg sits on which group changes.
// Joint 0 = alpha (lift, middle servo), 1 = beta (knee, outermost),
// 2 = gamma (rotation, the servo bolted to the body).
static const uint8_t CHANNEL[4][3] = {
  { 6,  7,  8},   // leg 0 -> scheda 7-9   (J13, J14, J15)  Leg-1 anteriore left
  { 9, 10, 11},   // leg 1 -> scheda 10-12 (J16, J17, J18)  Leg-2 posteriore left
  { 0,  1,  2},   // leg 2 -> scheda 1-3   (J7,  J8,  J9)   Leg-3 anteriore right
  { 3,  4,  5}    // leg 3 -> scheda 4-6   (J10, J11, J12)  Leg-4 posteriore right
};

// Per-servo correction, in microseconds. Needed because no servo is
// mounted perfectly straight: stand the robot up, see which joint droops,
// and nudge it here +/- 20 us at a time.
static const int TRIM_US[4][3] = {
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

// If a servo turns the opposite way to the kinematics, set it true.
static const bool REVERSED[4][3] = {
  { false, false, false },
  { false, false, false },
  { false, false, false },
  { false, false, false }
};

// The microsecond endpoints for 0 and 180 degrees.
// Same values as the esp_servos.ino test.
static const int US_0    = 500;
static const int US_180  = 2400;

// ==================================================
// === GEOMETRY (identical to the ATMega version) ===
// ==================================================
static const float length_a = 55, length_b = 77.5, length_c = 27.5, length_side = 71;
static const float z_absolute = -28;
static const float z_default = -50, z_up = -30, z_boot = z_absolute;
static const float x_default = 62, x_offset = 0;
static const float y_start = 0, y_step = 40;
static const float PI_F = 3.1415926f;

// The "leave this axis alone" marker, the original's KEEP.
static const float K = 255.0f;

// --- BODY ATTITUDE ---
// Half the spacing between the hips. The original kinematics has a single
// value for the body side, so we treat it as square: if your chassis is
// distinctly rectangular, measure it and put the two real half-spacings
// here.
static const float HX = length_side / 2;
static const float HY = length_side / 2;

// Direction of each leg's local axes relative to the body, where the world
// has +x to the right, +y forward and +z up. Every leg has local +x
// pointing outwards sideways and local +y pointing outwards fore-aft.
// Derived from the ATMega code: body_left() grows the x of the left-hand
// legs (so the body goes right), and step_forward's transfer pose lowers
// the y of the front legs and raises that of the rear ones, which is
// exactly the body moving forward.
// MEASURED with the pose mode: the real layout is rotated 180 degrees from
// what I had deduced from the drawing. The head is at the other end, so leg
// 0 is not the front-left but the REAR-RIGHT, and so on. The diagonals
// {0,3} and {1,2} are still diagonals, so the trot is unaffected, and the
// gaits do not use this table at all: it only matters for body attitude.
//                   0 = rear R   1 = front R  2 = rear L   3 = front L
static const float SIGN_X[4] = { +1, +1, -1, -1 };   // +1 = leg a right
static const float SIGN_Y[4] = { -1, +1, -1, +1 };   // +1 = leg davanti

static const float DEG = 3.14159265f / 180.0f;

// How fast the attitude may change. Without this limit a setAttitude() of
// 15 degrees would be executed whole in one tick: some twenty millimetres
// of foot travel in 20 ms, which is a hammer blow to the servos. With it,
// the body eases into the move.
// Attitude senses, measured with the pose mode on the real robot. They are
// needed because each axis's sense depends on how the legs are actually
// arranged around the body, and that only comes out by trying.
//   +1 = the command does what its name says
//   -1 = it needs flipping
// In the end they were all +1: the only real error was the SIGN_* table
// above, that is, how the legs sit around the body. Once that was fixed,
// roll, pitch, yaw, height and lateral all come out right by themselves.
// They stay here as knobs, in case the chassis ever changes.
static const float SENSE_ROLL     = +1;   // verificato
static const float SENSE_PITCH = +1;   // verificato
static const float SENSE_YAW  = +1;   // verificato
static const float SENSE_HEIGHT    = +1;   // verificato
static const float SENSE_LATERAL   = +1;   // verificato
static const float SENSE_FWD     = -1;   // vedi nota qui sotto

// SENSE_FWD is the one axis no pose in the pose mode ever exercised. The
// trot settled it: the crawl's transfer pose, which works, moves the body
// forward by RAISING the y of the front legs and LOWERING that of the rear
// ones. The attitude transform on its own did the opposite, so it has to be
// flipped.

static const float SPD_ANGLE = 60.0f;   // degrees al secondo
static const float SPD_SHIFT  = 80.0f;   // mm al secondo

// Moves a value towards its target by at most 'step'.
static float approachF(float value, float target, float step) {
  if (value < target) {
    value += step;
    if (value > target) value = target;
  } else if (value > target) {
    value -= step;
    if (value < target) value = target;
  }
  return value;
}

// Speeds in mm/s. On the ATMega they were mm per 20 ms tick:
// leg_move_speed 8 -> 400 mm/s, body_move_speed 3 -> 150 mm/s,
// spot_turn_speed 4 -> 200 mm/s, stand_seat_speed 1 -> 50 mm/s.
// Note: the smoothstep easing peaks at 1.5 times the average speed, which
// the ATMega's constant-velocity code never did. With SPD_LEG at 400 the
// peak reached 600 mm/s and the servos could not keep up: the legs slipped
// and forward/backward made no progress. The turns already worked because
// they run at 200, so SPD_LEG was brought down to match them.
static const float SPD_LEG  = 200;   // era 400
static const float SPD_BODY  = 150;
static const float SPD_TURN   = 200;
static const float SPD_LIFT = 50;

// Geometry of the turn on the spot: it depends on acos/sqrt, so it is
// computed at program start rather than at compile time.
static const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
static const float temp_b = 2 * (y_start + y_step) + length_side;
static const float temp_c = sqrt(pow(2 * x_default + length_side, 2) +
                                 pow(2 * y_start + y_step + length_side, 2));
static const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) /
                                     2 / temp_a / temp_b);
static const float turn_x1 = (temp_a - length_side) / 2;
static const float turn_y1 = y_start + y_step / 2;
static const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
static const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

// ==================================================
// === THE POSES ===
// ==================================================
// A pose says where each leg must end up and how fast. It is the one-to-one
// translation of the ATMega's "set_site(...); wait_all_reach();" blocks,
// except that here nobody waits in a while().
struct Pose {
  float v;
  float p[4][3];
  float attesaMs;   // minimum duration: needed by the double-support dwells,
                    // which move nothing and would otherwise last one tick
};

// --- STAND / SIT ---
static const Pose SEQ_STAND[] = {
  { SPD_LIFT, {{K,K,z_default},{K,K,z_default},{K,K,z_default},{K,K,z_default}} }
};
static const Pose SEQ_SIT[] = {
  { SPD_LIFT, {{K,K,z_boot},{K,K,z_boot},{K,K,z_boot},{K,K,z_boot}} }
};

// --- FORWARD, CRAWL STEP (the original's step_forward) ---
static const Pose SEQ_FWD_A[] = {
  { SPD_LEG, {{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_up},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{x_default+x_offset, y_start+2*y_step, z_up},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{x_default+x_offset, y_start+2*y_step, z_default},{K,K,K}} },
  { SPD_BODY, {{x_default+x_offset, y_start,          z_default},
                {x_default+x_offset, y_start+2*y_step, z_default},
                {x_default-x_offset, y_start+y_step,   z_default},
                {x_default-x_offset, y_start+y_step,   z_default}} },
  { SPD_LEG, {{K,K,K},{x_default+x_offset, y_start+2*y_step, z_up},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{x_default+x_offset, y_start, z_up},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{x_default+x_offset, y_start, z_default},{K,K,K},{K,K,K}} }
};
static const Pose SEQ_FWD_B[] = {
  { SPD_LEG, {{x_default+x_offset, y_start, z_up},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{x_default+x_offset, y_start+2*y_step, z_up},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{x_default+x_offset, y_start+2*y_step, z_default},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_BODY, {{x_default-x_offset, y_start+y_step,   z_default},
                {x_default-x_offset, y_start+y_step,   z_default},
                {x_default+x_offset, y_start,          z_default},
                {x_default+x_offset, y_start+2*y_step, z_default}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start+2*y_step, z_up}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_up}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_default}} }
};

// --- BACKWARD, CRAWL STEP (the original's step_back) ---
static const Pose SEQ_BACK_A[] = {
  { SPD_LEG, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_up}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start+2*y_step, z_up}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start+2*y_step, z_default}} },
  { SPD_BODY, {{x_default+x_offset, y_start+2*y_step, z_default},
                {x_default+x_offset, y_start,          z_default},
                {x_default-x_offset, y_start+y_step,   z_default},
                {x_default-x_offset, y_start+y_step,   z_default}} },
  { SPD_LEG, {{x_default+x_offset, y_start+2*y_step, z_up},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{x_default+x_offset, y_start, z_up},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{x_default+x_offset, y_start, z_default},{K,K,K},{K,K,K},{K,K,K}} }
};
static const Pose SEQ_BACK_B[] = {
  { SPD_LEG, {{K,K,K},{x_default+x_offset, y_start, z_up},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{x_default+x_offset, y_start+2*y_step, z_up},{K,K,K},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{x_default+x_offset, y_start+2*y_step, z_default},{K,K,K},{K,K,K}} },
  { SPD_BODY, {{x_default-x_offset, y_start+y_step,   z_default},
                {x_default-x_offset, y_start+y_step,   z_default},
                {x_default+x_offset, y_start+2*y_step, z_default},
                {x_default+x_offset, y_start,          z_default}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{x_default+x_offset, y_start+2*y_step, z_up},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_up},{K,K,K}} },
  { SPD_LEG, {{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_default},{K,K,K}} }
};

// --- FORWARD AND BACKWARD, TROT (opposite legs together) ---
// The diagonals are {0,3} and {1,2}, confirmed by the pinout drawing and
// then on the robot itself:
//   {0,3} = rear right + front left
//   {1,2} = front right + rear left
//
// These tables are NOT constant like the others: they depend on stride,
// lift and the support dwell, all adjustable while the robot is walking.
// Each phase is made of four poses:
//   1. the flying diagonal lifts off, the other pushes a third of the way
//   2. the airborne diagonal moves forward, the other pushes the second third
//   3. the airborne diagonal lands, the other finishes its push
//   4. all four on the ground for a moment, to get the balance back
// That fourth beat is the difference between a trot that works and one that
// rocks: without it the robot goes from two feet to the opposite two with
// never a still moment in between.
//
// MIND the sense of y. Each leg's local y axis points OUTWARDS from the
// body: for the front legs +y is forwards, for the rear ones +y is
// backwards. So a diagonal, which always holds one front and one rear leg,
// must be driven with OPPOSITE y values to send both the same way in the
// world. It is what the original's crawl gait already does, where leg 2
// goes from 0 to 80 while leg 1 goes from 80 to 0.
// Legs: 0 = rear R, 1 = front R, 2 = rear L, 3 = front L.
static const uint8_t N_TROT = 4;
static Pose TROT_A[N_TROT], TROT_B[N_TROT];
static Pose TROT_BACK_A[N_TROT], TROT_BACK_B[N_TROT];
static Pose TROT_READY_FWD[1], TROT_READY_BACK[1];

// --- TURN LEFT (the original's turn_left) ---
static const Pose SEQ_LEFT_A[] = {
  { SPD_TURN, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_up}} },
  { SPD_TURN, {{turn_x1-x_offset, turn_y1, z_default},
               {turn_x0-x_offset, turn_y0, z_default},
               {turn_x1+x_offset, turn_y1, z_default},
               {turn_x0+x_offset, turn_y0, z_up}} },
  { SPD_TURN, {{K,K,K},{K,K,K},{K,K,K},{turn_x0+x_offset, turn_y0, z_default}} },
  { SPD_TURN, {{turn_x1+x_offset, turn_y1, z_default},
               {turn_x0+x_offset, turn_y0, z_default},
               {turn_x1-x_offset, turn_y1, z_default},
               {turn_x0-x_offset, turn_y0, z_default}} },
  { SPD_TURN, {{K,K,K},{turn_x0+x_offset, turn_y0, z_up},{K,K,K},{K,K,K}} },
  { SPD_TURN, {{x_default+x_offset, y_start,        z_default},
               {x_default+x_offset, y_start,        z_up},
               {x_default-x_offset, y_start+y_step, z_default},
               {x_default-x_offset, y_start+y_step, z_default}} },
  { SPD_TURN, {{K,K,K},{x_default+x_offset, y_start, z_default},{K,K,K},{K,K,K}} }
};
static const Pose SEQ_LEFT_B[] = {
  { SPD_TURN, {{x_default+x_offset, y_start, z_up},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_TURN, {{turn_x0+x_offset, turn_y0, z_up},
               {turn_x1+x_offset, turn_y1, z_default},
               {turn_x0-x_offset, turn_y0, z_default},
               {turn_x1-x_offset, turn_y1, z_default}} },
  { SPD_TURN, {{turn_x0+x_offset, turn_y0, z_default},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_TURN, {{turn_x0-x_offset, turn_y0, z_default},
               {turn_x1-x_offset, turn_y1, z_default},
               {turn_x0+x_offset, turn_y0, z_default},
               {turn_x1+x_offset, turn_y1, z_default}} },
  { SPD_TURN, {{K,K,K},{K,K,K},{turn_x0+x_offset, turn_y0, z_up},{K,K,K}} },
  { SPD_TURN, {{x_default-x_offset, y_start+y_step, z_default},
               {x_default-x_offset, y_start+y_step, z_default},
               {x_default+x_offset, y_start,        z_up},
               {x_default+x_offset, y_start,        z_default}} },
  { SPD_TURN, {{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_default},{K,K,K}} }
};

// --- TURN RIGHT (the original's turn_right) ---
static const Pose SEQ_RIGHT_A[] = {
  { SPD_TURN, {{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_up},{K,K,K}} },
  { SPD_TURN, {{turn_x0-x_offset, turn_y0, z_default},
               {turn_x1-x_offset, turn_y1, z_default},
               {turn_x0+x_offset, turn_y0, z_up},
               {turn_x1+x_offset, turn_y1, z_default}} },
  { SPD_TURN, {{K,K,K},{K,K,K},{turn_x0+x_offset, turn_y0, z_default},{K,K,K}} },
  { SPD_TURN, {{turn_x0+x_offset, turn_y0, z_default},
               {turn_x1+x_offset, turn_y1, z_default},
               {turn_x0-x_offset, turn_y0, z_default},
               {turn_x1-x_offset, turn_y1, z_default}} },
  { SPD_TURN, {{turn_x0+x_offset, turn_y0, z_up},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_TURN, {{x_default+x_offset, y_start,        z_up},
               {x_default+x_offset, y_start,        z_default},
               {x_default-x_offset, y_start+y_step, z_default},
               {x_default-x_offset, y_start+y_step, z_default}} },
  { SPD_TURN, {{x_default+x_offset, y_start, z_default},{K,K,K},{K,K,K},{K,K,K}} }
};
static const Pose SEQ_RIGHT_B[] = {
  { SPD_TURN, {{K,K,K},{x_default+x_offset, y_start, z_up},{K,K,K},{K,K,K}} },
  { SPD_TURN, {{turn_x1+x_offset, turn_y1, z_default},
               {turn_x0+x_offset, turn_y0, z_up},
               {turn_x1-x_offset, turn_y1, z_default},
               {turn_x0-x_offset, turn_y0, z_default}} },
  { SPD_TURN, {{K,K,K},{turn_x0+x_offset, turn_y0, z_default},{K,K,K},{K,K,K}} },
  { SPD_TURN, {{turn_x1-x_offset, turn_y1, z_default},
               {turn_x0-x_offset, turn_y0, z_default},
               {turn_x1+x_offset, turn_y1, z_default},
               {turn_x0+x_offset, turn_y0, z_default}} },
  { SPD_TURN, {{K,K,K},{K,K,K},{K,K,K},{turn_x0+x_offset, turn_y0, z_up}} },
  { SPD_TURN, {{x_default-x_offset, y_start+y_step, z_default},
               {x_default-x_offset, y_start+y_step, z_default},
               {x_default+x_offset, y_start,        z_default},
               {x_default+x_offset, y_start,        z_up}} },
  { SPD_TURN, {{K,K,K},{K,K,K},{K,K,K},{x_default+x_offset, y_start, z_default}} }
};

// --- WAVE (the original's hand_wave, leg 0) ---
// Measured on the robot: leg 0 is the FRONT RIGHT leg, leg 1 the rear
// right, leg 2 the front left, leg 3 the rear left. The SIGN_Y table up
// above disagrees; it is tuned for body attitude and it works, so it is
// left alone, but do not read the leg names off it.
//
// With leg 0 in the air the spider balances on the line joining leg 1
// (rear right) and leg 2 (front left). From a square stance that line runs
// straight through the centre of mass: the margin is not small, it is
// zero, and the robot drops onto the very leg it is waving.
//
// So two legs move, each along the axis that actually buys margin:
//   leg 1 sideways   swings the near end of that line outwards
//   leg 2 FORWARDS   carries the far end ahead of the centre of mass
// Leg 2 must not open sideways. That swings the line back the wrong way
// and costs more than leg 1 gains: +30 mm sideways takes the margin from
// 9.6 mm down to 1.3. Forwards does the opposite, 9.6 mm to 22.6, and
// about 32 mm once the weight has shifted too. Leg 3 is the far corner and
// never touches this edge.
static const float WAVE_WIDEN = 35;   // leg 1, sideways
static const float WAVE_FWD   = 30;   // leg 2, forwards
static const float WAVE_SHIFT = 15;   // weight shift, the old body_left

static const Pose SEQ_WAVE[] = {
  // 1. Build the base: rear right out to the side, front left forward
  { SPD_LIFT, {{K, K, K},
               {x_default + WAVE_WIDEN, K, K},
               {K, y_start + y_step + WAVE_FWD, K},
               {K, K, K}} },
  // 2. Shift the weight away from the leg that is about to lift
  { SPD_LIFT, {{x_default + WAVE_SHIFT, K, K},
               {x_default + WAVE_WIDEN, K, K},
               {x_default - WAVE_SHIFT, K, K},
               {x_default - WAVE_SHIFT, K, K}} },
  { SPD_BODY,  {{turn_x1, turn_y1, 50},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_BODY,  {{turn_x0, turn_y0, 50},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_BODY,  {{turn_x1, turn_y1, 50},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_BODY,  {{turn_x0, turn_y0, 50},{K,K,K},{K,K,K},{K,K,K}} },
  { SPD_BODY,  {{x_default+15, y_start+y_step, z_default},{K,K,K},{K,K,K},{K,K,K}} },
  // Body back to the middle, and the front left foot back into line: its
  // y is still the one carried forward at the start.
  { SPD_LIFT, {{x_default, K, K},
               {x_default, K, K},
               {x_default, y_start + y_step, K},
               {x_default, K, K}} }
};

#define N_POSES(a) (uint8_t)(sizeof(a) / sizeof(Pose))

#define LEGS_LOCK()   portENTER_CRITICAL(&_mux)
#define LEGS_UNLOCK() portEXIT_CRITICAL(&_mux)

// Rebuilds the trot tables from the current parameters.
void Legs::_buildTrot() {
  const float av = y_start + _trStride;              // foot carried forward
  const float t1 = y_start + _trStride * 2.0f / 3.0f;
  const float t2 = y_start + _trStride * 1.0f / 3.0f;
  const float zu = z_default + _trLift;           // height of the airborne foot
  const float za = z_default;
  const float ys = y_start;
  const float dwell = (float)_trSupport;

  // Going forward: the REAR legs (0, 2) push with falling y and recover
  // with rising y; the FRONT ones (1, 3) do the opposite.

  // phase A: 0 (rear) and 3 (front) fly, 1 (front) and 2 (rear) push
  TROT_A[0] = { SPD_LEG, {{K,K,zu},  {K,t2,K}, {K,t1,K}, {K,K,zu}},  0 };
  TROT_A[1] = { SPD_LEG, {{K,av,K},  {K,t1,K}, {K,t2,K}, {K,ys,K}},  0 };
  TROT_A[2] = { SPD_LEG, {{K,K,za},  {K,av,K}, {K,ys,K}, {K,K,za}},  0 };
  TROT_A[3] = { SPD_LEG, {{K,K,K},   {K,K,K},  {K,K,K},  {K,K,K}},   dwell };

  // phase B: 1 (front) and 2 (rear) fly, 0 (rear) and 3 (front) push
  TROT_B[0] = { SPD_LEG, {{K,t1,K},  {K,K,zu}, {K,K,zu}, {K,t2,K}},  0 };
  TROT_B[1] = { SPD_LEG, {{K,t2,K},  {K,ys,K}, {K,av,K}, {K,t1,K}},  0 };
  TROT_B[2] = { SPD_LEG, {{K,ys,K},  {K,K,za}, {K,K,za}, {K,av,K}},  0 };
  TROT_B[3] = { SPD_LEG, {{K,K,K},   {K,K,K},  {K,K,K},  {K,K,K}},   dwell };

  // Reverse: senses swapped, the rear rises and the front falls.
  TROT_BACK_A[0] = { SPD_LEG, {{K,K,zu}, {K,t1,K}, {K,t2,K}, {K,K,zu}}, 0 };
  TROT_BACK_A[1] = { SPD_LEG, {{K,ys,K}, {K,t2,K}, {K,t1,K}, {K,av,K}}, 0 };
  TROT_BACK_A[2] = { SPD_LEG, {{K,K,za}, {K,ys,K}, {K,av,K}, {K,K,za}}, 0 };
  TROT_BACK_A[3] = { SPD_LEG, {{K,K,K},  {K,K,K},  {K,K,K},  {K,K,K}},  dwell };

  TROT_BACK_B[0] = { SPD_LEG, {{K,t2,K}, {K,K,zu}, {K,K,zu}, {K,t1,K}}, 0 };
  TROT_BACK_B[1] = { SPD_LEG, {{K,t1,K}, {K,av,K}, {K,ys,K}, {K,t2,K}}, 0 };
  TROT_BACK_B[2] = { SPD_LEG, {{K,av,K}, {K,K,za}, {K,K,za}, {K,ys,K}}, 0 };
  TROT_BACK_B[3] = { SPD_LEG, {{K,K,K},  {K,K,K},  {K,K,K},  {K,K,K}},  dwell };

  // Starting stance: diagonals offset, ready for phase A.
  TROT_READY_FWD[0]  = { SPD_BODY, {{x_default, ys, za},{x_default, ys, za},
                                       {x_default, av, za},{x_default, av, za}}, 0 };
  TROT_READY_BACK[0] = { SPD_BODY, {{x_default, av, za},{x_default, av, za},
                                       {x_default, ys, za},{x_default, ys, za}}, 0 };
  _trRebuild = false;
}

void Legs::setTrot(float strideMm, float liftMm, uint16_t supportMs) {
  LEGS_LOCK();
  _trStride    = constrain(strideMm,  20.0f, 90.0f);
  _trLift   = constrain(liftMm,  8.0f, 35.0f);
  _trSupport = (supportMs > 400) ? 400 : supportMs;
  _trRebuild = true;
  _trStance   = 0;    // le tabelle nuove vogliono la loro stance
  LEGS_UNLOCK();
}

// ==================================================
// === KINEMATICS (identical to the ATMega version) ===
// ==================================================

static void cartesian_to_polar(float &alpha, float &beta, float &gamma,
                               float x, float y, float z) {
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) /
                             2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) /
              2 / length_a / length_b);
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  alpha = alpha / PI_F * 180;
  beta = beta / PI_F * 180;
  gamma = gamma / PI_F * 180;
}

// The original's per-leg corrections: the legs on one side are mounted
// mirrored with respect to the other.
static void polar_to_angoli(int leg, float &alpha, float &beta, float &gamma) {
  if (leg == 0)      { alpha = 90 - alpha; gamma += 90; }
  else if (leg == 1) { alpha += 90; beta = 180 - beta; gamma = 90 - gamma; }
  else if (leg == 2) { alpha += 90; beta = 180 - beta; gamma = 90 - gamma; }
  else if (leg == 3) { alpha = 90 - alpha; gamma += 90; }
}

// Gentle acceleration and braking instead of the original's constant
// velocity: this is what takes the jerk out of every waypoint.
static inline float smooth(float t) {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  return t * t * (3.0f - 2.0f * t);
}

// ==================================================
// === STARTUP ===
// ==================================================

Legs::Legs()
  : _pwm(LEGS_ADDR),
    _ready(false),
    _enabled(false),
    _useTask(false),
    _task(nullptr),
    _mux(portMUX_INITIALIZER_UNLOCKED),
    _gaitWanted(GAIT_STOP),
    _step(STEP_CRAWL),
    _multiplier(1.0f),
    _waveWanted(false),
    _roll(0), _pitch(0), _yaw(0),
    _height(0), _lateral(0), _fwd(0),
    _rollNow(0), _pitchNow(0), _yawNow(0),
    _heightNow(0), _lateralNow(0), _fwdNow(0),
    _gait(GAIT_STOP),
    _seq(nullptr),
    _nSeq(0),
    _iSeq(0),
    _phaseB(false),
    _tPose(0),
    _poseTime(0),
    _tTick(0) {
  // Outside the initialiser list: it is declared after the others and the
  // compiler would complain about the order.
  _trRebuild = true;
  _trStance = 0;
  _calActive = false;
  _calBlend = 1.0f;
  // Set in the body rather than the list: they are declared after the
  // other members and the compiler would flag the order.
  _trStride = 50; _trLift = 20; _trSupport = 120;
  _trRebuild = true;
  for (int i = 0; i < 16; i++) _usWritten[i] = -1;
}

bool Legs::begin(bool avviaBus) {
  // Very first thing: servos off. There is no pull-up on ~OE on this
  // board, so until we drive it the pin floats and the servos could jerk
  // into life with the legs still folded.
  pinMode(LEGS_OE, OUTPUT);
  digitalWrite(LEGS_OE, HIGH);
  _enabled = false;

  if (avviaBus) {
    if (!Wire.begin(LEGS_SDA, LEGS_SCL, LEGS_I2C_FREQ)) {
      Serial.println("LEGS: cannot start I2C bus 0!");
      return false;
    }
  }

  _pwm.begin();
  _pwm.setOscillatorFrequency(27000000);
  _pwm.setPWMFreq(LEGS_PWM_HZ);

  // Starting position: the same as the ATMega version's setup().
  float partenza[4][3] = {
    { x_default - x_offset, y_start + y_step, z_boot },
    { x_default - x_offset, y_start + y_step, z_boot },
    { x_default + x_offset, y_start,          z_boot },
    { x_default + x_offset, y_start,          z_boot }
  };
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      _siteNow[i][j] = _siteFrom[i][j] = _siteTo[i][j] = partenza[i][j];

  // Work out and send the pulses BEFORE switching the outputs on, so the
  // servos wake up already in the right pose instead of snapping to it.
  _writeServos();
  delay(50);
  enable();

  _buildTrot();

  _ready = true;
  _tTick = millis();
  Serial.println("LEGS: PCA9685 ready, spider crouched.");
  return true;
}

bool Legs::startTask(uint8_t core, uint32_t stack, UBaseType_t priorita) {
  if (!_ready || _useTask) return false;
  BaseType_t outcome = xTaskCreatePinnedToCore(_taskLoop, "legs", stack,
                                             this, priorita, &_task, core);
  _useTask = (outcome == pdPASS);
  if (_useTask) Serial.printf("LEGS: motion on its own task (core %u).\n", core);
  return _useTask;
}

void Legs::_taskLoop(void *param) {
  Legs *self = static_cast<Legs *>(param);
  const TickType_t period = pdMS_TO_TICKS(LEGS_TICK_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    self->_tick();
    vTaskDelayUntil(&last, period);
  }
}

void Legs::update() {
  if (!_ready || _useTask) return;
  unsigned long ora = millis();
  if (ora - _tTick < LEGS_TICK_MS) return;
  _tTick = ora;
  _tick();
}

// ==================================================
// === COMMANDS ===
// ==================================================

void Legs::stand()      { LEGS_LOCK(); _gaitWanted = GAIT_STOP; LEGS_UNLOCK();
                             _startSequence(SEQ_STAND, N_POSES(SEQ_STAND)); }
void Legs::sit()       { LEGS_LOCK(); _gaitWanted = GAIT_STOP; LEGS_UNLOCK();
                             _startSequence(SEQ_SIT, N_POSES(SEQ_SIT)); }
void Legs::forward()       { LEGS_LOCK(); _gaitWanted = GAIT_FORWARD;    LEGS_UNLOCK(); }
void Legs::backward()     { LEGS_LOCK(); _gaitWanted = GAIT_BACK;  LEGS_UNLOCK(); }
void Legs::turnLeft() { LEGS_LOCK(); _gaitWanted = GAIT_LEFT;  LEGS_UNLOCK(); }
void Legs::turnRight()   { LEGS_LOCK(); _gaitWanted = GAIT_RIGHT;    LEGS_UNLOCK(); }
void Legs::halt()      { LEGS_LOCK(); _gaitWanted = GAIT_STOP;     LEGS_UNLOCK(); }
void Legs::wave()       { LEGS_LOCK(); _waveWanted = true; LEGS_UNLOCK(); }

void Legs::setAttitude(float roll, float pitch, float yaw,
                       float height, float lateral, float forward) {
  // Generous but not absurd limits: past certain angles the kinematics
  // stops finding a solution and the legs go unresponsive.
  LEGS_LOCK();
  _roll     = constrain(roll,     -25.0f, 25.0f);
  _pitch = constrain(pitch, -25.0f, 25.0f);
  _yaw  = constrain(yaw,  -25.0f, 25.0f);
  _height    = constrain(height,    -30.0f, 30.0f);
  _lateral   = constrain(lateral,   -40.0f, 40.0f);
  _fwd     = constrain(forward,     -40.0f, 40.0f);
  LEGS_UNLOCK();
}

void Legs::setStep(uint8_t tipo) {
  LEGS_LOCK();
  _step = tipo;
  LEGS_UNLOCK();
  _trStance = 0;      // cambiando step la stance va rifatta
}

void Legs::setSpeed(float multiplier) {
  if (multiplier < 0.1f) multiplier = 0.1f;
  if (multiplier > 3.0f) multiplier = 3.0f;
  LEGS_LOCK();
  _multiplier = multiplier;
  LEGS_UNLOCK();
}

void Legs::enable() {
  digitalWrite(LEGS_OE, LOW);
  _enabled = true;
}

void Legs::relax() {
  digitalWrite(LEGS_OE, HIGH);
  _enabled = false;
}

// ==================================================
// === MOTION ENGINE (one tick) ===
// ==================================================

void Legs::_tick() {
  if (!_ready) return;

  // 1. Advance the interpolation of the pose in progress.
  if (_seq != nullptr) {
    unsigned long elapsed = millis() - _tPose;
    float t = (_poseTime == 0) ? 1.0f : (float)elapsed / (float)_poseTime;
    float s = smooth(t);

    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 3; j++)
        _siteNow[i][j] = _siteFrom[i][j] + (_siteTo[i][j] - _siteFrom[i][j]) * s;

    if (t >= 1.0f) {
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++) _siteNow[i][j] = _siteTo[i][j];
      _nextPose();
    }
  } else {
    // 2. No pose running: decide what to do now.
    _chooseSequence();
  }

  // 3. Ramp back from calibration: we return to the kinematics over a
  //    second and a half rather than in a single tick.
  if (_calActive) {
    _calBlend = 0.0f;
  } else if (_calBlend < 1.0f) {
    _calBlend += (float)LEGS_TICK_MS / 1500.0f;
    if (_calBlend > 1.0f) _calBlend = 1.0f;
  }

  // 4. Turn the foot positions into pulses for the servos.
  _writeServos();
}

void Legs::_nextPose() {
  _iSeq++;
  const Pose *seq = (const Pose *)_seq;
  if (_iSeq < _nSeq) {
    _applyPose(seq[_iSeq].p, seq[_iSeq].v, seq[_iSeq].attesaMs);
    return;
  }
  // Sequence finished.
  _seq = nullptr;
  _chooseSequence();
}

void Legs::_startSequence(const void *seq, uint8_t n) {
  const Pose *s = (const Pose *)seq;
  _seq = seq;
  _nSeq = n;
  _iSeq = 0;
  _applyPose(s[0].p, s[0].v, s[0].attesaMs);
}

void Legs::_applyPose(const float p[4][3], float speed, float attesaMs) {
  float mult;
  LEGS_LOCK();
  mult = _multiplier;
  LEGS_UNLOCK();

  float maxDistance = 0;
  for (int i = 0; i < 4; i++) {
    float d[3];
    for (int j = 0; j < 3; j++) {
      _siteFrom[i][j] = _siteNow[i][j];

      float target = p[i][j];
      if (target == K) {
        target = _siteNow[i][j];          // asse da non toccare
      } else if (j == 2 && target == z_up) {
        // In the crawl tables z_up is not a fixed height but a marker
        // meaning "foot lifted": the real height comes from the lift
        // parameter, the same one that tunes the trot. Without this the
        // Lift slider had no effect on the crawl at all.
        target = z_default + _trLift;
      }
      _siteTo[i][j] = target;
      d[j] = _siteTo[i][j] - _siteFrom[i][j];
    }
    float dist = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    if (dist > maxDistance) maxDistance = dist;
  }

  // Every leg takes the same time and they all arrive together. The ATMega
  // code instead gave them all the same speed, so the ones with less ground
  // to cover finished early and waited for the others: one of the things
  // that made the step choppy.
  float ms = (maxDistance / (speed * mult)) * 1000.0f;
  // A pose that moves nothing (the four-legged dwell) would last a single
  // tick: the minimum duration is exactly what holds it there longer.
  if (ms < attesaMs) ms = attesaMs;
  _poseTime = (unsigned long)max(ms, (float)LEGS_TICK_MS);
  _tPose = millis();
}

// Decides which sequence to start once the previous one has finished.
void Legs::_chooseSequence() {
  uint8_t wanted;
  uint8_t step;
  bool wave;
  LEGS_LOCK();
  wanted = _gaitWanted;
  step = _step;
  wave = _waveWanted;
  _waveWanted = false;
  LEGS_UNLOCK();

  _gait = wanted;

  // If the trot parameters changed, rebuild them now: between one sequence
  // and the next, never mid-stride.
  bool rebuild;
  LEGS_LOCK();
  rebuild = _trRebuild;
  LEGS_UNLOCK();
  if (rebuild) _buildTrot();

  if (wave) {
    _startSequence(SEQ_WAVE, N_POSES(SEQ_WAVE));
    return;
  }

  // As in the original, the right phase is worked out from where the legs
  // are: that way you can change direction midway without tripping.
  bool leg2Back = fabs(_siteNow[2][1] - y_start) < 1.0f;
  bool leg3Back = fabs(_siteNow[3][1] - y_start) < 1.0f;

  switch (wanted) {
    case GAIT_FORWARD:
      if (step == STEP_TROT) {
        // Before trotting the diagonals must be offset. It cannot be
        // deduced from the y values as for the crawl, because in this
        // stance legs 0 and 1 sit at the same height: an explicit flag is
        // needed.
        if (_trStance != 1) {
          _startSequence(TROT_READY_FWD, 1);
          _trStance = 1;
          _phaseB = true;            // the next swap starts phase A
        } else {
          _phaseB = !_phaseB;
          _startSequence(_phaseB ? TROT_B : TROT_A, N_TROT);
        }
      } else {
        _trStance = 0;
        if (leg2Back) _startSequence(SEQ_FWD_A, N_POSES(SEQ_FWD_A));
        else                _startSequence(SEQ_FWD_B, N_POSES(SEQ_FWD_B));
      }
      break;

    case GAIT_BACK:
      if (step == STEP_TROT) {
        if (_trStance != 2) {
          _startSequence(TROT_READY_BACK, 1);
          _trStance = 2;
          _phaseB = true;
        } else {
          _phaseB = !_phaseB;
          _startSequence(_phaseB ? TROT_BACK_B : TROT_BACK_A, N_TROT);
        }
      } else {
        _trStance = 0;
        if (leg3Back) _startSequence(SEQ_BACK_A, N_POSES(SEQ_BACK_A));
        else                _startSequence(SEQ_BACK_B, N_POSES(SEQ_BACK_B));
      }
      break;

    case GAIT_LEFT:
      _trStance = 0;   // turning undoes the trot stance
      if (leg3Back) _startSequence(SEQ_LEFT_A, N_POSES(SEQ_LEFT_A));
      else                _startSequence(SEQ_LEFT_B, N_POSES(SEQ_LEFT_B));
      break;

    case GAIT_RIGHT:
      _trStance = 0;   // turning undoes the trot stance
      if (leg2Back) _startSequence(SEQ_RIGHT_A, N_POSES(SEQ_RIGHT_A));
      else                _startSequence(SEQ_RIGHT_B, N_POSES(SEQ_RIGHT_B));
      break;

    default:
      break;   // idle: stays put, writing no new poses
  }
}

// ==================================================
// === OUTPUT TO THE SERVOS ===
// ==================================================

// Moves and rotates the BODY while the feet stay put. A foot fixed in the
// world, seen from a body that has moved by t and rotated by R, sits at
// P' = R^T (P - t): subtract the translation, apply the inverse rotation.
// Then back into the leg's local coordinates.
void Legs::_applyAttitude(const float dentro[4][3], float fuori[4][3]) {
  float rolV, becV, imbV, txV, tyV, tzV;
  LEGS_LOCK();
  rolV = SENSE_ROLL   * _roll;   becV = SENSE_PITCH * _pitch;
  imbV = SENSE_YAW * _yaw;
  tzV  = SENSE_HEIGHT  * _height;  txV  = SENSE_LATERAL   * _lateral;
  tyV  = SENSE_FWD   * _fwd;
  LEGS_UNLOCK();

  // Chases the wanted attitude at a bounded rate, with no jerks.
  const float stepA = SPD_ANGLE * LEGS_TICK_MS / 1000.0f;
  const float stepT = SPD_SHIFT  * LEGS_TICK_MS / 1000.0f;
  _rollNow     = approachF(_rollNow,     rolV, stepA);
  _pitchNow = approachF(_pitchNow, becV, stepA);
  _yawNow  = approachF(_yawNow,  imbV, stepA);
  _heightNow    = approachF(_heightNow,    tzV,  stepT);
  _lateralNow   = approachF(_lateralNow,   txV,  stepT);
  _fwdNow     = approachF(_fwdNow,     tyV,  stepT);

  float rol = _rollNow,   bec = _pitchNow, imb = _yawNow;
  float tx  = _lateralNow, ty  = _fwdNow,     tz  = _heightNow;

  // Neutral attitude: just copy, so standing still costs nothing.
  if (rol == 0 && bec == 0 && imb == 0 && tx == 0 && ty == 0 && tz == 0) {
    memcpy(fuori, dentro, sizeof(float) * 12);
    return;
  }

  float cw = cos(-imb * DEG), sw = sin(-imb * DEG);
  float cr = cos(-rol * DEG), sr = sin(-rol * DEG);
  float cb = cos(-bec * DEG), sb = sin(-bec * DEG);

  for (int i = 0; i < 4; i++) {
    // From leg-local to body: add the hip's position.
    float px = SIGN_X[i] * (HX + dentro[i][0]);
    float py = SIGN_Y[i] * (HY + dentro[i][1]);
    float pz = dentro[i][2];

    // Body translation
    px -= tx;  py -= ty;  pz -= tz;

    // Inverse rotation: yaw (z) first, then roll (y), then pitch (x).
    // It is the transpose of Rz*Ry*Rx.
    float x1 = px * cw - py * sw;
    float y1 = px * sw + py * cw;
    float z1 = pz;

    float x2 =  x1 * cr + z1 * sr;
    float y2 =  y1;
    float z2 = -x1 * sr + z1 * cr;

    float x3 = x2;
    float y3 = y2 * cb - z2 * sb;
    float z3 = y2 * sb + z2 * cb;

    // Back to the leg's local coordinates
    fuori[i][0] = SIGN_X[i] * x3 - HX;
    fuori[i][1] = SIGN_Y[i] * y3 - HY;
    fuori[i][2] = z3;
  }
}

void Legs::_writeServos() {
  // The gait produces the foot positions; the attitude moves the body
  // underneath them. Two independent layers: you can tilt the robot while
  // it walks without touching a single line of the gaits.
  float sito[4][3];
  _applyAttitude(_siteNow, sito);

  for (int i = 0; i < 4; i++) {
    float alpha, beta, gamma;
    cartesian_to_polar(alpha, beta, gamma,
                       sito[i][0], sito[i][1], sito[i][2]);
    polar_to_angoli(i, alpha, beta, gamma);

    float ang[3] = { alpha, beta, gamma };
    for (int j = 0; j < 3; j++) {
      float a = ang[j];
      if (isnan(a)) continue;          // posizione irraggiungibile: si salta
      if (REVERSED[i][j]) a = 180.0f - a;
      a = constrain(a, 0.0f, 180.0f);

      int us = US_0 + (int)(a * (US_180 - US_0) / 180.0f) + TRIM_US[i][j];

      // With _calBlend at 0 everyone gets a round 90 degrees; at 1 the
      // kinematics comes out. In between it crosses over smoothly.
      if (_calBlend < 1.0f) {
        const int us90 = (US_0 + US_180) / 2;
        us = us90 + (int)((us - us90) * _calBlend);
      }
      us = constrain(us, US_0 - 100, US_180 + 100);

      uint8_t ch = CHANNEL[i][j];
      // Written only when something changed: standing still, the I2C bus
      // stays free for the PCA and anyone else using it.
      if (_usWritten[ch] != us) {
        _pwm.writeMicroseconds(ch, us);
        _usWritten[ch] = us;
      }
    }
  }
}

// ==================================================
// === CALIBRATION ===
// ==================================================

void Legs::calibrate90() {
  halt();
  _calActive = true;
  Serial.println("LEGS: all servos to 90 degrees, kinematics suspended.");
}

void Legs::endCalibration() {
  if (!_calActive) return;
  _calActive = false;      // la rampa la fa _tick()
  Serial.println("LEGS: coming back from calibration.");
}

// Square stance: every foot at its hip's neutral y. The pose inherited
// from the ATMega code instead starts with two legs at y_start and two at
// y_start + y_step, so one side ends up wider than the other: that is the
// crawl gait's starting stance, not a resting pose.
void Legs::squareStance() {
  static Pose square[1];
  square[0] = { SPD_BODY, {{x_default, y_start, z_default},
                              {x_default, y_start, z_default},
                              {x_default, y_start, z_default},
                              {x_default, y_start, z_default}}, 0 };
  halt();
  endCalibration();
  _trStance = 0;
  _startSequence(square, 1);
}

void Legs::rawAngles(int leg, float alpha, float beta, float gamma) {
  if (leg < 0 || leg > 3) return;
  float ang[3] = { alpha, beta, gamma };
  for (int j = 0; j < 3; j++) {
    float a = constrain(ang[j], 0.0f, 180.0f);
    if (REVERSED[leg][j]) a = 180.0f - a;
    int us = US_0 + (int)(a * (US_180 - US_0) / 180.0f) + TRIM_US[leg][j];
    uint8_t ch = CHANNEL[leg][j];
    _pwm.writeMicroseconds(ch, us);
    _usWritten[ch] = us;
  }
}

void Legs::identifyLeg(int leg) {
  if (leg < 0 || leg > 3 || !_ready) return;
  Serial.printf("LEGS: alzo la leg %d (canali %d, %d, %d)\n",
                leg, CHANNEL[leg][0], CHANNEL[leg][1], CHANNEL[leg][2]);
  float zTmp = _siteNow[leg][2];
  _siteNow[leg][2] = z_up + 20;
  _writeServos();
  delay(700);
  _siteNow[leg][2] = zTmp;
  _writeServos();
  delay(300);
}
