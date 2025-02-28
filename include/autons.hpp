#pragma once

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

int sgn(double num);

void moveMax(double dist, int timeout, double slowAt);

void set_drive(double inches, int time = 3000, float minSpeed = 0, float maxSpeed=127);

void set_drive_advanced(double inches, int time, float minSpeed, float maxSpeed, float earlyExitRange);

void disruptRingRushBlue();

void MogoSideSoloAWP(bool isBlue);

void disruptRingRush(bool isBlue);

void MogoSideSoloRed();

void MogoSideSoloBlue();

void verySimpleMogo(bool isBlue);

void simpleRing(bool isBlue);

void simpleMogo(bool isBlue);

void newMogoRush(bool isBlue);

void testAuton();

void stateSoloAwp(bool isBlue);

void safeFourRing(bool isBlue);

void safeRingSide(bool isBlue);
void RingRush6(bool isBlue);
void positiveSideQuals(bool isBlue);

// ASSET(ringRushBlue_txt);
// ASSET(ringRushBlueOld_txt);
// ASSET(ringRushRed_txt);
// ASSET(ringRushRedOld_txt);