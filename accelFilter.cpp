//=============================================================================
// File: accelFilter.cpp
//
// COPYRIGHT 2012 Robotics Alliance of the West Coast(Cow)
// All rights reserved.  Cow proprietary and confidential.
//             
// The party receiving this software directly from Cow (the "Recipient")
// may use this software and make copies thereof as reasonably necessary solely
// for the purposes set forth in the agreement between the Recipient and
// Cow(the "Agreement").  The software may be used in source code form
// solely by the Recipient's employees/volunteers.  The Recipient shall have 
// no right to sublicense, assign, transfer or otherwise provide the source
// code to any third party. Subject to the terms and conditions set forth in
// the Agreement, this software, in binary form only, may be distributed by
// the Recipient to its users. Cow retains all ownership rights in and to
// the software.
//
// This notice shall supercede any other notices contained within the software.
//=============================================================================


#include "accelFilter.h"
int32_t max_v = 18;
int32_t max_a = 5;

int32_t sd(int32_t v, int32_t a)
{
	if (v < 0) {
		v = -v;
		return -(v + v % a) * (v - v % a + a) / (2 * a);
	} else {
		return (v + v % a) * (v - v % a + a) / (2 * a);
	}
}

int32_t get_sign(int32_t i)
{
	if (i > 0) {
		return 1;
	} else if (i < 0) {
		return -1;
	} else {
		return 0;
	}
}

int32_t get_acceleration (int32_t distance_to_target, int32_t v, int32_t goal_v)
{
	int32_t sign = get_sign(distance_to_target);
	int32_t gooda;
        // Compute our maximum acceleration
	if (sign * v + max_a > max_v) {
		gooda = sign * max_v - v ;
	} else {
		gooda = sign * max_a;
	}
        // Loop while accelerating that way would throw us too far
	while (sign * (v + sd(v + gooda, max_a) - sd(goal_v, max_a) - distance_to_target) > 0) {
		gooda = gooda - sign;
	} 
	return gooda;
}
// To use this, do
// v = v + get_acceleration(goal - pos, v, goal_v)
// pos = pos + v
