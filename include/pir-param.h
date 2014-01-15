/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PIRANHA_PARAM_H
#define PIRANHA_PARAM_H

// TODO: subtract inseration depth of tool changer
#define LWA4_FT_L_fudge -1.5e-2 /* measured on physical arm */
#define LWA4_FT_L (82.2e-3 + LWA4_FT_L_fudge) ///< Length of F/T Sensor
#define LWA4_FT_ANGLE (45*M_PI/180)


#define LWA4_L_P_fudge (-.5e-2)   /* offset, measured with arm */
#define LWA4_L_P (0.05 + LWA4_L_P_fudge)   /* pedestal height, measured */
#define LWA4_L_0 (0.3000) /* pedestal base to shoulder */
#define LWA4_L_1 (0.6280 - LWA4_L_0) /* shoulder to elbow */
#define LWA4_L_2 (0.9510 - LWA4_L_1 - LWA4_L_0) /* elbow to wrist */
#define LWA4_L_e (1.0334 - LWA4_L_2 - LWA4_L_1 - LWA4_L_0) /* wrist to E.E end of powerball */

#define PIR_L_SHOULDER_WIDTH (6.35e-2) /* width of shoulder mount middle section */


// TODO: Adjust for new tool changer

#define SDH_LB 98e-3      ///< Base connector to finger joint
#define SDH_L1 0.0865     ///< Lower finger joint
#define SDH_L2 0.0675     ///< Upper finger joint
#define SDH_B 66e-3       ///< distance between fingers
#define SDH_TC 38.105e-3  ///< Thumb to center point
#define SDH_FC 19.053e-3  ///< Fingers line to center point



#define SDH_L_FINGER (2.785e-2) ///< width of finger
#define SDH_L_K1 (3.18e-2)      ///< upper knuckle
#define SDH_L0M (SDH_L_FINGER/2 + 0.5562e-2) ///< finger middle to lower knuckle, y-
#define SDH_L0P (SDH_L_FINGER/2 + 1.251e-2) ///< finger middle to lower knuckle, y+
#define SDH_L1M (SDH_L_FINGER/2 + 0) ///< finger middle to upper knuckle, y-
#define SDH_L1P (SDH_L_K1 - SDH_L_FINGER/2)  ///< finger middle to upper knuckle, y+

#define SDH_MASS 1.955                    ///< SDH Mass (kilograms)
#define SDH_WEIGHT (SDH_MASS*AA_K_STD_G)  ///< SDH Weight (Newton)

#endif //PIRANHA_PARAM_H
