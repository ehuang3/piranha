!! -*- mode: F90; -*-
!!
!! Copyright (c) 2013, Georgia Tech Research Corporation
!! All rights reserved.
!!
!! Author(s): Neil T. Dantam <ntd@gatech.edu>
!! Georgia Tech Humanoid Robotics Lab
!! Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
!!
!!
!! This file is provided under the following "BSD-style" License:
!!
!!
!!   Redistribution and use in source and binary forms, with or
!!   without modification, are permitted provided that the following
!!   conditions are met:
!!
!!   * Redistributions of source code must retain the above copyright
!!     notice, this list of conditions and the following disclaimer.
!!
!!   * Redistributions in binary form must reproduce the above
!!     copyright notice, this list of conditions and the following
!!     disclaimer in the documentation and/or other materials provided
!!     with the distribution.
!!
!!   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
!!   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
!!   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
!!   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
!!   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
!!   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
!!   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
!!   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
!!   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
!!   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
!!   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
!!   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
!!   POSSIBILITY OF SUCH DAMAGE.


module lwa4

  use ISO_C_BINDING
  use amino_la
  use amino_tf
  use reflex_kin
  implicit none

  real(C_DOUBLE), parameter :: LWA4_L0 = 0.3
  real(C_DOUBLE), parameter :: LWA4_L1 = 0.628 - LWA4_L0
  real(C_DOUBLE), parameter :: LWA4_L2 = 0.951 - LWA4_L1 - LWA4_L0
  real(C_DOUBLE), parameter :: LWA4_LE = 1.0334 - LWA4_L2 - LWA4_L1 - LWA4_L0

contains

  pure subroutine lwa4_kin2( q, TT_rel ) &
       bind( C, name="lwa4_tf_rel" )
    real(C_DOUBLE), intent(in) :: q(7)
    real(C_DOUBLE), intent(out) :: TT_rel(3,4,7)

    real(C_DOUBLE) :: TT_q(3,4,7)

    call aa_tf_xangle2rotmat( -q(1), TT_q(:,:,1) )
    call aa_tf_yangle2rotmat( -q(2), TT_q(:,:,2) )
    call aa_tf_xangle2rotmat( -q(3), TT_q(:,:,3) )
    call aa_tf_yangle2rotmat( -q(4), TT_q(:,:,4) )
    call aa_tf_xangle2rotmat( -q(4), TT_q(:,:,4) )

  end subroutine lwa4_tf_rel

end module lwa4
