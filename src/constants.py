import math

hrp2_14_half_sitting_freeflyer = [ 0, 0, 0.6487, 0, 0, 0 ]

hrp2_14_half_sitting_q = [ 0, 0, -26, 50, -24, 0,
                           0, 0, -26, 50, -24, 0,
                           0, 0,
                           0, 0,
                           15, -10, 0, -30, 0, 0, 10,
                           15,  10, 0, -30, 0, 0, 10,
                          -10.0, 10.0, -10.0, 10.0, -10.0,
                          -10.0, 10.0, -10.0, 10.0, -10.0 ]

hrp2_hs = hrp2_14_half_sitting_freeflyer + [e*math.pi/180.0 for e in hrp2_14_half_sitting_q]
