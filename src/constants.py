# Copyright (c) 2010-2011, Duong Dang <mailto:dang.duong@gmail.com>
# This file is part of robot-viewer.

# robot-viewer is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# robot-viewer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with robot-viewer.  If not, see <http://www.gnu.org/licenses/>.
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
