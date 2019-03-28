#pragma once

#include <pathfinder.h>

/*

Four ways to follow a path. Assume all paths are generated to be driven in forward order with front
of the robot.

Front: Should drive with the front of the robot. Front == false means Back
Forward: Should drive from first point to last point of the path. Forward == false means Reverse

Driving with back of robot (make pathfinder think the back is the front):
    Flip headings by 180 degrees
    Multiply encoder values and outputs by -1

Driving with back in reverse order
    Error = end distance - distance traveled

*/

namespace team2655{

enum class PathfinderMode { FrontForward, BackForward, FrontReverse, BackReverse };

namespace pathfindertools{

/**
 * Create a pathfinder encoder follower for a specified configuration
 * @param trajectoryLen The length of the Segment array the follower will be used with
 * @param front Should configure to follow with the front of the robot
 *        If false will configure to drive with the back of the robot
 * @param forward Should configure to follow path from first waypoint to last waypoint. If false
 *                will configure to follow from last point to first point
 * @return The EncoderFollower to be used to follow the Segment array
 */
EncoderFollower createEncoderFollower(int trajectoryLen, PathfinderMode mode);

void trajetorySwapByMode(PathfinderMode mode, Segment **left, Segment **right);

double followEncoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajecotryLength, int encoderTicks, PathfinderMode mode);

// These will drive reverse with back of robot
double pathfinder_follow_encoder_reverse(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick);
double pathfinder_follow_encoder2_reverse(EncoderConfig c, EncoderFollower *follower, Segment s, Segment lastSegment, int trajectory_length, int encoder_tick);

} // namespace pathfindertools
} // namespace team2655

