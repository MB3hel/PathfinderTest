#include <team2655/pftools.hpp>

using namespace team2655;

EncoderFollower pathfindertools::createEncoderFollower(int trajectoryLen, PathfinderMode mode){
    if(mode == PathfinderMode::FrontForward || mode == PathfinderMode::BackForward){
        return {0, 0, 0, 0, 0};
    }else if(mode == PathfinderMode::BackReverse || mode == PathfinderMode::FrontReverse){
        return {0, 0, 0, trajectoryLen - 1, 0};
    }
}

double pathfindertools::followEncoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajecotryLength, int encoderTicks, PathfinderMode mode){
    if(mode == PathfinderMode::FrontForward){
        return pathfinder_follow_encoder(c, follower, trajectory, trajecotryLength, encoderTicks);
    }else if(mode == PathfinderMode::BackReverse){
        return -1 * pathfinder_follow_encoder_reverse(c, follower, trajectory, trajecotryLength, encoderTicks);
    }else if(mode == PathfinderMode::BackForward){
        encoderTicks *= -1;
        return -1 * pathfinder_follow_encoder(c, follower, trajectory, trajecotryLength, encoderTicks);
    }else if(mode == PathfinderMode::FrontReverse){
        encoderTicks *= -1;
        return pathfinder_follow_encoder_reverse(c, follower, trajectory, trajecotryLength, encoderTicks);
    }
}


double pathfindertools::pathfinder_follow_encoder_reverse(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick) {
    int segment = follower->segment;
    if (segment <= 0) {
        follower->finished = 1;
        follower->output = 0.0;
        Segment last = trajectory[0];
        follower->heading = last.heading;
        return 0.0;
    } else {
        return pathfinder_follow_encoder2_reverse(c, follower, trajectory[segment], trajectory[trajectory_length - 1], trajectory_length, encoder_tick);
    }
}

// Follows in reverse order with back of robot
double pathfindertools::pathfinder_follow_encoder2_reverse(EncoderConfig c, EncoderFollower *follower, Segment s, Segment lastSeg, int trajectory_length, int encoder_tick) {
    double distance_covered = ((double)encoder_tick - (double)c.initial_position) /  ((double)c.ticks_per_revolution);
    distance_covered = distance_covered * c.wheel_circumference;
    
    if (follower->segment > 0) {
        follower->finished = 0;
        double error = lastSeg.position - (s.position - distance_covered); // Error is distance from endpoint
        double calculated_value = c.kp * error +
                                  c.kd * ((error - follower->last_error) / s.dt) +
                                  (c.kv * s.velocity + c.ka * s.acceleration);
        
        follower->last_error = error;
        follower->heading = s.heading;
        follower->output = calculated_value;
        follower->segment = follower->segment - 1;
        return calculated_value;
    } else {
        follower->finished = 1;
        return 0.0;
    }
}