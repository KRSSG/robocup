syntax = "proto2";

import "ssl_game_event.proto";
import "ssl_game_event_2019.proto";

// Each UDP packet contains one of these messages.
message Referee {
    // The UNIX timestamp when the packet was sent, in microseconds.
    // Divide by 1,000,000 to get a time_t.
    required uint64 packet_timestamp = 1;

    // These are the "coarse" stages of the game.
    enum Stage {
        // The first half is about to start.
        // A kickoff is called within this stage.
        // This stage ends with the NORMAL_START.
        NORMAL_FIRST_HALF_PRE = 0;
        // The first half of the normal game, before half time.
        NORMAL_FIRST_HALF = 1;
        // Half time between first and second halves.
        NORMAL_HALF_TIME = 2;
        // The second half is about to start.
        // A kickoff is called within this stage.
        // This stage ends with the NORMAL_START.
        NORMAL_SECOND_HALF_PRE = 3;
        // The second half of the normal game, after half time.
        NORMAL_SECOND_HALF = 4;
        // The break before extra time.
        EXTRA_TIME_BREAK = 5;
        // The first half of extra time is about to start.
        // A kickoff is called within this stage.
        // This stage ends with the NORMAL_START.
        EXTRA_FIRST_HALF_PRE = 6;
        // The first half of extra time.
        EXTRA_FIRST_HALF = 7;
        // Half time between first and second extra halves.
        EXTRA_HALF_TIME = 8;
        // The second half of extra time is about to start.
        // A kickoff is called within this stage.
        // This stage ends with the NORMAL_START.
        EXTRA_SECOND_HALF_PRE = 9;
        // The second half of extra time.
        EXTRA_SECOND_HALF = 10;
        // The break before penalty shootout.
        PENALTY_SHOOTOUT_BREAK = 11;
        // The penalty shootout.
        PENALTY_SHOOTOUT = 12;
        // The game is over.
        POST_GAME = 13;
    }
    required Stage stage = 2;

    // The number of microseconds left in the stage.
    // The following stages have this value; the rest do not:
    // NORMAL_FIRST_HALF
    // NORMAL_HALF_TIME
    // NORMAL_SECOND_HALF
    // EXTRA_TIME_BREAK
    // EXTRA_FIRST_HALF
    // EXTRA_HALF_TIME
    // EXTRA_SECOND_HALF
    // PENALTY_SHOOTOUT_BREAK
    //
    // If the stage runs over its specified time, this value
    // becomes negative.
    optional sint32 stage_time_left = 3;

    // These are the "fine" states of play on the field.
    enum Command {
        // All robots should completely stop moving.
        HALT = 0;
        // Robots must keep 50 cm from the ball.
        STOP = 1;
        // A prepared kickoff or penalty may now be taken.
        NORMAL_START = 2;
        // The ball is dropped and free for either team.
        FORCE_START = 3;
        // The yellow team may move into kickoff position.
        PREPARE_KICKOFF_YELLOW = 4;
        // The blue team may move into kickoff position.
        PREPARE_KICKOFF_BLUE = 5;
        // The yellow team may move into penalty position.
        PREPARE_PENALTY_YELLOW = 6;
        // The blue team may move into penalty position.
        PREPARE_PENALTY_BLUE = 7;
        // The yellow team may take a direct free kick.
        DIRECT_FREE_YELLOW = 8;
        // The blue team may take a direct free kick.
        DIRECT_FREE_BLUE = 9;
        // The yellow team may take an indirect free kick.
        INDIRECT_FREE_YELLOW = 10;
        // The blue team may take an indirect free kick.
        INDIRECT_FREE_BLUE = 11;
        // The yellow team is currently in a timeout.
        TIMEOUT_YELLOW = 12;
        // The blue team is currently in a timeout.
        TIMEOUT_BLUE = 13;
        // The yellow team just scored a goal.
        // For information only.
        // For rules compliance, teams must treat as STOP.
        // Deprecated: Use the score field from the team infos instead. That way, you can also detect revoked goals.
        GOAL_YELLOW = 14 [deprecated = true];
        // The blue team just scored a goal. See also GOAL_YELLOW.
        GOAL_BLUE = 15 [deprecated = true];
        // Equivalent to STOP, but the yellow team must pick up the ball and
        // drop it in the Designated Position.
        BALL_PLACEMENT_YELLOW = 16;
        // Equivalent to STOP, but the blue team must pick up the ball and drop
        // it in the Designated Position.
        BALL_PLACEMENT_BLUE = 17;
    }
    required Command command = 4;

    // The number of commands issued since startup (mod 2^32).
    required uint32 command_counter = 5;

    // The UNIX timestamp when the command was issued, in microseconds.
    // This value changes only when a new command is issued, not on each packet.
    required uint64 command_timestamp = 6;

    // Information about a single team.
    message TeamInfo {
        // The team's name (empty string if operator has not typed anything).
        required string name = 1;
        // The number of goals scored by the team during normal play and overtime.
        required uint32 score = 2;
        // The number of red cards issued to the team since the beginning of the game.
        required uint32 red_cards = 3;
        // The amount of time (in microseconds) left on each yellow card issued to the team.
        // If no yellow cards are issued, this array has no elements.
        // Otherwise, times are ordered from smallest to largest.
        repeated uint32 yellow_card_times = 4 [packed = true];
        // The total number of yellow cards ever issued to the team.
        required uint32 yellow_cards = 5;
        // The number of timeouts this team can still call.
        // If in a timeout right now, that timeout is excluded.
        required uint32 timeouts = 6;
        // The number of microseconds of timeout this team can use.
        required uint32 timeout_time = 7;
        // The pattern number of this team's goalkeeper.
        required uint32 goalkeeper = 8;
        // The total number of countable fouls that act towards yellow cards
        optional uint32 foul_counter = 9;
        // The number of consecutive ball placement failures of this team
        optional uint32 ball_placement_failures = 10;
        // Indicate if the team is able and allowed to place the ball
        optional bool can_place_ball = 12;
        // The maximum number of bots allowed on the field based on division and cards
        optional uint32 max_allowed_bots = 13;
        // The team has submitted an intent to substitute one or more robots at the next chance
        optional bool bot_substitution_intent = 14;
    }

    // Information about the two teams.
    required TeamInfo yellow = 7;
    required TeamInfo blue = 8;

    // The coordinates of the Designated Position. These are measured in
    // millimetres and correspond to SSL-Vision coordinates. These fields are
    // always either both present (in the case of a ball placement command) or
    // both absent (in the case of any other command).
    message Point {
        required float x = 1;
        required float y = 2;
    }
    optional Point designated_position = 9;

    // Information about the direction of play.
    // True, if the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system.
    // Obviously, the yellow team will play on the opposite half.
    optional bool blue_team_on_positive_half = 10;

    // The game event that caused the referee command.
    // deprecated in favor of game_events.
    optional Game_Event game_event = 11 [deprecated = true];

    // The command that will be issued after the current stoppage and ball placement to continue the game.
    optional Command next_command = 12;

    // All game events that were detected since the last RUNNING state.
    // Will be cleared as soon as the game is continued.
    repeated GameEvent game_events = 13;

    // All non-finished proposed game events that may be processed next.
    repeated ProposedGameEvent proposed_game_events = 14;

    // The time in microseconds that is remaining until the current action times out
    // The time will not be reset. It can get negative.
    // An autoRef would raise an appropriate event, if the time gets negative.
    // Possible actions where this time is relevant:
    //  * free kicks
    //  * kickoff, penalty kick, force start
    //  * ball placement
    optional int32 current_action_time_remaining = 15;
}

message ProposedGameEvent {
    // The UNIX timestamp when the game event proposal will time out, in microseconds.
    required uint64 valid_until = 1;
    // The identifier of the proposer.
    required string proposer_id = 2;
    // The proposed game event.
    required GameEvent game_event = 3;
}
