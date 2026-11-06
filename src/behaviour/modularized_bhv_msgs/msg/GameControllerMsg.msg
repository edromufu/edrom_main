# Disconnected messages contain defaults; check connected before using fields.
bool connected
uint8 packet
uint8 players_per_team
uint8 competition
bool stopped
string phase
string state
string set_play
bool first_half
# -1 means no kicking team.
int16 kicking_team -1
int16 secs_remaining
int16 secondary_time
GameControllerTeam[] teams

# Configured player view. A missing player is not an unpenalized player.
bool team_found
bool player_found
uint8 team_number
uint8 player_number
uint8 score
uint8 opponent_score
bool has_kickoff
bool is_goalkeeper
uint16 message_budget
uint8 penalty
string penalty_name
uint8 secs_till_unpenalised
uint8 cautions
