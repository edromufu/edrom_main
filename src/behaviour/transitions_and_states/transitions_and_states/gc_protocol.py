"""Decode GameController v20 (RGme, 158 bytes), as specified in the PDF guide."""
import struct

HEADER = struct.Struct('<4sBBBBBBBBBBhh')
TEAM = struct.Struct('<BBBBBBHH' + 'BBB' * 20)
STATES = ['INITIAL', 'READY', 'SET', 'PLAYING', 'FINISHED']
PHASES = ['NORMAL', 'PENALTY_SHOOT_OUT', 'EXTRA_TIME', 'TIMEOUT']
SET_PLAYS = ['NONE', 'DIRECT_FREE_KICK', 'INDIRECT_FREE_KICK',
             'PENALTY_KICK', 'THROW_IN', 'GOAL_KICK', 'CORNER_KICK']
PENALTIES = ['NONE', 'ILLEGAL_POSITIONING', 'MOTION_IN_SET', 'MOTION_IN_STOP',
             'LOCAL_GAME_STUCK', 'INCAPABLE_ROBOT', 'PICK_UP', 'BALL_HOLDING',
             'LEAVING_THE_FIELD', 'PLAYING_WITH_ARMS_HANDS', 'PUSHING',
             'CAUTIONED', 'SENT_OFF', 'SUBSTITUTE']
COLOURS = ['BLUE', 'RED', 'YELLOW', 'BLACK', 'WHITE', 'GREEN',
           'ORANGE', 'PURPLE', 'BROWN', 'GRAY']


def name(values, index):
    return values[index] if 0 <= index < len(values) else str(index)


def parse(data):
    """Return decoded data, or None for an unsupported/malformed datagram."""
    if len(data) != HEADER.size + 2 * TEAM.size or data[:4] != b'RGme':
        return None
    (_, version, packet, per_team, comp, stopped, phase, state,
     set_play, first_half, kicking, secs, secondary) = HEADER.unpack_from(data)
    if version != 20 or not 1 <= per_team <= 20:
        return None
    teams = []
    for i in range(2):
        v = TEAM.unpack_from(data, HEADER.size + TEAM.size * i)
        players = [dict(number=p + 1, penalty=v[8 + 3*p],
                        penalty_name=name(PENALTIES, v[8 + 3*p]),
                        secs_till_unpenalised=v[9 + 3*p], cautions=v[10 + 3*p])
                   for p in range(per_team)]
        teams.append(dict(team_number=v[0], field_colour=name(COLOURS, v[1]),
                          goalkeeper_colour=name(COLOURS, v[2]), goalkeeper=v[3],
                          score=v[4], penalty_shot=v[5], single_shots=v[6],
                          message_budget=v[7], players=players))
    return dict(connected=True, packet=packet, players_per_team=per_team,
                competition=comp, stopped=bool(stopped), phase=name(PHASES, phase),
                state=name(STATES, state), set_play=name(SET_PLAYS, set_play),
                first_half=bool(first_half), kicking_team=None if kicking == 255 else kicking,
                secs_remaining=secs, secondary_time=secondary, teams=teams)


def add_me(state, team_num, player_num):
    """Attach the configured team's/player's view, matching the PDF schema."""
    own = next((t for t in state['teams'] if t['team_number'] == team_num), None)
    state['me'] = None
    if own is None:
        return
    opponent = next(t for t in state['teams'] if t is not own)
    me = dict(team_number=team_num, player_number=player_num, score=own['score'],
              opponent_score=opponent['score'], has_kickoff=state['kicking_team'] == team_num,
              is_goalkeeper=own['goalkeeper'] == player_num,
              message_budget=own['message_budget'])
    if 1 <= player_num <= len(own['players']):
        me.update({k: v for k, v in own['players'][player_num - 1].items() if k != 'number'})
    state['me'] = me
