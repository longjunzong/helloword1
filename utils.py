import queue


def one_step(startx, starty, goalx, goaly):
    if startx == goalx:
        if goaly > starty:
            return 0
        elif goaly<starty:
            return 1
    elif starty==goaly:
        if goalx > startx:
            return 3
        elif goalx<startx:
            return 2


def encode_direction(startx, starty, goalx, goaly):
    """
    从start走到goal，对应的退避编码
    :param startx:
    :param starty:
    :param goalx:
    :param goaly:
    :return:
    """
    if startx == goalx:
        if goaly > starty:
            return (1, 0, 0, -1)
        else:
            return (1, 0, 0, 1)
    else:
        if goalx > startx:
            return (0, 1, -1, 0)
        else:
            return (0, 1, 1, 0)


def find_neighbors(robot_pos, start):
    motions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    neighbors = [(start[0] + u[0], start[1] + u[1]) for u in motions]
    t = []
    for i in neighbors:
        if i in robot_pos:
            t.append(i)
    return t
