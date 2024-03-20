"""
A_star 2D
@author: huiming zhou
"""
import sys

import heapq

import search
from roles import Robot, Berth, Boat, Goods

from utils import one_step, find_neighbors

# 机器人相关
robot_num = 10
robot = [Robot(_) for _ in range(robot_num)]
my_robot_order = [a for a in range(robot_num)]  # 自己安排机器人的顺序
need_check = []  # 每次让机器人前进的时候，那些即将陷入碰撞的机器人的坐标，下标表示第几个机器人
success = []  # 每次让机器人走的时候，成功走下一步的机器人的列表，里面是机器人对象
robot_pos = [(0, 0) for _ in range(robot_num)]
instruction_list = []

# 地图相关
n = 200
ch = []
astar = search.AStar()

# 货物相关
N = 210
gds = [[0 for _ in range(N)] for _ in range(N)]
my_gds = []

# 码头相关
berth_num = 10
berth = [Berth() for _ in range(berth_num)]
berth_pos = [(0, 0) for _ in range(berth_num)]
prior_berth = []  # 可以使用的码头的在berth里的下标
remote_birth = []
last_berth = []
MAX_TRANSPORT_TIME = 0
MIN_TRANSPORT_TIME = 0

# 船相关
boat = [Boat() for _ in range(5)]
boat_capacity = 0

money = 0
id = 0
the_last = True
get_gds = 0


def get_berth_without_boat(flag):
    global berth
    global prior_berth
    max_inventory = (-1, 0)
    if flag:
        for i, b in enumerate(berth):
            if b.ship == -1 and b.inventory > max_inventory[1]:
                max_inventory = (i, b.inventory)
    else:
        for i, b in enumerate(berth):
            if b.ship == -1 and b.inventory > max_inventory[1] and i not in prior_berth:
                max_inventory = (i, b.inventory)
    return max_inventory[0]


def set_optiom_berth():
    global berth
    global prior_berth
    global remote_birth
    global MAX_TRANSPORT_TIME
    global MIN_TRANSPORT_TIME
    e = []
    for k, i in enumerate(berth):
        heapq.heappush(e, (i.transport_time, k))
    for j in range(10):
        if j < 5:
            prior_berth.append(heapq.heappop(e)[1])
        else:
            remote_birth.append(heapq.heappop(e)[1])
    remote_birth.reverse()
    MAX_TRANSPORT_TIME = berth[remote_birth[0]].transport_time
    MIN_TRANSPORT_TIME = berth[prior_berth[0]].transport_time


def get_optim_berth(zhenshu):
    global berth
    global remote_birth
    global boat
    most = 0  # 存货量最多的码头在berth中的下标
    m = -1
    for i in range(10):
        if berth[i].inventory > m and berth[i].ship == -1 and zhenshu+2*berth[i].transport_time<=14999:
            idx = remote_birth[prior_berth.index(i)] if i in prior_berth else prior_berth[remote_birth.index(i)]
            if berth[idx].ship==-1 or boat[berth[idx].ship].flag:
                m = berth[i].inventory
                most = i
    return most


def boat_transport(zhenshu):
    global boat
    global berth
    global MAX_TRANSPORT_TIME
    global prior_berth
    global remote_birth
    for i in range(5):
        if boat[i].status == 1 and boat[i].pos != -1:
            p = boat[i].pos
            l0 = berth[p].loading_speed if berth[p].inventory > berth[p].loading_speed else berth[p].inventory
            boat[i].num += l0
            berth[p].inventory -= l0
            if zhenshu + berth[p].transport_time > 14947:
                instruction_list.append("go " + str(i))
            elif berth[p].inventory == 0:
                if boat[i].flag:
                    if zhenshu+ berth[p].transport_time + 2 * berth[prior_berth[remote_birth.index(p)] if p in remote_birth else remote_birth[
                    prior_berth.index(p)]].transport_time<=14997:
                        if boat[i].instruction is None:
                            instruction_list.append("go " + str(i))
                            berth[p].ship = -1
                            boat[i].instruction = "go"
                        else:
                            instruction_list.append("go " + str(i))
                else:
                    if zhenshu+berth[prior_berth[remote_birth.index(p)] if p in remote_birth else remote_birth[
                    prior_berth.index(p)]].transport_time<14450:
                        if boat[i].instruction is None:
                            m = None
                            if p in prior_berth:
                                m = remote_birth[prior_berth.index(p)]
                                instruction_list.append("ship " + str(i) + " " + str(m))
                            else:
                                m = prior_berth[remote_birth.index(p)]
                                instruction_list.append("ship " + str(i) + " " + str(m))
                            boat[i].instruction = ("ship", m)
                            boat[i].flag = True
                            berth[m].ship = i
                            berth[p].ship = -1
                        else:
                            instruction_list.append("ship " + str(i) + " " + str(boat[i].instruction[1]))
            elif boat[i].num >= boat_capacity:
                if boat[i].instruction is None:
                    instruction_list.append("go " + str(i))
                    berth[p].ship = -1
                    boat[i].instruction = "go"
                else:
                    instruction_list.append("go " + str(i))
        elif boat[i].status == 1 and boat[i].pos == -1:
            if boat[i].instruction is None:
                boat[i].num = 0
                boat[i].flag = False
                m = get_optim_berth(zhenshu)
                if m!=-1:
                    instruction_list.append("ship " + str(i) + " " + str(m))
                    berth[m].ship = i
                    boat[i].instruction = ("ship", m)
            else:
                instruction_list.append("ship " + str(i) + " " + str(berth[boat[i].instruction[1]]))
        elif boat[i].status == 0:
            boat[i].instruction = None


def substract_gds_id(g_id):
    for item in robot:
        if item.goods_idx > g_id:
            item.goods_idx -= 1
        elif item.goods_idx == g_id:
            item.cost = 0


def Init():
    global boat_capacity
    global berth
    count = 0
    for i in range(0, n):
        line = input()
        line = list(line)
        for j, k in enumerate(line):
            if k == 'A':
                robot_pos[count] = (i, j)
                count += 1
        ch.append(line)
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
        berth_pos[id] = (berth_list[1], berth_list[2])
    boat_capacity = int(input())
    astar.ch = ch
    set_optiom_berth()
    for i in range(5):
        instruction_list.append("ship " + str(i) + " " + str(prior_berth[i]))
    for i in range(10):
        path = []
        for j in range(10):
            astar.s_start = robot_pos[i]
            astar.s_goal = berth_pos[j]
            path = astar.searching()
            if len(path) > 0:
                break
        if len(path) == 0:
            my_robot_order.pop(i)
    okk = input()
    print("OK")
    sys.stdout.flush()


def Input():
    global boat_capacity
    global astar
    global robot
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        good = Goods(x, y, val, a_star=astar)
        my_gds.append(good)
        gds[x][y] = val
    for i in range(robot_num):
        robot[i].aStar = astar
        a, b, c, d = map(int, input().split())
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = a, b, c, d
        robot_pos[i] = (b, c)
    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()
    for i, item in enumerate(my_gds):
        item.TTL -= 1
        if item.TTL == 0:
            my_gds.pop(i)
            substract_gds_id(i)
    return id


def avoid(robot_id):
    global robot
    one_robot = robot[robot_id]
    x, y = one_robot.x, one_robot.y
    now_idx = one_robot.path.index((x, y))
    newx, newy = one_robot.path[now_idx + 1]
    if (newx, newy) in robot_pos:  # 再次判断是否前面有人挡住，因为此时有可能别人已经让位了
        o = robot_pos.index((newx, newy))  # 占据了自己下一步想走的地方的机器人编号
        another_robot = robot[o]
        k = another_robot.path.index((newx, newy))  # 占据的机器人在那一点上，在它路径上的index
        if len(another_robot.path) > k + 1 and another_robot.path[k + 1] == (x, y):  # 判断两个机器人的方向是否相反,相反才退，不相反就不动
            next_two_position = None
            if len(another_robot.path) > k + 2:  # 判断占据的机器人的路径还剩多长
                next_two_position = another_robot.path[k + 2]  # 找到占据机器人的下一步和下两步的坐标
            motions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
            t = [(x + u[0], y + u[1]) for u in motions]  # 查看目标机器人的四个方向
            last_choice = None
            for i in t:
                if i not in robot_pos and ch[i[0]][i[1]] not in ('#', '*'):  # 判断可以退避的方向
                    if i != next_two_position:  # 如果退避的地方不是占据机器人第二步要走的地方，就往那里退
                        instruction_list.append(
                            "move " + str(robot_id) + " " + str(one_step(one_robot.x, one_robot.y, *i)))
                        one_robot.x, one_robot.y = i
                        robot_pos[robot_id] = i
                        if i not in one_robot.path:
                            one_robot.path.insert(now_idx, i)
                        last_choice = None
                        break
                    else:
                        last_choice = i
            if last_choice is not None:
                instruction_list.append(
                    "move " + str(robot_id) + " " + str(one_step(one_robot.x, one_robot.y, *last_choice)))
                if last_choice not in one_robot.path:
                    one_robot.path.insert(now_idx, last_choice)
                one_robot.x, one_robot.y = last_choice
                one_robot.mbx, one_robot.mby = x, y
                robot_pos[robot_id] = last_choice
    else:
        success_move(one_robot, robot_id, newx, newy, False)


def success_move(one_robot, robot_id, newx, newy, to_success):
    global boat_capacity
    global gds
    global instruction_list
    global robot
    if to_success:
        success.append(robot_id)
    instruction_list.append("move " + str(robot_id) + " " + str(one_step(one_robot.x, one_robot.y, newx, newy)))
    robot_pos[robot_id] = (newx, newy)
    one_robot.x, one_robot.y = newx, newy
    destx, desty = one_robot.path[-1]
    if one_robot.goods == 1:
        if destx <= newx <= destx + 3 and desty <= newy <= desty + 3:
            instruction_list.append("pull " + str(robot_id))
    else:
        if (newx, newy) == (destx, desty) and gds[newx][newy] != 0:
            instruction_list.append("get " + str(robot_id))


def try_to_move(robot_id, zhen):
    global robot
    global berth
    global prior_berth
    global get_gds
    global the_last
    global last_berth
    one_robot = robot[robot_id]
    x, y = one_robot.x, one_robot.y
    destx, desty = one_robot.path[-1] if len(one_robot.path) > 0 else (-1, -1)
    if one_robot.goods == 0:
        if destx <= x <= destx + 3 and desty <= y <= desty + 3 and (destx, desty) in berth_pos:  # pull成功
            one_robot.cost = 1
            one_robot.goods_idx = -999
            berth[one_robot.berth_index].inventory += 1
            # with open("test.txt", "a") as f:
            #    f.writelines(
            #        str((zhen, robot_id, one_robot.berth_index, berth[one_robot.berth_index].inventory)) + "\n")
        elif (x, y) == (destx, desty) and (destx, desty) not in berth_pos and gds[x][y] != 0:  # get失败，重新get
            instruction_list.append("get " + str(robot_id))
        cant_goods = one_robot.choose_goods(my_gds)  # 选一条路，若get失败，则只能选自己脚下，若pull成功或其他情况则可以正常选
        for i in cant_goods:
            j = my_gds.index(i)
            my_gds.remove(i)
            substract_gds_id(j)
    else:
        if (destx, desty) == (x, y) and (destx, desty) not in berth_pos and gds[x][y] != 0:  # get成功，修改路径至港口，把货物移除
            g_id = one_robot.goods_idx
            one_robot.choose_berth(berth,
                                   [h for h in range(10)] if the_last else last_berth)
            my_gds.pop(g_id)
            substract_gds_id(g_id)
            get_gds += gds[x][y]
            gds[x][y] = 0
        elif destx <= x <= destx + 3 and desty <= y <= desty + 3 and (destx, desty) in berth_pos:  # pull失败，重新pull
            instruction_list.append("pull " + str(robot_id))
    destx, desty = one_robot.path[-1] if len(one_robot.path) > 0 else (x, y)
    # with open("path.txt", "a") as f:
    #    f.writelines(str(zhen) + "--" + str(robot_id) + "---" + str(one_robot.path) + "\n\n")
    if len(one_robot.path) > 0 and (destx, desty) != (x, y):  # 走下一步，若get失败不通过if条件，pull失败会让机器人继续在港口里走
        path, good_id = one_robot.path, one_robot.goods_idx
        newx, newy = path[path.index((x, y)) + 1]
        if newx + newy - x - y == 2:
            newx, newy = path[path.index((x, y)) + 2]
        if (newx, newy) not in robot_pos:
            success_move(one_robot, robot_id, newx, newy, True)
        else:
            need_check.append(robot_id)


def conduct_instruction():
    global instruction_list
    for i in instruction_list:
        print(i)
        sys.stdout.flush()
    instruction_list.clear()


def get_str(robot_id):
    global need_check
    global robot_pos
    global robot
    global my_robot_order
    one_robot = robot[robot_id]
    n_idx = one_robot.path.index(robot_pos[robot_id])
    old_x, old_y = None, None
    if n_idx > 0:
        old_x, old_y = one_robot.path[n_idx - 1]
    else:
        return
    if ch[old_x][old_y] not in robot_pos:
        neighbors = find_neighbors(robot_pos, (old_x, old_y))
        for i in neighbors:
            idx = None
            if i != (one_robot.x, one_robot.y):
                idx = robot_pos.index(i)
            if idx in need_check:
                k = robot[idx].path.index(robot_pos[idx])
                if robot[idx].path[k + 1] == (old_x, old_y):
                    my_robot_order.remove(idx)
                    my_robot_order.append(idx)
                    success_move(robot[idx], idx, old_x, old_y, True)
                    need_check.remove(idx)
                    break


def main():
    global need_check
    global instruction_list
    global robot
    global berth
    global MAX_TRANSPORT_TIME
    global MIN_TRANSPORT_TIME
    global prior_berth
    global the_last
    global get_gds
    global last_berth
    Init()
    for zhen in range(1, 15001):
        id = Input()
        boat_transport(id)
        # with open("robot_pos.txt", "a") as f:
        #    e = []
        #    for i in robot:
        #        e.append((zhen, i.x, i.y, round(i.cost, 1),
        #                  len(i.path[i.path.index((i.x, i.y)) if len(i.path) > 0 else 0:]), i.goods_idx, i.goods,
        #                  i.is_avoid))
        #    f.writelines(str(e) + "\n")
        # with open("goods_list.txt", "a") as f:
        #    e = []
        #    for i, m in enumerate(my_gds):
        #        e.append((i, m.x, m.y, m.is_reserved))
        #    f.writelines(str(zhen) + "--" + str(e) + "\n")
        # with open("my_order.txt", "a") as f:
        #    f.writelines(str(zhen) + "--" + str(my_robot_order) + "\n")
        with open("berth.txt", "a") as f:
            e = []
            for i in range(10):
                e.append((i, berth[i].inventory, berth[i].ship, berth[i].loading_speed, berth[i].transport_time))
            f.writelines(str(id) + "--" + str(e) + "\n")
        with open("goods.txt", "a") as f:
            f.writelines(str(id) + "--" + str(get_gds) + "\n")
        e = []
        if id + MIN_TRANSPORT_TIME > 14497 and the_last:
            the_last = False
            for j in range(10):
                if berth[j].ship != -1:
                    e.append(j)
            for i in robot:
                if i.goods == 1:
                    i.choose_berth(berth, e)
            last_berth = e
        for i in my_robot_order:
            try_to_move(i, id)
        if len(success) < len(my_robot_order):
            for i in success:
                get_str(i)
            for i in need_check:
                avoid(i)
        need_check.clear()
        success.clear()
        conduct_instruction()
        print("OK")
        sys.stdout.flush()


if __name__ == '__main__':
    main()
