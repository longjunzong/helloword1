import math
import heapq
import numpy as np


class AStar:
    """AStar set the cost + heuristics as the priority
    """

    def __init__(self, s_start=(0,0), s_goal=(0,0), heuristic_type="manhattan", ch=None, obs_type=None):
        if obs_type is None:
            obs_type = ['#', '*']
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.x_range = 200  # size of background
        self.y_range = 200
        self.motions = [(-1, 0), (0, 1),
                        (1, 0), (0, -1)]
        self.obs_type = obs_type
        self.ch = ch
        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """
        if self.ch[self.s_goal[0]][self.s_goal[1]] in self.obs_type:
            return []
        self.PARENT.clear()
        self.PARENT[self.s_start] = self.s_start
        self.g.clear()
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        self.OPEN.clear()
        heapq.heappush(self.OPEN,  # 一个小顶堆，param1: 小顶堆的list，param2: 入堆元素(A,(B,C))，如果要去点(B,C),则最终到终点的代价是A
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)  # 从起点到s的代价 + 从s到s_n的代价，若s_n在障碍物中，则代价为无穷大

                if s_n not in self.g:  # 如果s_n不在g中，则初始化为无穷大
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # 修改g中的代价，并把s_n的父节点设为s，然后若s_n是可达点则入堆
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT)

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.motions]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion，返回从s_start到s_goal的曼哈顿距离
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """
        if s_start[0] < 0 or s_start[1] < 0 or s_goal[0] < 0 or s_goal[1] < 0:
            return math.inf

        if s_start[0] >= self.x_range or s_start[1] >= self.y_range or s_goal[0] >= self.x_range or s_goal[1] >= self.y_range:
            return math.inf

        if self.ch[s_goal[0]][s_goal[1]] in self.obs_type:
            return math.inf

        return abs(s_goal[0] - s_start[0])+abs(s_goal[1] - s_start[1])


    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """
        return self.ch[s_end[0]][s_end[1]] in self.obs_type



    def f_value(self, s):
        """
        f = g + h. (g: 从起点到点s的代价, h: 从终点到点s的启发式代价，其实就是曼哈顿距离或欧几里得距离） f就是总代价
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal
        while True:
            try:
                s = PARENT[s]
            except KeyError :
                return []
            path.append(s)
            if s == self.s_start:
                break
        return list(path)

    def heuristic(self, s):
        """
        计算终点到中间点s的曼哈顿距离或欧几里得距离
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])
