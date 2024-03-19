import heapq
import math


class Robot:
    def __init__(self, startX=0, startY=0, goods=0, status=0, aStar=None):
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.cost = 1
        self.path = []
        self.goods_idx = -999
        self.aStar = aStar
        self.is_avoid = False
        self.get_goods = False

    def choose_goods(self, goods_list):
        self.aStar.s_start = (self.x, self.y)
        value_estimate = []
        cant_get=[]
        for i, good in enumerate(goods_list):
            h = abs(good.x - self.x) + abs(good.y - self.y) + 1
            value = -good.val / h
            if good.TTL > h + 20:
                if not good.is_reserved:
                    heapq.heappush(value_estimate, (value, i, good.x, good.y))
        if len(value_estimate) > 0:
            path =[]
            while len(path)==0 and len(value_estimate)>0:
                optim = heapq.heappop(value_estimate)
                if optim[0] < self.cost-1:
                    self.aStar.s_goal = (optim[-2], optim[-1])
                    path = self.aStar.searching()
                    if len(path)>0:
                        goods_list[optim[1]].is_reserved = True
                        path.reverse()
                        if self.goods == 0 and self.goods_idx != -999:
                            goods_list[self.goods_idx].is_reserved = False
                        self.cost, self.path, self.goods_idx = optim[0], path, optim[1]
                    else:
                        cant_get.append(goods_list[optim[1]])
        return cant_get


class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
        self.inventory = 0
        self.ship = -1


class Boat:
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos
        self.status = status
        self.instruction = None


class Goods:
    def __init__(self, x=0, y=0, val=0, ttl=1000, a_star=None, is_transport=False):
        self.x = x
        self.y = y
        self.val = val
        self.TTL = ttl
        self.aStar = a_star
        self.is_transport = is_transport
        self.is_reserved = False
        self.prior_berth = None  # [到码头的路径长度，第几个码头，码头路径]

    def choose_berth(self, berth_list, volecity):
        self.aStar.s_start = (self.x, self.y)
        distance_estimate = []
        for berth,i in berth_list:
            value = (abs(berth.x - self.x) + abs(berth.y - self.y))
            heapq.heappush(distance_estimate, (value, i, berth.x, berth.y))
        optim = heapq.heappop(distance_estimate)
        self.aStar.s_goal = (optim[-2], optim[-1])
        path = self.aStar.searching()
        path.reverse()
        self.prior_berth = [optim[1], path]
        return optim[1], path


    def get_berth(self):
        return self.prior_berth
