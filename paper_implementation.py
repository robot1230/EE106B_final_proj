import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--num_robots', '-nr', type=int, default=3)
parser.add_argument('--N', '-N', type=int, default=0)
args = parser.parse_args()

NUM_ROBOTS = args.num_robots
COM_RANGE = 60#R
GRID_LENGTH = 25#l (not exactly l but corresponds to l)
ROBOT_RAD = GRID_LENGTH*2#r (here it is slightly different from r since plt scaling is weird)
TRANSMIT_FREQ = 100#f_comm
# SPEED = 0.1 #blocks/sec
GRID_SIZE = 100
# LISTENING_TIME = 100/TRANSMIT_FREQ
dt = 1/200000
Q = []
robots = []
msg_locks = {}
ack_locks = {}
request_locks = {}

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def norm(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

# class MsgBuff():
#     def __init__(self):
#         self.m = []
#         self.tthresh = LISTENING_TIME
#     def push(self, msg, lock):
#         lock.acquire()
#         self.m.append(msg)
#         while self.m[-1][0] - self.m[0][0] > self.tthresh:
#             self.m.pop(0)
#         lock.release()
#     def get(self, request_time, lock):
#         lock.acquire()
#         for i in range(len(self.m)):
#             if request_time - self.m[i][0] <= self.tthresh:
#                 ret = self.m[i:]
#                 lock.release()
#                 return ret
#         if len(self.m) > 0:
#             ret = [self.m[-1]]
#             lock.release()
#             return ret
#         else:
#             lock.release()
#             return []

class Robot():
    def __init__(self, x, y, id):
        self.x = GRID_LENGTH*(x + 0.5)
        self.y = GRID_LENGTH*(y + 0.5)
        self.p = [self.x, self.y]
        self.wp = self.p
        self.nextwp = self.wp
        self.id = id
        self.hop = np.inf
        self.delta_t = 50/TRANSMIT_FREQ
        self.qu = Q[np.random.randint(len(Q))]
        # if self.id == 2:
        #     self.T = Q[1]
        # else:
        #     self.T = Q[0]
        self.T = Q[np.random.randint(len(Q))]
        print(self.id, [int(self.T[0]/GRID_LENGTH - 0.5), int(self.T[1]/GRID_LENGTH - 0.5)])
        self.last_check = time.time()
        self.msg = [time.time(), self.p, self.wp, self.nextwp, self.T, self.qu, self.hop]
        self.ack = None
        self.requests = []

    def main_rob(self):
        time.sleep(1)
        t1 = threading.Thread(target=self.broadcast, args=(msg_locks[self.id],))
        t2 = threading.Thread(target=self.goal_manager)
        t1.start()
        t2.start()

        while True:
            surroundings = [[self.wp[0] - GRID_LENGTH, self.wp[1]], [self.wp[0], self.wp[1] - GRID_LENGTH],
                [self.wp[0] + GRID_LENGTH, self.wp[1]], [self.wp[0], self.wp[1] + GRID_LENGTH]]

            #boundary checks
            for i in range(len(surroundings)-1, -1, -1):
                x, y = surroundings[i][0], surroundings[i][1]
                if x < 0 or y < 0 or x > GRID_SIZE or y > GRID_SIZE:
                    surroundings.pop(i)

            wait_flag = 0
            for i in surroundings:
                if manhattan(self.T, i) < manhattan(self.T, self.wp):
                    msg_locks[self.id].acquire()
                    self.nextwp = i
                    msg_locks[self.id].release()
                    break

            rcvmsgs = []
            started_listening = time.time()
            while len(rcvmsgs) == 0 or time.time()-started_listening <= self.delta_t:
                rcvmsgs = []
                request_time = time.time()
                for r in robots:
                    if r.id != self.id and norm(r.p, self.p) <= COM_RANGE:
                        msg_locks[r.id].acquire()
                        rcvmsgs.append(r.msg.copy())
                        msg_locks[r.id].release()

            if len(rcvmsgs) > 0:
                msg_min = min(rcvmsgs, key=lambda x: x[-1])
                msg_locks[self.id].acquire()
                self.hop = 1 + msg_min[-1]
                self.qu = msg_min[-2]
                msg_locks[self.id].release()
                for i in surroundings:
                    if i in Q:
                        occupied = 0
                        for msg in rcvmsgs:
                            if msg[-3] == i:
                                occupied = 1
                                break
                        if occupied == 0:
                            msg_locks[self.id].acquire()
                            self.qu = i
                            self.hop = 0
                            msg_locks[self.id].release()
                            break
                # if self.id == 0:
                #     print(msg_min)
                #     print(self.nextwp)
                for msg in rcvmsgs:
                    if msg[2][0] == self.nextwp[0] and msg[2][1] == self.nextwp[1]:
                        # if self.id == 0:
                            # print(msg[2])
                        wait_flag = 1
                    if msg[3] == self.nextwp:
                        ix, iy = msg[1]
                        if ix > self.x or (ix == self.x and iy > self.y):
                            wait_flag = 1
            # if self.id == 2:
            # print("MAIN", self.id)
                # print(self.T)
                # print(self.p)
            if wait_flag == 0 and time.time() - self.last_check > self.delta_t:
                msg_locks[self.id].acquire()
                self.wp = self.nextwp
                start_moving = time.time()
                ox, oy = self.x, self.y
                while norm(self.p, self.wp) >= 0.1:
                    self.x += dt*(self.wp[0] - ox)
                    self.y += dt*(self.wp[1] - oy)
                    self.p = [self.x, self.y]
                self.p = self.wp
                self.last_check = time.time()
                msg_locks[self.id].release()

        t1.join()
        t2.join()
        return False
    def broadcast(self, lock):
        while True:
            # print("BROADCAST", self.id)
            # lock.acquire()
            self.msg = [time.time(), self.p, self.wp, self.nextwp, self.T, self.qu, self.hop]
            # lock.release()
            # print(self.id, [int(self.wp[0]/GRID_LENGTH - 0.5), int(self.wp[1]/GRID_LENGTH - 0.5)], [int(self.T[0]/GRID_LENGTH - 0.5), int(self.T[1]/GRID_LENGTH - 0.5)], self.requests)
            time.sleep(1/TRANSMIT_FREQ)

    def goal_manager(self):

        def two_way_handshake(r_msg, self_p, self_T):
            #2-way handshake
            rx, ry = r_msg[1]
            waiting_time = 0.3
            successful = False
            new_goal = None
            x, y = self_p
            if rx > x or (rx == x and ry > y):   #act as client
                request_locks[r.id].acquire()
                r.requests.append(self.id)
                request_locks[r.id].release()
                start = time.time()
                while time.time()-start <= waiting_time:
                    ack_locks[self.id].acquire()
                    if self.ack is not None:
                        successful = True
                        new_goal = self.ack
                        self.ack = None
                        ack_locks[self.id].release()
                        break
                    ack_locks[self.id].release()

            else:   #act as server
                start = time.time()
                while time.time()-start <= waiting_time:
                    request_locks[self.id].acquire()
                    if r.id in self.requests:
                        new_goal = r_msg[4].copy()
                        ack_locks[r.id].acquire()
                        r.ack = self_T
                        ack_locks[r.id].release()
                        self.requests = []
                        request_locks[self.id].release()
                        successful = True
                        break

                    request_locks[self.id].release()
            return successful, new_goal

        while True:
            s = time.time()
            # print("GOAL MANAGER", self.id)
            for r in robots:
                # if self.id == 2:
                #     print(r.id)
                if norm(r.p, self.p) <= COM_RANGE:

                    msg_locks[r.id].acquire()

                    rp = r.msg[1].copy()
                    rwp = r.msg[2].copy()
                    rnextwp = r.msg[3].copy()
                    rT = r.msg[4].copy()
                    rqu = r.msg[5].copy()
                    rhop = r.msg[6]
                    r_msg = [r.msg[0], rp, rwp, rnextwp, rT, rqu, rhop]
                    msg_locks[r.id].release()

                    msg_locks[self.id].acquire()
                    p = self.msg[1].copy()
                    wp = self.msg[2].copy()
                    nextwp = self.msg[3].copy()
                    T = self.msg[4].copy()
                    qu = self.msg[5].copy()
                    hop = self.msg[6]

                    msg_locks[self.id].release()

                    if r_msg[4] == T:  #if neighbor has same goal
                        rx, ry = r_msg[1]
                        x, y = p
                        if rx > x or (rx == x and ry > y):   #if neighbor is lexically larger position
                            # print('acquiring', self.id)
                            # msg_locks[self.id].acquire()
                            # print('acquired', self.id)
                            if np.random.random() > 0.1:
                                # msg_locks[self.id].acquire()
                                self.T = qu
                                # msg_locks[self.id].release()
                                self.last_check = time.time()
                            else:
                                # msg_locks[self.id].acquire()
                                self.T = Q[np.random.randint(len(Q))]
                                # msg_locks[self.id].release()
                                self.last_check = time.time()
                            # msg_locks[self.id].release()

                    if manhattan(r_msg[4], wp) + manhattan(r_msg[2], T) < manhattan(T, wp) + manhattan(r_msg[4], r_msg[2]):
                        successful, new_goal = two_way_handshake(r_msg, p, T)

                        if successful:
                            # msg_locks[self.id].acquire()
                            self.T = new_goal
                            # msg_locks[self.id].release()
                            self.last_check = time.time()
                    if manhattan(r_msg[4], wp) + manhattan(r_msg[2], T) == manhattan(T, wp) + manhattan(r_msg[4], r_msg[2]):
                        if np.random.random() < 0.6:
                            successful, new_goal = two_way_handshake(r_msg, p, T)
                            if successful:
                                # msg_locks[self.id].acquire()
                                self.T = new_goal
                                # msg_locks[self.id].release()
                                self.last_check = time.time()
                    # msg_locks[r.id].release()
            # if self.id == 2:
            #     print(time.time()-s)

class Draw():
    def __init__(self, robots, fig):
        self.fig = fig
        self.ax = self.fig.add_subplot(1, 1, 1)
        major_ticks = np.arange(0, GRID_SIZE+1, GRID_LENGTH*4)
        minor_ticks = np.arange(0, GRID_SIZE+1, GRID_LENGTH)

        self.ax.set_xticks(major_ticks)
        self.ax.set_xticks(minor_ticks, minor=True)
        self.ax.set_yticks(major_ticks)
        self.ax.set_yticks(minor_ticks, minor=True)
        self.ax.grid(which='both')
        plt.gca().set_aspect("equal")



        self.plot = {}
        colors = ['blue', 'red', 'green', 'black', 'brown']
        for i in range(len(robots)):
            self.plot[robots[i]], = plt.plot([], [], marker='o', markersize=ROBOT_RAD, c=colors[i%len(colors)])

        self.ani = FuncAnimation(self.fig, self.update, frames=600, fargs=(robots), interval=10,
                    init_func=self.init_func, blit=True)
        plt.show()
        plt.grid()

    def init_func(self):
        self.ax.set_xlim(0, GRID_SIZE)
        self.ax.set_ylim(0, GRID_SIZE)
        for r in list(self.plot.keys()):
            self.plot[r].set_data([], [])
        return self.plot.values()

    def update(self, i, *robots):
        for r in robots:
            x, y = r.p
            self.plot[r].set_data(x, y)
        return self.plot.values()


def main():
    global robots, Q
    fig = plt.figure()
    #converting from index to coordinates
    for i in range(NUM_ROBOTS):
        target = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
        while target in Q:
            target = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
        Q.append(target)
    if args.N == 1:
        Q = [[0,0],[0,1],[0,2],[0,3],[1,2],[2,1],[3,0],[3,1],[3,2],[3,3]]
    # Q.append([0,3])
    # Q.append([10,1])
    # Q.append([3,5])
    print(Q)
    for i in range(len(Q)):
        for j in range(len(Q[0])):
            Q[i][j] = GRID_LENGTH*(Q[i][j]+0.5)

    rob_init_pos = []
    robots = []
    id = 0
    for i in range(NUM_ROBOTS):
        init_pos = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
        while init_pos in rob_init_pos:
            init_pos = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
        robots.append(Robot(init_pos[0], init_pos[1], id))
        rob_init_pos.append(init_pos)
        id+=1
    # r = Robot(0, 0, 0)  #blue - id:0
    # r1 = Robot(1, 0, 1) #red - id:1
    # r2 = Robot(0, 1, 2) #green - id:2
    # r1 = Robot(10, 80, 1)
    # robots = [r, r1, r2]
    for r in robots:
        msg_locks[r.id] = threading.Lock()
        ack_locks[r.id] = threading.Lock()
        request_locks[r.id] = threading.Lock()
    threads = [threading.Thread(target=i.main_rob) for i in robots]

    for thread in threads:
        thread.start()

    d = Draw(robots, fig)

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
