import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--num_robots', '-nr', type=int, default=3)
parser.add_argument('--shape', '-sh', type=str, default='any')
args = parser.parse_args()

GRID_SIZE = 100
GRID_LENGTH = 20#l (not exactly l but corresponds to l)
ROBOT_RAD = GRID_LENGTH/(3*np.sqrt(2))#r
COM_RANGE = GRID_LENGTH*2.5#R
# ROBOT_RAD = GRID_LENGTH*2#r (here it is slightly different from r since plt scaling is weird)
TRANSMIT_FREQ = 200#f_comm
# SPEED = 0.1 #blocks/sec
# LISTENING_TIME = 100/TRANSMIT_FREQ
dt = 1/200000
if args.shape == 'n' or args.shape == 'u':
    NUM_ROBOTS = 2*(GRID_SIZE//GRID_LENGTH) + (GRID_SIZE//GRID_LENGTH)-2
else:
    NUM_ROBOTS = args.num_robots

Q = []
robots = []
msg_locks = {}
ack_locks = {}
request_locks = {}
run_dur = 150
done = False

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
        self.angle = np.random.randint(4)*90
        self.p = [self.x, self.y]
        self.wp = self.p
        self.nextwp = self.wp
        self.id = id
        self.hop = np.inf
        self.delta_t = 180/TRANSMIT_FREQ
        self.qu = Q[np.random.randint(len(Q))]
        # if self.id == 2:
        #     self.T = Q[1]
        # else:
        #     self.T = Q[0]
        self.T = Q[np.random.randint(len(Q))]
        print(self.id, [int(self.p[0]/GRID_LENGTH - 0.5), int(self.p[1]/GRID_LENGTH - 0.5)], [int(self.T[0]/GRID_LENGTH - 0.5), int(self.T[1]/GRID_LENGTH - 0.5)])
        self.last_check = time.time()
        self.msg = [time.time(), self.p, self.wp, self.nextwp, self.T, self.qu, self.hop]
        self.ack = None
        self.requests = []
        self.total_dist_traveled = 0
        self.done = False

    def main_rob(self):
        time.sleep(1)
        # t1 = threading.Thread(target=self.broadcast, args=(msg_locks[self.id],))
        t1 = threading.Thread(target=self.broadcast)
        t2 = threading.Thread(target=self.goal_manager)
        self.start = time.time()
        t1.start()
        t2.start()

        # while time.time()-self.start <= run_dur:
        while not done:
            # print(self.id, manhattan(self.wp, self.T), self.done, done)
            if manhattan(self.wp, self.T) < 1:
                self.done = True
            else:
                self.done = False
            # print(self.p, self.x, self.y)
            # print(self.hop)
            surroundings = [[self.wp[0], self.wp[1] + GRID_LENGTH], [self.wp[0] - GRID_LENGTH, self.wp[1]],
                [self.wp[0], self.wp[1] - GRID_LENGTH], [self.wp[0] + GRID_LENGTH, self.wp[1]]]
            idxs = [0,1,2,3]

            #boundary checks
            for i in range(len(surroundings)-1, -1, -1):
                x, y = surroundings[i][0], surroundings[i][1]
                if x < 0 or y < 0 or x > GRID_SIZE or y > GRID_SIZE:
                    surroundings.pop(i)
                    idxs.pop(i)

            wait_flag = 0
            next_angle = self.angle
            for i in range(len(surroundings)):
                if manhattan(self.T, surroundings[i]) < manhattan(self.T, self.wp):
                    # msg_locks[self.id].acquire()
                    self.nextwp = surroundings[i]
                    # msg_locks[self.id].release()
                    next_angle = idxs[i]*90
                    break
            # print(self.p, self.wp, self.nextwp)
            rcvmsgs = []
            started_listening = time.time()
            while time.time()-started_listening <= self.delta_t:
                rcvmsgs = []
                # request_time = time.time()
                for r in robots:
                    if r.id != self.id and norm(r.p, self.p) <= COM_RANGE:
                        msg_locks[r.id].acquire()
                        rcvmsgs.append(r.msg)
                        msg_locks[r.id].release()
            # print(self.id, len(rcvmsgs))
            if len(rcvmsgs) > 0:
                msg_min = min(rcvmsgs, key=lambda x: x[-1])
                # msg_locks[self.id].acquire()
                self.hop = 1 + msg_min[-1]
                self.qu = msg_min[-2]
                # msg_locks[self.id].release()
                for i in surroundings:
                    if i in Q:
                        occupied = 0
                        for msg in rcvmsgs:
                            if msg[-3] == i:
                                occupied = 1
                                break
                        if occupied == 0:
                            # msg_locks[self.id].acquire()
                            self.qu = i
                            self.hop = 0
                            # msg_locks[self.id].release()
                            break
                # if self.id == 0:
                #     print(msg_min)
                #     print(self.nextwp)
                # print(self.x, self.y)
                for msg in rcvmsgs:
                    if msg[2][0] == self.nextwp[0] and msg[2][1] == self.nextwp[1]:
                        # if self.id == 0:
                            # print(msg[2])
                        wait_flag = 1
                    if msg[3][0] == self.nextwp[0] and msg[3][1] == self.nextwp[1]:
                        ix, iy = msg[1]
                        if ix > self.x or (ix == self.x and iy > self.y):
                            wait_flag = 1
            # if self.id == 2:
            # print("MAIN", self.id)
                # print(self.T)
                # print(self.p)
            # if len()
            # print(self.id, self.wp, rcvmsgs[0])
            if wait_flag == 0 and time.time() - self.last_check > self.delta_t:
                # msg_locks[self.id].acquire()
                self.wp = self.nextwp
                start_moving = time.time()
                ox, oy = self.x, self.y
                oang = self.angle
                dir = 1 if next_angle - self.angle <= 180 else -1
                # print('yea')
                # print(self.angle, next_angle, self.p, self.id)
                # self.angle = next_angle

                # while abs(self.angle%360 - next_angle) >= 0.1:
                #     self.angle += dt*dir*30
                # self.angle = next_angle

                while norm(self.p, self.wp) >= 0.1:
                    self.x += dt*(self.wp[0] - ox)
                    self.y += dt*(self.wp[1] - oy)
                    self.p = [self.x, self.y]
                self.p = self.wp
                self.x, self.y = self.p
                self.total_dist_traveled += GRID_LENGTH
                self.last_check = time.time()
                # msg_locks[self.id].release()

        t1.join()
        t2.join()
        return False

    # def broadcast(self, lock):
    def broadcast(self):
        # while time.time()-self.start <= run_dur:
        while not done:
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

        # while time.time()-self.start <= run_dur:
        while not done:
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
                                msg_locks[self.id].acquire()
                                self.T = qu
                                msg_locks[self.id].release()
                                self.last_check = time.time()
                            else:
                                msg_locks[self.id].acquire()
                                self.T = Q[np.random.randint(len(Q))]
                                msg_locks[self.id].release()
                                self.last_check = time.time()
                            # msg_locks[self.id].release()

                    if manhattan(r_msg[4], wp) + manhattan(r_msg[2], T) < manhattan(T, wp) + manhattan(r_msg[4], r_msg[2]):
                        successful, new_goal = two_way_handshake(r_msg, p, T)

                        if successful:
                            msg_locks[self.id].acquire()
                            self.T = new_goal
                            msg_locks[self.id].release()
                            self.last_check = time.time()
                    if manhattan(r_msg[4], wp) + manhattan(r_msg[2], T) == manhattan(T, wp) + manhattan(r_msg[4], r_msg[2]):
                        if np.random.random() < 0.1:
                            successful, new_goal = two_way_handshake(r_msg, p, T)
                            if successful:
                                msg_locks[self.id].acquire()
                                self.T = new_goal
                                msg_locks[self.id].release()
                                self.last_check = time.time()
                    # msg_locks[r.id].release()
            # if self.id == 2:
            #     print(time.time()-s)

class Draw():
    # def __init__(self, robots, fig):
    def __init__(self, data, fig):
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


        self.data = data
        self.robs = {}
        colors = ['blue', 'red', 'green', 'black', 'brown']
        # self.title = self.ax.text(0.5,0.85, "", bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
        #         transform=self.ax.transAxes, ha="center")
        for i in range(NUM_ROBOTS):
            # self.plot[i], = plt.plot([], [], marker=(3, 0, self.data[2,i]), markersize=ROBOT_RAD, c=colors[i%len(colors)])
            self.robs[i] = plt.Circle((), radius=ROBOT_RAD, color=colors[i%len(colors)])

        # self.ani = FuncAnimation(self.fig, self.update, frames=600, fargs=(robots), interval=10,
        #             init_func=self.init_func, blit=True)

        self.ani = FuncAnimation(self.fig, self.update, frames=self.data.shape[1]//NUM_ROBOTS, interval=25,
                    init_func=self.init_func, blit=True, repeat=True)
        plt.show()
        plt.grid()

    def init_func(self):
        self.ax.clear()
        plt.grid()
        self.ax.set_xlim(0, GRID_SIZE)
        self.ax.set_ylim(0, GRID_SIZE)
        # for r in list(self.plot.keys()):
        #     self.plot[r].set_data([], [])
        for i in range(NUM_ROBOTS):
            self.robs[i].center = (self.data[0,i], self.data[1,i])
            self.ax.add_patch(self.robs[i])
        return self.robs.values()

    # def update(self, i, *robots):
    #     for r in robots:
    #         x, y = r.p
    #         self.plot[r].set_data(x, y)
    #         self.plot[r].set_marker((3, 0, r.angle))
    #     return self.plot.values()
    def update(self, i):
        # plt.scatter([1],[1])
        for j in range(NUM_ROBOTS):
            x, y, angle = self.data[:, NUM_ROBOTS*i + j]
            self.robs[j].center = (x,y)
            # self.plot[j].set_marker((3, 0, angle))
        # self.fig.suptitle('Frame: {0}'.format(i))
        # self.title.set_text(u"Frame: {0}".format(i))
        return self.robs.values()


def main():
    global robots, Q, done
    fig = plt.figure()
    #converting from index to coordinates
    # print(NUM_ROBOTS)
    for i in range(NUM_ROBOTS):
        target = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
        while target in Q:
            target = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
        Q.append(target)
    if args.shape == 'n':
        Q = []
        Q.extend([[0, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])
        Q.extend([[i+1, GRID_SIZE//GRID_LENGTH - 2 - i] for i in range(int(GRID_SIZE//GRID_LENGTH - 2))])
        Q.extend([[GRID_SIZE//GRID_LENGTH - 1, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])
    elif args.shape == 'u':
        Q = []
        Q.extend([[0, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])
        Q.extend([[i+1, 0] for i in range(int(GRID_SIZE//GRID_LENGTH - 2))])
        Q.extend([[GRID_SIZE//GRID_LENGTH - 1, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])
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

    full_data = np.zeros((3,NUM_ROBOTS))
    for i in range(NUM_ROBOTS):
        full_data[0,i]=robots[i].p[0]
        full_data[1,i]=robots[i].p[1]
        full_data[2,i]=robots[i].angle

    for thread in threads:
        thread.start()

    startedloop=time.time()
    count = 0
    while len([t for t in threads if t.is_alive()]) > 0:
        # print("YO")
        change_done = True
        lst = np.zeros((3,NUM_ROBOTS))
        for i in range(NUM_ROBOTS):
            lst[0,i]=robots[i].msg[1][0]
            lst[1,i]=robots[i].msg[1][1]
            lst[2,i]=robots[i].angle
            if robots[i].done != True:
                change_done = False
                count = 0
        if change_done or time.time()-startedloop > run_dur:# and count > 10:
            done = True

        # akl = [lst[0,i], lst[1,i], robots[i].id]
        # seen = []
        # for i in range(NUM_ROBOTS):
        #     akl = (lst[0,i], lst[1,i])
        #     seen.append(akl)

        # for i in range(NUM_ROBOTS):
        #     print(robots[i].id, robots[i].msg[1], robots[i].msg[2], robots[i].msg[3])
        # if len(set(seen)) != len(seen):
        #     print("OHHHH NOOOOO")
            # print('------------------------------------------------------------------------')
            # for i in range(NUM_ROBOTS):
            #     for j in range(i+1,NUM_ROBOTS):
            #         if seen[i] == seen[j]:
            #             print(robots[i].id, robots[i].msg[1], robots[i].msg[2], robots[i].msg[3])
            #             print(robots[j].id, robots[j].msg[1], robots[j].msg[2], robots[j].msg[3])
            # print('------------------------------------------------------------------------')


        # print(lst.shape)
        # print(p_circ.shape)
        full_data = np.hstack((full_data, lst))
        time.sleep(0.05)
        # if count % 100 == 0:
        print(time.time()-startedloop)
        count+=1

    print("Time Taken: {0}".format(time.time()-startedloop))
    tot_dist = 0
    for rob in robots:
        tot_dist += rob.total_dist_traveled
    print("Total Distance Traveled: {0}".format(tot_dist))

    taken = []
    starting_dist = 0
    for pos in rob_init_pos:
        lst = sorted(Q, key=lambda x: manhattan(x, pos))
        for i in range(len(lst)):
            if lst[i] not in taken:
                taken.append(lst[i])
                starting_dist += manhattan(lst[i], pos)
                break
    print("Initial Distance: {0}".format(starting_dist))


    taken = []
    dist_to_convergence = 0
    for rob in robots:
        lst = sorted(Q, key=lambda x: manhattan(x, rob.p))
        for i in range(len(lst)):
            if lst[i] not in taken:
                taken.append(lst[i])
                dist_to_convergence += manhattan(lst[i], rob.p)
                break
    print("Distance to Convergence: {0}".format(dist_to_convergence))
    print('Percent Completed: {0}'.format((1-(dist_to_convergence/starting_dist))*100))

    for thread in threads:
        thread.join()
    # print(full_data)
    d = Draw(full_data, fig)



if __name__ == "__main__":
    main()
