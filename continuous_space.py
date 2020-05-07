import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import argparse
from shapely.geometry import Polygon
from shapely.geometry.point import Point
from shapely.geometry import LineString

parser = argparse.ArgumentParser()
parser.add_argument('--num_robots', '-nr', type=int, default=3)
parser.add_argument('--shape', '-sh', type=str, default='any')
args = parser.parse_args()

GRID_SIZE = 100
GRID_LENGTH = 20#l (not exactly l but corresponds to l)
ROBOT_RAD = GRID_LENGTH/(3*np.sqrt(2))#r
SAFETY_BUFFER = ROBOT_RAD*0.1
COM_RANGE = GRID_LENGTH*2.5*2#R
# ROBOT_RAD = GRID_LENGTH*2#r (here it is slightly different from r since plt scaling is weird)
TRANSMIT_FREQ = 200#f_comm
# SPEED = 0.1 #blocks/sec
# LISTENING_TIME = 100/TRANSMIT_FREQ
dt = 1/200000
STEP_SIZE = 5

if args.shape == 'n' or args.shape == 'u':
    NUM_ROBOTS = 2*(GRID_SIZE//GRID_LENGTH) + (GRID_SIZE//GRID_LENGTH)-2
else:
    NUM_ROBOTS = args.num_robots

Q = []
robots = []
ack_locks = {}
request_locks = {}
run_dur = 50

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def norm(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

def check_col(a, b1, b2):
    roba = Point(a[0], a[1]).buffer(ROBOT_RAD + SAFETY_BUFFER)
    robb = LineString([(b1[0], b1[1]), (b2[0], b2[1])]).buffer(ROBOT_RAD + SAFETY_BUFFER)
    return roba.intersects(robb)
    # print(p1.intersects(p2))


class Robot():
    def __init__(self, x, y, id):
        self.x = x#GRID_LENGTH*(x + 0.5)
        self.y = y#GRID_LENGTH*(y + 0.5)
        self.angle = np.random.randint(4)*90
        self.p = [self.x, self.y]
        self.wp = self.p
        self.nextwp = self.wp
        self.id = id
        self.hop = np.inf
        self.delta_t = 100/TRANSMIT_FREQ
        self.qu = Q[np.random.randint(len(Q))]
        self.T = Q[np.random.randint(len(Q))]
        print(self.id, [x, y], [self.T[0], self.T[1]])
        self.last_check = time.time()
        self.msg = [time.time(), self.p, self.wp, self.nextwp, self.T, self.qu, self.hop]
        self.ack = None
        self.requests = []

    def main_rob(self):
        time.sleep(1)
        t1 = threading.Thread(target=self.broadcast)
        t2 = threading.Thread(target=self.goal_manager)
        self.start = time.time()
        t1.start()
        t2.start()

        while time.time()-self.start <= run_dur:
            # if self.id ==1:
            #     print('------------------------------')
            #     print(self.id, self.wp, self.T)
            # surroundings = [[self.wp[0], self.wp[1] + GRID_LENGTH], [self.wp[0] - GRID_LENGTH, self.wp[1]],
            #     [self.wp[0], self.wp[1] - GRID_LENGTH], [self.wp[0] + GRID_LENGTH, self.wp[1]]]

            thetas = np.arange(0,2*np.pi,np.pi/16).tolist()

            # idxs = [0,1,2,3]

            #boundary checks
            for i in range(len(thetas)-1, -1, -1):
                x, y = self.x + STEP_SIZE*np.cos(thetas[i]), self.y + STEP_SIZE*np.sin(thetas[i])
                # x, y = surroundings[i][0], surroundings[i][1]
                if x < ROBOT_RAD or y < ROBOT_RAD or x > GRID_SIZE-ROBOT_RAD or y > GRID_SIZE-ROBOT_RAD:
                    thetas.pop(i)
                    # idxs.pop(i)

            wait_flag = 0
            # next_angle = self.angle
            bestnorm = norm(self.T, self.wp)
            for i in range(len(thetas)):
                next_point = [self.x + STEP_SIZE*np.cos(thetas[i]), self.y + STEP_SIZE*np.sin(thetas[i])]
                if norm(self.T, next_point) < bestnorm:#norm(self.T, self.wp):

                    # print(next_point)
                    self.nextwp = next_point
                    bestnorm = norm(self.T, next_point)
                    # if self.id == 1:
                    #     print('YEAAA')
                    #     print(self.wp, self.nextwp, self.T)
                    # next_angle = idxs[i]*90
                    # break
                elif norm(self.T, self.wp) <= STEP_SIZE:
                    self.nextwp = self.T
                    break
            # if self.id == 0:
            #     print(self.nextwp)

            rcvmsgs = []
            started_listening = time.time()
            while time.time()-started_listening <= self.delta_t:
                rcvmsgs = []
                for r in robots:
                    if r.id != self.id and norm(r.p, self.p) <= COM_RANGE:
                        rcvmsgs.append(r.msg)
            if len(rcvmsgs) > 0:
                msg_min = min(rcvmsgs, key=lambda x: x[-1])
                self.hop = 1 + msg_min[-1]
                self.qu = msg_min[-2]
                for i in thetas:
                    poss_state = [self.x + STEP_SIZE*np.cos(i), self.y + STEP_SIZE*np.sin(i)]
                    # if poss_state in Q:
                    state = None
                    for j in Q:
                        if norm(poss_state, j) <= ROBOT_RAD:
                            state = j
                            break
                    if state is not None:
                        occupied = 0
                        for msg in rcvmsgs:
                            if norm(msg[-3], state) <= ROBOT_RAD:
                            # if msg[-3] == i:
                                occupied = 1
                                break
                        if occupied == 0:   #fix this later
                            self.qu = state
                            self.hop = 0
                            break
                for msg in rcvmsgs:
                    collision = check_col(msg[2], self.wp, self.nextwp)
                    same_next_collision = check_col(msg[3], self.wp, self.nextwp)
                    # if msg[2][0] == self.nextwp[0] and msg[2][1] == self.nextwp[1]:
                    if collision:
                        print("FIRST CASE")
                        print(self.id, self.wp, self.nextwp, self.T)
                        wait_flag = 1
                    # if msg[3][0] == self.nextwp[0] and msg[3][1] == self.nextwp[1]:
                    if same_next_collision:
                        ix, iy = msg[1]
                        if ix > self.x or (ix == self.x and iy > self.y):
                            print("SECOND CASE")
                            print(self.id, self.wp, self.nextwp, self.T)
                            wait_flag = 1
            # if self.id == 1 and self.wp != self.nextwp and wait_flag == 1:
            #     print(self.id, self.wp, self.nextwp)
            if wait_flag == 0 and time.time() - self.last_check > self.delta_t:
                # if self.id == 0:
                #     print("YEA")
                #     if self.wp == self.nextwp:
                #         print('staying still')
                #     else:
                #         print("moving")
                self.wp = self.nextwp
                start_moving = time.time()
                ox, oy = self.x, self.y
                # oang = self.angle
                # dir = 1 if next_angle - self.angle <= 180 else -1

                while norm(self.p, self.wp) >= 0.1:
                    self.x += dt*(self.wp[0] - ox)
                    self.y += dt*(self.wp[1] - oy)
                    self.p = [self.x, self.y]
                self.p = self.wp
                self.x, self.y = self.p
                self.last_check = time.time()

        t1.join()
        t2.join()
        return False

    def broadcast(self):
        while time.time()-self.start <= run_dur:
            self.msg = [time.time(), self.p, self.wp, self.nextwp, self.T, self.qu, self.hop]
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

        while time.time()-self.start <= run_dur:
            s = time.time()
            for r in robots:
                if norm(r.p, self.p) <= COM_RANGE:


                    rp = r.msg[1].copy()
                    rwp = r.msg[2].copy()
                    rnextwp = r.msg[3].copy()
                    rT = r.msg[4].copy()
                    rqu = r.msg[5].copy()
                    rhop = r.msg[6]
                    r_msg = [r.msg[0], rp, rwp, rnextwp, rT, rqu, rhop]

                    p = self.msg[1].copy()
                    wp = self.msg[2].copy()
                    nextwp = self.msg[3].copy()
                    T = self.msg[4].copy()
                    qu = self.msg[5].copy()
                    hop = self.msg[6]

                    if r_msg[4] == T:  #if neighbor has same goal
                        rx, ry = r_msg[1]
                        x, y = p
                        if rx > x or (rx == x and ry > y):   #if neighbor is lexically larger position
                            if np.random.random() > 0.1:
                                self.T = qu
                                self.last_check = time.time()
                            else:
                                self.T = Q[np.random.randint(len(Q))]
                                self.last_check = time.time()

                    if norm(r_msg[4], wp) + norm(r_msg[2], T) < norm(T, wp) + norm(r_msg[4], r_msg[2]):
                        successful, new_goal = two_way_handshake(r_msg, p, T)

                        if successful:
                            self.T = new_goal
                            self.last_check = time.time()
                    if norm(r_msg[4], wp) + norm(r_msg[2], T) == norm(T, wp) + norm(r_msg[4], r_msg[2]):
                        if np.random.random() < 0.5:
                            successful, new_goal = two_way_handshake(r_msg, p, T)
                            if successful:
                                self.T = new_goal
                                self.last_check = time.time()

class Draw():
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
        for i in range(NUM_ROBOTS):
            self.robs[i] = plt.Circle((), radius=ROBOT_RAD, color=colors[i%len(colors)])

        self.ani = FuncAnimation(self.fig, self.update, frames=self.data.shape[1]//NUM_ROBOTS, interval=20,
                    init_func=self.init_func, blit=False, repeat=True)
        plt.show()
        plt.grid()

    def init_func(self):
        self.ax.set_xlim(0, GRID_SIZE)
        self.ax.set_ylim(0, GRID_SIZE)

        for i in range(NUM_ROBOTS):
            self.robs[i].center = (self.data[0,i], self.data[1,i])
            self.ax.add_patch(self.robs[i])
        return self.robs.values()

    def update(self, i):
        final_points = np.array(Q)
        plt.scatter(final_points[:,0], final_points[:,1], c='red')
        for j in range(NUM_ROBOTS):
            x, y, angle = self.data[:, NUM_ROBOTS*i + j]
            self.robs[j].center = (x,y)
        self.fig.suptitle('Frame: {0}'.format(i))
        return self.robs.values()


def main():
    global robots, Q
    fig = plt.figure()
    #converting from index to coordinates
    for i in range(NUM_ROBOTS):
        collides = True
        while collides:
            collides = False
            target = [(GRID_SIZE - 2*ROBOT_RAD)*np.random.random() + ROBOT_RAD, (GRID_SIZE - 2*ROBOT_RAD)*np.random.random() + ROBOT_RAD]
            # target = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
            p1 = Point(target[0], target[1]).buffer(ROBOT_RAD + SAFETY_BUFFER)
            for j in range(len(Q)):
                p2 = Point(Q[j][0], Q[j][1]).buffer(ROBOT_RAD + SAFETY_BUFFER)
                if p1.intersects(p2):
                    collides = True
                    break

        Q.append(target)
    # if args.shape == 'n':
    #     Q = []
    #     Q.extend([[0, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])
    #     Q.extend([[i+1, GRID_SIZE//GRID_LENGTH - 2 - i] for i in range(int(GRID_SIZE//GRID_LENGTH - 2))])
    #     Q.extend([[GRID_SIZE//GRID_LENGTH - 1, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])
    # elif args.shape == 'u':
    #     Q = []
    #     Q.extend([[0, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])
    #     Q.extend([[i+1, 0] for i in range(int(GRID_SIZE//GRID_LENGTH - 2))])
    #     Q.extend([[GRID_SIZE//GRID_LENGTH - 1, i] for i in range(int(GRID_SIZE//GRID_LENGTH))])

    # Q.append([0,3])
    # Q.append([10,1])
    # Q.append([3,5])
    print(Q)
    # for i in range(len(Q)):
    #     for j in range(len(Q[0])):
    #         Q[i][j] = GRID_LENGTH*(Q[i][j]+0.5)

    rob_init_pos = []
    robots = []
    id = 0
    for i in range(NUM_ROBOTS):
        collides = True
        while collides:
            collides = False
            init_pos = [(GRID_SIZE - 2*ROBOT_RAD)*np.random.random() + ROBOT_RAD, (GRID_SIZE - 2*ROBOT_RAD)*np.random.random() + ROBOT_RAD]
            # target = [np.random.randint(GRID_SIZE//GRID_LENGTH), np.random.randint(GRID_SIZE//GRID_LENGTH)]
            p1 = Point(init_pos[0], init_pos[1]).buffer(ROBOT_RAD + SAFETY_BUFFER)
            for j in range(len(rob_init_pos)):
                p2 = Point(rob_init_pos[j][0], rob_init_pos[j][1]).buffer(ROBOT_RAD + SAFETY_BUFFER)
                if p1.intersects(p2):
                    collides = True
                    break
        robots.append(Robot(init_pos[0], init_pos[1], id))
        rob_init_pos.append(init_pos)
        id+=1
    # r = Robot(0, 0, 0)  #blue - id:0
    # r1 = Robot(1, 0, 1) #red - id:1
    # r2 = Robot(0, 1, 2) #green - id:2
    # r1 = Robot(10, 80, 1)
    # robots = [r, r1, r2]
    for r in robots:
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
        lst = np.zeros((3,NUM_ROBOTS))
        for i in range(NUM_ROBOTS):
            lst[0,i]=robots[i].msg[1][0]
            lst[1,i]=robots[i].msg[1][1]
            lst[2,i]=robots[i].angle

        # akl = [lst[0,i], lst[1,i], robots[i].id]
        # seen = []
        # for i in range(NUM_ROBOTS):
        #     akl = (lst[0,i], lst[1,i])
        #     seen.append(akl)
        # if count %50 == 0:
        #     for i in range(NUM_ROBOTS):
        #         # print(robots[i].id, robots[i].msg[1], robots[i].msg[2], robots[i].msg[3])
        #         print(robots[i].id, robots[i].msg[1], robots[i].msg[2], robots[i].T)
        #     print("---------------------------------------------------------------------------")
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
        if count % 100 == 0:
            print(time.time()-startedloop)
        count+=1

    for thread in threads:
        thread.join()
    print(full_data)
    d = Draw(full_data, fig)



if __name__ == "__main__":
    main()
