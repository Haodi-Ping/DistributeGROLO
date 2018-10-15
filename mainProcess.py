from hexapodRobot import *
# from GlobalDebug import *
from D3_TE import *
from dv_distance import *
import shelve
import sys
import redis
import json
import time
from redis_netlock import dist_lock
communication_distance = 0   # assign in create_network_topology()


def cmp_by_value(lhs):
    return lhs[1]


def create_network_topology():
    global communication_distance
    points = np.loadtxt('data/node.npy')
    beacon_dis = np.loadtxt('data/beacon_dis.npy')
    Robot_Num = points.shape[0]
    communication_distance = beacon_dis[len(beacon_dis)-1]
    print('communication_distance', communication_distance)
    robots = [Robot(identity=x, epoch=50, lrn=0.02) for x in range(Robot_Num)]
    Beacon = np.array(beacon_dis[0: len(beacon_dis)-1], dtype=int)
    for index in Beacon:
        robots[index].setBeacon(points[index])
    # calculate the neighbor and sorted
    for i in range(Robot_Num):
        for j in range(i+1, Robot_Num):
            tempDistance = np.sqrt( (points[i][0] - points[j][0])**2 + (points[i][1] - points[j][1])**2
                                    + (points[i][2] - points[j][2])**2)
            if tempDistance < communication_distance:
                robots[i].myNeighbor.append([j, tempDistance])
                robots[j].myNeighbor.append([i, tempDistance])
    for r in robots:
        r.myNeighbor = sorted(r.myNeighbor, key=cmp_by_value)
        # robots move in Z, up and down.
        r.nei_id = []
        r.nei_pos = []
        for nei in r.myNeighbor:
            rid = r.id
            nid = nei[0]
            r.nei_id.append(nid)
            r.nei_pos.append(robots[nid].get_coord())
            r.measured_distance[nid] = np.sqrt((points[rid][0]-points[nid][0])**2 +
                    (points[rid][1]-points[nid][1])**2 +
                    (points[rid][2]+r.t-points[nid][2])**2 )

    # debug print
    for r in robots:
        s = ""
        for t in r.myNeighbor:
            s = s + "%d "%t[0]
        print("Robot%d have neighbor %s"%(r.id, s))
        print("Robot measured_distance", r.measured_distance)
        print("Robot%d neighbor's coord is "%(r.id), r.nei_pos)

    return points, robots

def get_new_topology(points, robots):
    # calculate the neighbor and
    Robot_Num = len(robots)
    global communication_distance
    for i in range(Robot_Num):
        for j in range(i+1, Robot_Num):
            tempDistance = np.sqrt( (points[i][0] - points[j][0])**2 + (points[i][1] - points[j][1])**2
                                    + (points[i][2] - points[j][2])**2)
            if tempDistance < communication_distance:
                robots[i].myNeighbor.append([j, tempDistance])
                robots[j].myNeighbor.append([i, tempDistance])
    for r in robots:
        r.myNeighbor = sorted(r.myNeighbor, key=cmp_by_value)
        # robots move in Z, up and down.
        r.nei_id = []
        for nei in r.myNeighbor:
            rid = r.id
            nid = nei[0]
            r.nei_id.append(nid)
            r.measured_distance[nid] = np.sqrt((points[rid][0]-points[nid][0])**2 +
                    (points[rid][1]-points[nid][1])**2 +
                    (points[rid][2]+r.t-points[nid][2])**2 )


def set_real_position(robots, localization_Nodes):
    cal_nodes = 0
    for index in range(len(robots)):
        if robots[index].isBeacon == False:
            robots[index].isFinalPos = False
        else:
            robots[index].isFinalPos = True
    print('real_position: localizationNodes is ', localization_Nodes)
    while cal_nodes < localization_Nodes:
        for index in range(len(robots)):
            if robots[index].isBeacon == True:
                continue
            if robots[index].isFinalPos == True:
                continue
            print('index %d come to calculate, cal_nodes is %d  '% (index, cal_nodes))
            p1 = robots[index].parent1
            p2 = robots[index].parent2
            if(p1 != -1 and p2 != -1 and robots[p1].isFinalPos == True and robots[p2].isFinalPos == True):
                ix, iy = robots[index].get_coord()
                p1x, p1y = robots[p1].get_coord()
                p2x, p2y = robots[p2].get_coord()
                dis1 = robots[index].d2_distances[p1]
                dis2 = robots[index].d2_distances[p2]
                def my_solve(paramter):
                    x, y = paramter[0], paramter[1]
                    return [
                        (x - p1x) ** 2 + (y - p1y) ** 2 - dis1 ** 2,
                        (x - p2x) ** 2 + (y - p2y) ** 2 - dis2 ** 2]
                sol = np.real(fsolve(my_solve, np.array([ix, iy]), xtol=1e-3))
                print('fsolve index ',index,sol)
                robots[index].set_coord(sol[0], sol[1])
                robots[index].isFinalPos = True
                cal_nodes = cal_nodes + 1


def localization_ontime(robots, current_robot_id, rs, flexibleNum, epochs=200):  # only current_robot can gradient descent and Guass-Newton
    robot_num = len(robots)
    epoch = 1
    while True:
        print("epoch %d:------------------------------------------------" % epoch)
        for r in robots:
            if r.id == current_robot_id:
                continue
            read_from_redis(r,rs)
        nei_dis = [value for value in robots[current_robot_id].d2_distances.values()]
        nei_pos = [robots[key].get_coord() for key in robots[current_robot_id].d2_distances.keys()]

        # if epoch > 2 and (epoch == epochs):
        #     set_real_position(robots, current_robot_id, robot_num - flexibleNum-3)
        #     continue
        print('localization_ontime robot',current_robot_id)
        robots[current_robot_id].run(nei_pos=nei_pos, nei_dis=nei_dis)
        print("robots[%d].coord: " % current_robot_id, robots[current_robot_id].get_coord())
        write_to_redis(robots[current_robot_id], rs)
        epoch = epoch+1

def write_to_redis(r, rs):
    strr = 'robot'+str(r.id)
    # pool = redis.ConnectionPool(host='localhost', port=6379, decode_responses=True)
    # rs = redis.Redis(connection_pool=pool)
    # with dist_lock('Testlock',rs):
    #     time.sleep(3)
    #     print('you sleep 3 seconds')
    # print('now wirte to redis')

    rs.set(strr + 'id' , r.id)
    rs.set(strr + 'coord', r.get_coord())
    if r.id == 8:
        print('------------------------------------------robots[8] coord ', r.get_coord())
    rs.set(strr + 'nei_id', r.nei_id)
    rs.set(strr + 'state', r.state)
    rs.set(strr + 'parent1', r.parent1)
    rs.set(strr + 'parent2', r.parent2)
    rs.set(strr + 'root1', r.root1)
    rs.set(strr + 'root2', r.root2)
    rs.set(strr + 'extra', r.extra)
    rs.set(strr + 'myNeighbor', r.myNeighbor)
    rs.set(strr + 'z', r.z)
    rs.set(strr + 'measured_distance', r.measured_distance)
    rs.set(strr + 'd2_distances', r.d2_distances)
    # print('write d2_distance', r.d2_distances)
    rs.set(strr + 'isBeacon', r.isBeacon)
    rs.set(strr + 'isFinalPos', r.isFinalPos)


def write_to_db(r):
    # save to db-robots
    db = shelve.open('data/db-robots'+str(r.id))
    # pickle.dump(r, open('data/test.pkl', 'wb'))
    try:
        db['id'] = r.id
        db['coord'] = r.get_coord()
        db['nei_id'] = r.nei_id
        db['state'] = r.state
        db['parent1'] = r.parent1
        db['parent2'] = r.parent2
        db['root1'] = r.root1
        db['root2'] = r.root2
        db['extra'] = r.extra
        db['myNeighbor'] = r.myNeighbor
        db['z'] = r.z
        db['measured_distance'] = r.measured_distance
        db['d2_distances'] = r.d2_distances
        # print('write d2_distance', r.d2_distances)
        db['isBeacon'] = r.isBeacon
        db['isFinalPos'] = r.isFinalPos
    finally:
        db.close()

def Test_D3_TE():
    points, robots = create_network_topology()
    parentList, distanceList, zList,flexibleNum = from_3D_to_2D(robots)
    print('parentList',parentList)
    print('ZList is', zList)
    print('distanceList',distanceList)


def read_from_redis(r, rs):
    # pool = redis.ConnectionPool(host='localhost', port=6379, decode_responses=True)
    # rs = redis.Redis(connection_pool=pool)
    strr = 'robot' + str(r.id)
    # print("robot id is %d ,rs.get(strr + 'coord')"%r.id, rs.get(strr + 'coord'))
    rediscoord = json.loads(rs.get(strr + 'coord'))
    # print('rediscoord is, tpye is, rediscoord[0] is', rediscoord, type(rediscoord),rediscoord[0])

    r.set_coord(rediscoord[0], rediscoord[1])
    # print('get robot[%d] s coord' % r.id, db['coord'], db['coord'][0], db['coord'][1])
    # print('get error key ',db['errorkey'])

    r.nei_id = json.loads(rs.get(strr + 'nei_id'))
    r.state = int(rs.get(strr + 'state'))
    r.parent1 = int(rs.get(strr + 'parent1'))
    r.parent2 = int(rs.get(strr + 'parent2'))
    r.root1 = int(rs.get(strr + 'root1'))
    r.root2 = int(rs.get(strr + 'root2'))
    r.extra = int(rs.get(strr + 'extra'))
    r.myNeighbor = json.loads(rs.get(strr + 'myNeighbor'))
    # print('r.myNeighbor',r.myNeighbor)
    r.z = rs.get('z')
    # print("rs.get(strr + 'measured_distance') ",rs.get(strr + 'measured_distance'))
    r.measured_distance = eval(rs.get(strr + 'measured_distance'))
    # print("rs.get(strr + 'd2_distances')", rs.get(strr + 'd2_distances'))
    r.d2_distances = eval(rs.get(strr + 'd2_distances'))
    # print("rs.get(strr + 'isBeacon')", rs.get(strr + 'isBeacon'))
    r.isBeacon = rs.get(strr + 'isBeacon') == str(True)
    r.isFinalPos = rs.get(strr + 'isFinalPos') == str(True)


def read_from_db(r):
    db = shelve.open('data/db-robots'+str(r.id))
    # print('db is',db)
    try:
        # r.id = db['id']

        r.set_coord(db['coord'][0],db['coord'][1])
        # print('get robot[%d] s coord' % r.id, db['coord'], db['coord'][0], db['coord'][1])
        # print('get error key ',db['errorkey'])
        r.nei_id = db['nei_id']
        r.state = db['state']
        r.parent1 = db['parent1']
        r.parent2 = db['parent2']
        r.root1 = db['root1']
        r.root2 = db['root2']
        r.extra = db['extra']
        r.myNeighbor = db['myNeighbor']
        # print('r.myNeighbor',r.myNeighbor)
        r.z = db['z']
        r.measured_distance = db['measured_distance']
        r.d2_distances = db['d2_distances']
        # print('r.d2_distances',r.d2_distances)
        r.isBeacon = bool(db['isBeacon'])
        r.isFinalPos = bool(db['isFinalPos'])
    finally:
        db.close()


def center_main():         # the center nodes . write all the robots to database
    pool = redis.ConnectionPool(host='192.168.1.212', port=6379, decode_responses=True)
    rs = redis.Redis(connection_pool=pool)
    points, robots = create_network_topology()
    parentList, distanceList, zList, flexibleNum = from_3D_to_2D(robots)  # calculate Z, and finish triangle extension
    setInitial(robots)  # estimate the robots' coord(x,y)
    for r in robots:
        write_to_redis(r, rs)
    # localization_ontime(robots, flexibleNum, 5)
    # show(points, robots)


def distribute_main(current_robot_id):
    Robot_Num = 10
    # robots = Robot(identity=current_robot, epoch=50, lrn=0.02)
    robots = [Robot(identity=x, epoch=50, lrn=0.02) for x in range(Robot_Num)]
    sess = tf.Session()
    robots[current_robot_id].robotStart(sess)
    pool = redis.ConnectionPool(host='192.168.1.212', port=6379, decode_responses=True)
    rs = redis.Redis(connection_pool=pool)
    # lock
    with dist_lock('Testlock', rs):
        time.sleep(1)
        print('you sleep 1 seconds')
    print('now wirte to redis')
    for r in robots:
        read_from_redis(r, rs)
    localization_ontime(robots, current_robot_id,rs, 7, 500)
    print('the robot parent is %d %d, state is %d' % (robots[current_robot_id].parent1, robots[current_robot_id].parent2, robots[current_robot_id].state))
    return robots


if __name__ == '__main__':
    if len(sys.argv) == 1:
        center_main()
        exit(0)
    #
    if len(sys.argv) != 2:
        print('Usage: python input current_robot_id')
        exit(1)

    current_robot_id = int(sys.argv[1])
    print('current_robot_id is ', current_robot_id)
    robots = distribute_main(current_robot_id)
    print('\n\n the finally robot[%d] coord is' % current_robot_id, robots[current_robot_id].get_coord() )

    # points = np.loadtxt('data/node.npy')
    # show(points, robots)

