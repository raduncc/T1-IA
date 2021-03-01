from math import sqrt
import time
import matplotlib.pyplot as plt

LIMIT = 6
IS_OBSTACLE = 2
INF = 99999
result = {}
H = {}
prev_s = None
prev_a = None

# pos_dict -> mapeaza id -> (x, y, is_obstacle)
# adj_list -> retine pt fiecare id, id-urile vecinilor
# cost_dict -> mapeaza (id_sursa, id_dest) -> cost
def init_env(file_in):
    global src, src_id, dst, dst_id, no_pos, no_edges, pos_dict, adj_list, cost_dict
    f = open(file_in, 'r')
    line = line_to_list(f)
    src = (int(line[0]), int(line[1]))
    line = line_to_list(f)
    dst = (int(line[0]), int(line[1]))
    line = line_to_list(f)
    no_pos = int(line[0])
    pos_dict = {}
    cost_dict = {}
    adj_list = {}
    for i in range(no_pos):
        line = line_to_list(f)
        if (int(line[1]), int(line[2])) == src:
            src_id = int(line[0])
        if (int(line[1]), int(line[2])) == dst:
            dst_id = int(line[0])
        if (len(line) == 4):
            pos_dict[int(line[0])] = (int(line[1]), int(line[2]), True)
        else:
            pos_dict[int(line[0])] = (int(line[1]), int(line[2]), False)
    line = line_to_list(f)
    no_edges = int(line[0])
    for i in range(no_edges):
        line = line_to_list(f)
        a = int(line[0])
        b = int(line[1])
        cost = int(line[2])
        cost_dict[(a, b)] = cost
        cost_dict[(b, a)] = cost
        if a not in adj_list.keys():
            adj_list[a] = [b]
        else:
            adj_list[a].append(b)
        if b not in adj_list.keys():
            adj_list[b] = [a]
        else:
            adj_list[b].append(a)


def is_final(state):
    return state == dst_id


# transforma o linie din fisierul de input in lista de elemente relevante
def line_to_list(file_in):
    line = file_in.readline()
    line = line[:-1]
    line = line.split(", ")
    return line

#2*sqrt(|x1-x2|*|y1-y2|)
def my_heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return int(2*sqrt(abs(x1-x2)*abs(y1-y2)))

#sqrt((x1-x2)^2+(y1-y2)^2)
def euclidean_distance(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return int(sqrt((x1-x2)**2 + (y1-y2)**2))


def get_next_states():
    global next_sts
    next_sts = {}
    for id in pos_dict.keys():
        for s in adj_list[id]:
            if pos_dict[s][IS_OBSTACLE] == False:
                if id not in next_sts:
                    next_sts[id] = [s]
                else:
                    next_sts[id].append(s)


def DFID(s, g, U):
    global U1, discovered

    if is_final(s):
        path = []
        parent = discovered[dst_id][0]
        path.append(dst_id)
        while parent is not None:
            path.append(parent)
            parent = discovered[parent][0]
        path.reverse()
        print('costul e ' + str(discovered[dst_id][1]))
        return path

    succesors = next_sts[s]

    for v in succesors:
        current_cost = cost_dict[(s, v)]
        if v in discovered.keys():
            if g + current_cost >= discovered[v][1]:
                continue
        if g + current_cost <= U:
            discovered[v] = (s, g + current_cost)
            p = DFID(v, g + current_cost, U)
            if p != []:
                return p
        else:
            if g + current_cost < U1:
                U1 = g + current_cost
    return []


def DFID_loop(state):
    global U1, discovered

    U1 = 0
    best_path = []
    while best_path == [] and U1 != INF:
        U = U1
        U1 = INF
        discovered = {state: (None, 0)}
        best_path = DFID(state, 0, U)
    return best_path


def IDA(s, g, U, h):
    global U1, discovered

    if is_final(s):
        path = []
        parent = discovered[dst_id][0]
        path.append(dst_id)
        while parent is not None:
            path.append(parent)
            parent = discovered[parent][0]
        path.reverse()
        print('costul e ' + str(discovered[dst_id][1]))
        return path

    succesors = next_sts[s]

    for v in succesors:

        (x, y, is_obs) = pos_dict[v]
        h_value = h((x, y), dst)
        current_cost = cost_dict[(s, v)] + h_value

        if v in discovered.keys():
            if g + current_cost >= discovered[v][1]:
                continue

        if g + current_cost <= U:
            discovered[v] = (s, g + current_cost)
            p = IDA(v, g + current_cost - h_value, U, h)
            if p != []:
                return p
        else:
            if g + current_cost < U1:
                U1 = g + current_cost

    return []


def IDA_loop(state, h):
    global U1, discovered

    U1 = h(src, dst)
    best_path = []
    while best_path == [] and U1 != INF:
        U = U1
        U1 = INF
        discovered = {state: (None, 0)}
        best_path = IDA(state, 0, U, h)
    return best_path


def plot_result(path):
    for id in adj_list.keys():
        if id == src_id:
            (src_x, src_y) = src
            plt.plot(src_x, src_y, 'ys', markersize=10)
        if id == dst_id:
            (dst_x, dst_y) = dst
            plt.plot(dst_x, dst_y, 'rs', markersize=10)
        (x, y, is_obs) = pos_dict[id]
        if id in path:
            if (x, y) != src and (x, y) != dst:
                plt.plot(x, y, 'gs', markersize=10)
        elif is_obs:
            plt.plot(x, y, 'bs', markersize=10)
    plt.show()


def LRTA(s1, h):
    global H, result, prev_a, prev_s
    if is_final(s1):
        return None
    if s1 not in H:
        (x, y, is_obs) = pos_dict[s1]
        H[s1] = h((x, y), dst)
    if prev_s != None:
        result[(prev_s, prev_a)] = s1
        val = INF
        actions = next_sts[prev_s]
        for act in actions:
            aux = LRTA_cost(prev_s, act, h)
            if aux < val:
                val = aux
        H[prev_s] = val
    val = INF
    aux_a = None
    actions = next_sts[s1]
    for act in actions:
        aux = LRTA_cost(s1, act, h)
        if aux < val:
            val = aux
            aux_a = act
    prev_a = aux_a
    prev_s = s1
    return prev_a


def LRTA_cost(s1, s2, h):
    global H, result, prev_a, prev_s
    if s2 not in H:
        (x, y, is_obs) = pos_dict[s1]
        return h((x, y), dst)
    return cost_dict[(s1, s2)] + H[s2]


def get_path_LRTA(srs, h):
    path = [srs]
    x = LRTA(srs, h)
    while x != None:
        path.append(x)
        x = LRTA(x, h)
    return path


def exec_LRTA(h):
    flag = 0
    times_run = 1
    cost = 0
    start_time = time.time()
    prev_path = get_path_LRTA(src_id, h)
    while True:
        if flag == LIMIT:
            break
        path = get_path_LRTA(src_id, h)
        times_run += 1
        if path == prev_path:
            flag += 1
        else:
            flag = 0
        prev_path = path
    print("--- %s seconds ---" % (time.time() - start_time))
    
    for i in range(0, len(path)-1):
        cost += cost_dict[path[i], path[i + 1]]
    print('costul e ' + str(cost) + ' dupa ' + str(times_run) + ' rulari')
    print(str(path))
    # plot_result(path)


def exec_IDA(h):
    start_time = time.time()
    path = IDA_loop(src_id, h)
    print("--- %s seconds ---" % (time.time() - start_time))
    print(str(path))
    # plot_result(path)


def exec_DFID():
    start_time = time.time()
    path = DFID_loop(src_id)
    print("--- %s seconds ---" % (time.time() - start_time))
    print(str(path))
    # plot_result(path)


if __name__ == "__main__":
    input_no = input('Insert input number: ')
    input_name = 'input' + input_no + '.txt'
    init_env(input_name)
    get_next_states()
    print('===  IDA* MY HEURISTIC  ===')
    exec_IDA(my_heuristic)
    print('===  IDA* EUCLIDEAN  ===')
    exec_IDA(euclidean_distance)
    print('===  DFID  ===')
    exec_DFID()
    print('===  LRTA* MY HEURISTIC  ===')
    #trebuie resetate variabilele globale pentru urmatorul apel de LRTA*
    exec_LRTA(my_heuristic)
    result = {}
    H = {}
    prev_s = None
    prev_a = None
    print('===  LRTA* EUCLIDEAN  ===')
    exec_LRTA(euclidean_distance)