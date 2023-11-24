# 提供索引值取得 node 的座標
def get_nodes(node_list):
    locations = []
    for i in node_list:
        locations.append(data['num_locations'][i])
    return locations

# 計算角度: 計算 Drone 的返回點, 讓等待最小化
def calculate_angle(locations):
    A, B, C = locations[0], locations[1], locations[2]
    vector_AB = [B[0] - A[0], B[1] - A[1]] # 計算向量 AB 和 BC
    vector_BC = [C[0] - B[0], C[1] - B[1]]
    dot_product = vector_AB[0] * vector_BC[0] + vector_AB[1] * vector_BC[1] # 計算向量的點積和模長
    magnitude_AB = math.sqrt(vector_AB[0]**2 + vector_AB[1]**2)
    magnitude_BC = math.sqrt(vector_BC[0]**2 + vector_BC[1]**2)
    angle_radians = math.acos(dot_product / (magnitude_AB * magnitude_BC)) # 計算角度(以弧度為單位)
    angle_degrees = math.degrees(angle_radians) # 轉換為角度(以度為單位)
    return round(angle_degrees, 1)

# 判斷路線方向, 並更新路徑
def get_direction_by_truck_route(routes, sidekick_routes, name):
    assert name == 'drone' or name == 'robot'
    new_routes = sidekick_routes.copy()
    for i, value in enumerate(sidekick_routes):
        for j, route in enumerate(value): # 每筆路徑方向
            start = sidekick_routes[i][j][0]
            end = sidekick_routes[i][j][-1]          
            s_index = routes[i].index(start)
            e_index = routes[i].index(end)
            if s_index > e_index: # 判斷路徑是否為車輛同行進方向, 如果則改變路徑方向
                new_routes[i][j] = new_routes[i][j][::-1]
    return new_routes

# 找到每個 node 排除組合中最近的兩個點, 並使用 set() 排除重複點
def get_closest_nodes(nodes, exclude=[]):
    neighbors = []
    for i in nodes:
        f_dists = []
        f_indexes = []
        f_veh = []
        # 取得 node 與其他點的距離, 使用 exclude 排除掉特定 nodes
        for index, dist in enumerate(data['distance_matrix'][i]):
            if (index not in nodes) and (index not in exclude):
                f_dists.append(dist)
                f_indexes.append(index)
        neighbors.append(sorted(zip(f_dists, f_indexes))[0][1])
        neighbors.append(sorted(zip(f_dists, f_indexes))[1][1])
    return list(set(neighbors))

# 計算鄰近點的所有可行路徑組合
def get_robot_route_combination(robot_route, neighbors):
    solution = []
    for i in neighbors:
        for j in neighbors:
            if i != j:
                _route = robot_route.copy()
                _route.insert(0, i)
                _route.insert(-1, j)
                duration = get_objective_value(data, _route, ROBOT_SPEED)
                if duration + ROBOT_SETUP_TIME * 2 < ROBOT_BATTERY_LIFE:
                    solution.append(_route) 
    return solution

# 根據車輛路線取得 Robot 路徑
def get_robot_route():
    robot_data = []
    robot_routes = [[] for i in range(len(routes))]
    excluded_nodes = data['drone'] + [item for sublist in data['robot'] for item in sublist]
    for group in data['robot']:  
        neighbors = []
        for i in group:
            f_dists, f_indexes = [], []
            for index, dist in enumerate(data['distance_matrix'][i]):
                if index not in excluded_nodes:
                    f_dists.append(dist)
                    f_indexes.append(index)
            sorted_index = [y for x, y in sorted(zip(f_dists, f_indexes))]        
            count = 0
            for j in sorted_index:
                if count == 3:
                    break
                for veh_i, veh_r in enumerate(routes):
                    if j in veh_r:
                        # 找出與 Group 中每個 Node 最近的三個點 (2 個可能會無解)
                        neighbors.append((j, veh_i))
                        count += 1
                    if count == 3:
                        break
        # 計算負責車輛
        points = [0 for i in range(len(routes))]
        best_neighbors = [[] for i in range(len(routes))]   
        neighbors = list(set(neighbors))
        for node in neighbors:
            points[node[1]] += 1
            best_neighbors[node[1]].append(node)
            if max(points) == 2:
                assigned_veh = points.index(max(points))    
        # 排除掉其他車輛的 Node
        neighbors = [i[0] for i in neighbors if i[1] == assigned_veh]
        # 找出該車輛路徑中, 鄰近點構成的最佳路徑排列組合
        if len(neighbors) > 0:
            combination, time = [], []        
            for index_j, j in enumerate(neighbors):
                for index_k, k in enumerate(neighbors):
                    if j != k:
                        r_route = [j]+group+[k]
                        duration = get_objective_value(data, route=r_route, speed=ROBOT_SPEED)
                        combination.append(r_route)
                        time.append(duration)
            shortest_route = combination[time.index(min(time))]
            robot_data.append({'name': 'robot', 'node': group, 'route': shortest_route, 'vehicle': assigned_veh, 'duration': min(time), 'neighbors': neighbors})
            print(f'Group {group} assign to: {assigned_veh};  Route: {shortest_route} ({min(time)})')
    for obj in robot_data:
        robot_routes[obj['vehicle']].append(obj['route'])  
    # 調整 robot 的方向, 使方向與車輛路線一致
    robot_routes = get_direction_by_truck_route(routes, robot_routes, 'robot')
    return robot_routes

# Set Time Window for VRPTW
def set_time_window():

    robot_durs = [[] for i in range(len(routes))]

    data["time_windows"] = [(0, 640) for i in range(data["num_nodes"])]
    temp_constraint = {'nodes': [], 'min_travel': [], 'vehicle': []}

    for index, group in enumerate(robot_routes):
        print('Vehicle', index)
        for r_route in group:
            r_dur = get_objective_value(data, r_route, ROBOT_SPEED) # 取得 robot_routes 中每個組合的時間
            print('Robot Route:', r_route, ', Duration:', r_dur)
            robot_durs[index].append(r_dur)
            for node in r_route:
                if 0 < r_route.index(node) < len(r_route) - 1:
                    data['demands'][node] = 999  # 將指定點的需求設置為 999
                    data["vehicle_capacities"][index] -= 1  # 修正該 node 負責車輛的容量         
                # 頭尾設定 Time Windows (車輛不能等待超過 10 分鐘)          
                elif r_route.index(node) == 0:
                    cumul_time = get_objective_value(data, routes[index][:routes[index].index(node)+1])
                    node_tw = (cumul_time, cumul_time)
                    data["time_windows"][node] = node_tw
                    print('- Node', node, node_tw)
                elif r_route.index(node) == len(r_route) - 1:
                    node_tw = (cumul_time + r_dur, cumul_time + r_dur + 10)
                    data["time_windows"][node] = node_tw
                    print('- Node', node, node_tw)
            temp_constraint['nodes'].append((r_route[0], r_route[-1]))
            temp_constraint['min_travel'].append(r_dur)
            temp_constraint['vehicle'].append(index)
    return emp_constraint

def print_vrptw_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{manager.IndexToNode(index)}"
                f" Time({solution.Min(time_var)},{solution.Max(time_var)})"
                " -> "
            )
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += (
            f"{manager.IndexToNode(index)}"
            f" Time({solution.Min(time_var)},{solution.Max(time_var)})\n"
        )
        plan_output += f"Time of the route: {solution.Min(time_var)}min\n"
        print(plan_output)
        total_time += solution.Min(time_var)
    print(f"Total time of all routes: {total_time}min")

def solve_vrptw(data, temp_constraint={}):
    """Solve the VRP with time windows."""
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["time_matrix"]), data["num_vehicles"], data["depot"]
    )
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create a demand callback.
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        10,  # allow waiting time
        640,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    depot_idx = data["depot"]
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1]
        )
    # Instantiate route start and end times to produce feasible times.
    for i in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Assign nodes to specific vehicle
    if len(temp_constraint) > 0:
        for index, nodes in enumerate(temp_constraint['nodes']):
            print(nodes)
            '''
            Sets the vehicles which can visit a given node. 
            If the node is in a disjunction, this will not prevent it from being unperformed. 
            Specifying an empty vector of vehicles has no effect (all vehicles will be allowed to visit the node)
            '''
            vehicle_id = temp_constraint['vehicle'][index]
            print(vehicle_id)
            routing.SetAllowedVehiclesForIndex([vehicle_id], nodes[0])
            routing.SetAllowedVehiclesForIndex([vehicle_id], nodes[1])

    # 設計懲罰項: AddDisjuntion() 只能呼叫一次, 要將 node 排除在路徑中需要調整 Capacity
    penalty = 1000
    for node in range(1, len(data["time_matrix"])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # 平衡車輛 loading, 避免部分車輛不出發跑路線
    for vehicle_id in range(data['num_vehicles']):
        routing.SetVehicleUsedWhenEmpty(True, vehicle_id)
    routing.AddConstantDimension(
        1, # +1 for each visit (note start node is counted so unused vehicle is still 1)
        data["num_nodes"] // data["num_vehicles"], # max visit allowed,  hard limit
        True,  # start cumul to zero
        "Counter")
    counter_dimension = routing.GetDimensionOrDie("Counter")
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.End(vehicle_id)
        counter_dimension.SetCumulVarSoftLowerBound(index, 2, 100000) # penalty of 100000 for each empty vehicle since counter will be 1 aka (2 - 1) * 100_000  

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.solution_limit = 100
    search_parameters.time_limit.seconds = 5

    # Solve the problem & print solution.
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        vrptw_routes = get_routes(solution, routing, manager)
        print_vrptw_solution(data, manager, routing, solution)
        return vrptw_routes
    else:
        print('No solution.')

# 畫出分群結果
def plot_cluster(labels):
    unique_labels = set(labels)
    if len(unique_labels) == 1:
        print('Failed clustering!')
    else:
        color_labels = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
        colors = [color_labels[label - 1] for label in labels] # combine labes and colors

        for node, label, color in zip(data['num_locations'], labels, colors):
            if label == -1: # Black used for noise.
                color = [0, 0, 0, 1]
            plt.scatter(x=node[0], y=node[1], s=300, color=color, alpha=0.8, edgecolors='black') 
            plt.text(x=node[0], y=node[1], s=str(data['num_locations'].index(node)), fontsize=7.5, color="white", horizontalalignment='center', verticalalignment='center')
        plt.show()

def solve_by_VRPR(data):
    start_time = time.time()
    tracemalloc.start()
    '''algorithm starts'''
    routes = solve_routing_problem(data)
    robot_routes = select_robot_nodes(data, routes)
    # 調整車輛路線
    for index, route in enumerate(routes):
        exclude_nodes = [item for sublist in robot_routes[index] for item in sublist if 0 < sublist.index(item) < len(sublist) - 1]
        routes[index] = [i for i in route if i not in exclude_nodes]
    # Set Time Windows
    robot_durs = [[] for i in range(len(routes))]

    data["time_windows"] = [(0, 640) for i in range(data["num_nodes"])]

    temp_constraint = {'nodes': [], 'min_travel': [], 'vehicle': []}

    for index, group in enumerate(robot_routes):
        print('Vehicle', index)
        for r_route in group:
            r_dur = get_objective_value(data, r_route, ROBOT_SPEED) # 取得 robot_routes 中每個組合的時間
            print('Robot Route:', r_route, ', Duration:', r_dur)
            robot_durs[index].append(r_dur)
            for node in r_route:
                if 0 < r_route.index(node) < len(r_route) - 1:
                    data['demands'][node] = 999  # 將指定點的需求設置為 999
                    data["vehicle_capacities"][index] -= 1  # 修正該 node 負責車輛的容量         
                # 頭尾設定 Time Windows (車輛不能等待超過 10 分鐘)          
                elif r_route.index(node) == 0:
                    cumul_time = get_objective_value(data, routes[index][:routes[index].index(node)+1])
                    node_tw = (cumul_time, cumul_time)
                    data["time_windows"][node] = node_tw
                    print('- Node', node, node_tw)
                elif r_route.index(node) == len(r_route) - 1:
                    node_tw = (cumul_time + r_dur, cumul_time + r_dur + 10)
                    data["time_windows"][node] = node_tw
                    print('- Node', node, node_tw)
            temp_constraint['nodes'].append((r_route[0], r_route[-1]))
            temp_constraint['min_travel'].append(r_dur)
            temp_constraint['vehicle'].append(index)
    # Solve VRPTW
    vrptw_routes = solve_vrptw(data, temp_constraint)
    if vrptw_routes:
        robot_waiting = get_waiting_time(data, vrptw_routes, robot_routes, 'robot')
        drone_waiting = [[] for i in range(len(robot_waiting))]
        total_dist, travel_time, waiting_time = get_travelling_time(data, vrptw_routes, drone_waiting, robot_waiting)
    else:
        robot_waiting, drone_waiting, total_dist, travel_time, waiting_time = None, None, None, None, None
    '''algorithm ends'''
    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    time_complexity = round(end_time - start_time, 4)
    space_complexity = round(current / 1024, 4)
    VRPR_result = dict(VRPR_routes=vrptw_routes, VRPR_robot_routes=robot_routes, VRPR_distances=total_dist, VRPR_time=travel_time, VRPR_waiting=waiting_time, VRPR_time_complexity=time_complexity, VRPR_space_complexity=space_complexity)
    return VRPR_result

def solve_by_VRPRD(data):
    start_time = time.time()
    tracemalloc.start()
    '''algorithm starts'''
    routes = solve_routing_problem(data)
    print(routes, '\n')
    robot_routes = select_robot_nodes(data, routes)
    for index, route in enumerate(routes):
        exclude_nodes = [item for sublist in robot_routes[index] for item in sublist if 0 < sublist.index(item) < len(sublist) - 1]
        routes[index] = [i for i in route if i not in exclude_nodes]
    print('\nRoutes:', routes, '\n')
    # Set Time Window for VRPTW
    robot_durs = [[] for i in range(len(routes))]
    data["time_windows"] = [(0, 640) for i in range(data["num_nodes"])]
    temp_constraint = {
        'nodes': [],
        'min_travel': [],
        'vehicle': []
    }
    for index, group in enumerate(robot_routes):
        print('Vehicle', index)
        for r_route in group:
            r_dur = get_objective_value(data, r_route, ROBOT_SPEED) # 取得 robot_routes 中每個組合的時間
            print('Robot Route:', r_route, ', Duration:', r_dur)
            robot_durs[index].append(r_dur)
            for node in r_route:
                if 0 < r_route.index(node) < len(r_route) - 1:
                    data['demands'][node] = 999  # 將指定點的需求設置為 999
                    data["vehicle_capacities"][index] -= 1  # 修正該 node 負責車輛的容量         
                # 頭尾設定 Time Windows (車輛不能等待超過 10 分鐘)          
                elif r_route.index(node) == 0:
                    cumul_time = get_objective_value(data, routes[index][:routes[index].index(node)+1])
                    node_tw = (cumul_time, cumul_time)
                    data["time_windows"][node] = node_tw
                    print('- Node', node, node_tw)
                elif r_route.index(node) == len(r_route) - 1:
                    node_tw = (cumul_time + r_dur, cumul_time + r_dur + 10)
                    data["time_windows"][node] = node_tw
                    print('- Node', node, node_tw)
            temp_constraint['nodes'].append((r_route[0], r_route[-1]))
            temp_constraint['min_travel'].append(r_dur)
            temp_constraint['vehicle'].append(index)
    vrptw_routes = solve_vrptw(data, temp_constraint)
    if vrptw_routes:
        drone_routes = select_drone_nodes(data, vrptw_routes)
        for index, group in enumerate(drone_routes):
            for r in group:
                vrptw_routes[index].remove(r[1])
        robot_waiting = get_waiting_time(data, vrptw_routes, robot_routes, 'robot')
        drone_waiting = get_waiting_time(data, vrptw_routes, drone_routes, 'drone')
        total_dist, travel_time, waiting_time = get_travelling_time(data, vrptw_routes, drone_waiting, robot_waiting)
    else:
        drone_routes, robot_waiting, drone_waiting, total_dist, travel_time, waiting_time = None, None, None, None, None, None
    '''algorithm ends'''
    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    time_complexity = round(end_time - start_time, 4)
    space_complexity = round(current / 1024, 4)
    VRPRD_result = dict(VRPRD_routes=vrptw_routes, VRPRD_drone_routes=drone_routes, VRPRD_robot_routes=robot_routes, VRPRD_distances=total_dist, VRPRD_time=travel_time, VRPRD_waiting=waiting_time, VRPRD_time_complexity=time_complexity, VRPRD_space_complexity=space_complexity)
    return VRPRD_result