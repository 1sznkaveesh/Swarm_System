import heapq


def heuristic(a, b):
    """Calculates Manhattan distance from the box's current position to the goal."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def shortest_push(grid, robot_start_pos, robot_start_dir, box_start, goal):
    """Finds the shortest path of actions (moves and rotations) to push a box to a goal.

    Each action (moving forward, turning left, turning right) costs 1 tick.
    """
    # Expanded state tracking: (robot_position, robot_direction, box_position)
    start = (robot_start_pos, robot_start_dir, box_start)

    frontier = []
    # Heap entries store: (priority, state)
    heapq.heappush(frontier, (0, start))

    cost = {start: 0}
    parent = {}

    goal_state = None

    # Clockwise order of headings: Right, Down, Left, Up
    DIRECTIONS = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    while frontier:
        _, state = heapq.heappop(frontier)
        robot_pos, robot_dir, box_pos = state

        # The goal is reached when the box lands on the goal target
        if box_pos == goal:
            goal_state = state
            break

        # --- ACTION 1: ROTATE LEFT (Costs 1 Tick) ---
        current_idx = DIRECTIONS.index(robot_dir)
        left_dir = DIRECTIONS[(current_idx - 1) % 4]
        state_left = (robot_pos, left_dir, box_pos)
        cost_left = cost[state] + 1

        if state_left not in cost or cost_left < cost[state_left]:
            cost[state_left] = cost_left
            priority = cost_left + heuristic(box_pos, goal)
            heapq.heappush(frontier, (priority, state_left))
            parent[state_left] = state

        # --- ACTION 2: ROTATE RIGHT (Costs 1 Tick) ---
        right_dir = DIRECTIONS[(current_idx + 1) % 4]
        state_right = (robot_pos, right_dir, box_pos)
        cost_right = cost[state] + 1

        if state_right not in cost or cost_right < cost[state_right]:
            cost[state_right] = cost_right
            priority = cost_right + heuristic(box_pos, goal)
            heapq.heappush(frontier, (priority, state_right))
            parent[state_right] = state

        # --- ACTION 3: MOVE FORWARD / PUSH (Costs 1 Tick) ---
        # The robot can only move along the axis it is currently facing
        dx, dy = robot_dir
        new_robot = (robot_pos[0] + dx, robot_pos[1] + dy)

        # Check if the forward cell is within grid boundaries and clear of static obstacles
        if grid.in_bounds(new_robot) and new_robot not in grid.obstacles:
            new_box = box_pos
            is_move_valid = True

            # If the robot steps directly into the box, it attempts a push
            if new_robot == box_pos:
                push_target = (box_pos[0] + dx, box_pos[1] + dy)

                # Verify the box can actually slide into the next cell ahead
                if grid.in_bounds(push_target) and push_target not in grid.obstacles:
                    new_box = push_target
                else:
                    is_move_valid = False  # Blocked: Box hits a wall or obstacle

            if is_move_valid:
                state_forward = (new_robot, robot_dir, new_box)
                cost_forward = cost[state] + 1

                if state_forward not in cost or cost_forward < cost[state_forward]:
                    cost[state_forward] = cost_forward
                    priority = cost_forward + heuristic(new_box, goal)
                    heapq.heappush(frontier, (priority, state_forward))
                    parent[state_forward] = state

    if goal_state is None:
        return []

    # Reconstruct the sequence of states
    path = []
    s = goal_state

    while s in parent:
        path.append(s)
        s = parent[s]

    path.reverse()
    return path


def assign_tasks(robots, boxes):
    """Greedily assigns tasks to robots based on closest Manhattan distance."""
    assignments = {}
    remaining = boxes[:]

    for robot in robots:
        if not remaining:
            break

        best = min(
            remaining,
            key=lambda b: abs(robot.pos[0] - b.pos[0]) + abs(robot.pos[1] - b.pos[1]),
        )

        assignments[robot.id] = best
        remaining.remove(best)

    return assignments