import tkinter as tk
from tkinter import messagebox
import heapq
import time

# Simulation Parameters
SIMULATION_SPEED = 300  
MOVE_COST = 10
WAIT_COST = 2
TURN_PENALTY = 25
BASE_HEALTH = 1000

BOT_COLORS = ["#3498db", "#9b59b6", "#1abc9c", "#e67e22", "#e74c3c"]
BLOCK_COLORS = ["#e74c3c", "#e67e22", "#f39c12", "#f1c40f", "#2ecc71"]
DIRECTIONS = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Right, Down, Left, Up

def evaluate_fitness(path, ideal_distance):
    """Evaluates cost performance metrics for routed paths."""
    if not path: 
        return {"score": 0, "moves": 0, "waits": 0, "turns": 0, "detour": 0}
        
    moves = 0
    waits = 0
    turns = 0
    
    for i in range(1, len(path)):
        prev_bot, prev_dir, _, _ = path[i - 1]
        curr_bot, curr_dir, _, _ = path[i]
        
        if curr_bot == prev_bot:
            if curr_dir != prev_dir:
                turns += 1
            else:
                waits += 1
        else:
            moves += 1
            
    penalty = (moves * MOVE_COST) + (waits * WAIT_COST) + (turns * TURN_PENALTY)
    return {
        "score": max(0, BASE_HEALTH - penalty), 
        "moves": moves, 
        "waits": waits, 
        "turns": turns, 
        "detour": max(0, moves - ideal_distance)
    }

class SolidTrackPlanner:
    def __init__(self, grid_width, grid_height):
        self.max_x = grid_width - 1
        self.max_y = grid_height - 1

    def plan_coordinated_route(self, bot_start, bot_start_dir, block_start, goal_position, global_reservations):
        """Plans space-time path for a single bot-box pair avoiding global footprints."""
        start_state = (bot_start, bot_start_dir, block_start, 0)
        priority_queue = [(0, start_state)]
        cost_so_far = {start_state: 0}
        parent_map = {start_state: None}
        
        iterations = 0

        while priority_queue:
            iterations += 1
            if iterations > 40000: 
                break
                
            _, current = heapq.heappop(priority_queue)
            bot, bot_dir, block, time_step = current

            if block == goal_position:
                path = []
                while current:
                    path.append(current)
                    current = parent_map[current]
                return path[::-1]

            if time_step > 50: 
                continue
            next_time = time_step + 1

            dir_index = DIRECTIONS.index(bot_dir)
            transitions = [
                (bot_dir, WAIT_COST), 
                (DIRECTIONS[(dir_index - 1) % 4], TURN_PENALTY), 
                (DIRECTIONS[(dir_index + 1) % 4], TURN_PENALTY)
            ]
            
            for next_dir, cost_increment in transitions:
                bot_reserved = (bot[0], bot[1], next_time) in global_reservations
                block_reserved = (block[0], block[1], next_time) in global_reservations
                
                if not bot_reserved and not block_reserved:
                    next_state = (bot, next_dir, block, next_time)
                    new_cost = cost_so_far[current] + cost_increment
                    if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                        cost_so_far[next_state] = new_cost
                        h_bot_to_block = abs(bot[0] - block[0]) + abs(bot[1] - block[1])
                        h_block_to_goal = abs(block[0] - goal_position[0]) + abs(block[1] - goal_position[1])
                        heuristic = (h_bot_to_block + h_block_to_goal) * MOVE_COST
                        heapq.heappush(priority_queue, (new_cost + heuristic, next_state))
                        parent_map[next_state] = current

            dir_x, dir_y = bot_dir
            next_bot = (bot[0] + dir_x, bot[1] + dir_y)
            
            if 0 <= next_bot[0] <= self.max_x and 0 <= next_bot[1] <= self.max_y:
                if next_bot == block:
                    next_block = (block[0] + dir_x, block[1] + dir_y)
                    if 0 <= next_block[0] <= self.max_x and 0 <= next_block[1] <= self.max_y:
                        collision_keys = [
                            (next_bot[0], next_bot[1], next_time), 
                            (next_block[0], next_block[1], next_time),
                            (next_bot[0], next_bot[1], bot[0], bot[1], time_step), 
                            (next_block[0], next_block[1], block[0], block[1], time_step)
                        ]
                        if all(key not in global_reservations for key in collision_keys):
                            next_state = (next_bot, bot_dir, next_block, next_time)
                            new_cost = cost_so_far[current] + MOVE_COST
                            if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                                cost_so_far[next_state] = new_cost
                                h_block_to_goal = abs(next_block[0] - goal_position[0]) + abs(next_block[1] - goal_position[1])
                                heuristic = h_block_to_goal * MOVE_COST
                                heapq.heappush(priority_queue, (new_cost + heuristic, next_state))
                                parent_map[next_state] = current
                else:
                    collision_keys = [
                        (next_bot[0], next_bot[1], next_time), 
                        (next_bot[0], next_bot[1], bot[0], bot[1], time_step)
                    ]
                    if all(key not in global_reservations for key in collision_keys):
                        next_state = (next_bot, bot_dir, block, next_time)
                        new_cost = cost_so_far[current] + MOVE_COST
                        if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                            cost_so_far[next_state] = new_cost
                            h_bot_to_block = abs(next_bot[0] - block[0]) + abs(next_bot[1] - block[1])
                            h_block_to_goal = abs(block[0] - goal_position[0]) + abs(block[1] - goal_position[1])
                            heuristic = (h_bot_to_block + h_block_to_goal) * MOVE_COST
                            heapq.heappush(priority_queue, (new_cost + heuristic, next_state))
                            parent_map[next_state] = current
        return []

    def plan_cooperative_escape(self, bot_start, bot_start_dir, target_parking, block_fixed_pos, start_time, global_reservations):
        """Routes a chassis away to a clearance parking cell once payload delivery finishes."""
        start_state = (bot_start, bot_start_dir, block_fixed_pos, start_time)
        priority_queue = [(0, start_state)]
        cost_so_far = {start_state: 0}
        parent_map = {start_state: None}

        while priority_queue:
            _, current = heapq.heappop(priority_queue)
            bot, bot_dir, block, time_step = current

            if bot == target_parking:
                path = []
                while current:
                    path.append(current)
                    current = parent_map[current]
                return path[::-1]

            if time_step > 90:
                continue
            next_time = time_step + 1

            dir_index = DIRECTIONS.index(bot_dir)
            transitions = [(bot_dir, WAIT_COST), (DIRECTIONS[(dir_index - 1) % 4], TURN_PENALTY), (DIRECTIONS[(dir_index + 1) % 4], TURN_PENALTY)]
            
            for next_dir, cost_increment in transitions:
                if (bot[0], bot[1], next_time) not in global_reservations:
                    next_state = (bot, next_dir, block, next_time)
                    new_cost = cost_so_far[current] + cost_increment
                    if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                        cost_so_far[next_state] = new_cost
                        heuristic = (abs(bot[0] - target_parking[0]) + abs(bot[1] - target_parking[1])) * MOVE_COST
                        heapq.heappush(priority_queue, (new_cost + heuristic, next_state))
                        parent_map[next_state] = current

            dir_x, dir_y = bot_dir
            next_bot = (bot[0] + dir_x, bot[1] + dir_y)
            if 0 <= next_bot[0] <= self.max_x and 0 <= next_bot[1] <= self.max_y and next_bot != block_fixed_pos:
                collision_keys = [(next_bot[0], next_bot[1], next_time), (next_bot[0], next_bot[1], bot[0], bot[1], time_step)]
                if all(key not in global_reservations for key in collision_keys):
                    next_state = (next_bot, bot_dir, block, next_time)
                    new_cost = cost_so_far[current] + MOVE_COST
                    if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                        cost_so_far[next_state] = new_cost
                        heuristic = (abs(next_bot[0] - target_parking[0]) + abs(next_bot[1] - target_parking[1])) * MOVE_COST
                        heapq.heappush(priority_queue, (new_cost + heuristic, next_state))
                        parent_map[next_state] = current
        return []

class SolidSwarmPlatform:
    def __init__(self, root):
        self.root = root
        self.root.title("Cooperative Multi-Agent Flight Deck")
        self.root.configure(bg="#121214")

        self.grid_width = 12
        self.grid_height = 12
        self.spacing = 52
        self.margin = 40
        
        self.is_simulating = False
        self.selected_tool = "BOT"
        self.tick_count = 0
        self.occupancy_map = {}
        self.global_paths = {}

        self.setup_ui()
        self.draw_track_network()

    def compute_global_schedules(self):
        self.canvas.delete("path_line")
        self.global_paths.clear()
        for widget in self.score_inner.winfo_children(): 
            widget.destroy()

        entities = self._collect_entities()
        bots, blocks, goals = entities['bots'], entities['blocks'], entities['goals']
        if not bots or not blocks or not goals:
            tk.Label(self.score_inner, text="Place elements on tracks...", fg="#555", bg="#1e1e22").pack()
            return

        block_to_goal_assignments = {}
        remaining_goals = list(goals)
        for block in sorted(blocks, key=lambda bk: min([abs(bk['pos'][0] - g['pos'][0]) + abs(bk['pos'][1] - g['pos'][1]) for g in goals])):
            if not remaining_goals: 
                break
            best_goal = min(remaining_goals, key=lambda g: abs(block['pos'][0] - g['pos'][0]) + abs(block['pos'][1] - g['pos'][1]))
            block_to_goal_assignments[block['id']] = best_goal
            remaining_goals.remove(best_goal)

        bot_tasks = {}
        available_blocks = [b for b in blocks if b['id'] in block_to_goal_assignments]
        all_possible_pairings = []
        
        for bot in bots:
            for block in available_blocks:
                goal = block_to_goal_assignments[block['id']]
                dist_bot_to_block = abs(bot['pos'][0] - block['pos'][0]) + abs(bot['pos'][1] - block['pos'][1])
                dist_block_to_goal = abs(block['pos'][0] - goal['pos'][0]) + abs(block['pos'][1] - goal['pos'][1])
                all_possible_pairings.append((dist_bot_to_block + dist_block_to_goal, bot, block))
        
        all_possible_pairings.sort(key=lambda x: x[0])
        assigned_bots = set()
        assigned_blocks = set()
        
        for cost, bot, block in all_possible_pairings:
            if bot['id'] not in assigned_bots and block['id'] not in assigned_blocks:
                bot_tasks[bot['id']] = {'block': block, 'goal': block_to_goal_assignments[block['id']]}
                assigned_bots.add(bot['id'])
                assigned_blocks.add(block['id'])

        global_reservations = {}
        planner = SolidTrackPlanner(self.grid_width, self.grid_height)

        for t in range(100):
            for b in bots:
                global_reservations[(b['pos'][0], b['pos'][1], t)] = b['id']
            for bk in blocks:
                global_reservations[(bk['pos'][0], bk['pos'][1], t)] = f"block_{bk['id']}"

        execution_order = sorted(bots, key=lambda b: (abs(b['pos'][0] - bot_tasks[b['id']]['block']['pos'][0]) + 
                                                      abs(b['pos'][1] - bot_tasks[b['id']]['block']['pos'][1])) if b['id'] in bot_tasks else -1, reverse=True)

        bot_id_to_display_idx = {b['id']: b['color_idx'] for b in bots}

        for bot in execution_order:
            display_idx = bot_id_to_display_idx[bot['id']]
            task = bot_tasks.get(bot['id'])
            
            if not task:
                self.global_paths[bot['id']] = [(bot['pos'], (1, 0), bot['pos'], 0)]
                self.render_failed_card(display_idx)
                continue

            local_reservations = {k: v for k, v in global_reservations.items() if v != bot['id'] and v != f"block_{task['block']['id']}" and v != f"block_{bot['id']}"}
            delivery_path = planner.plan_coordinated_route(bot['pos'], (1, 0), task['block']['pos'], task['goal']['pos'], local_reservations)
            
            if delivery_path:
                # Clear out initial space-time footprints before committing active path steps
                keys_to_remove = [k for k, v in global_reservations.items() if v == bot['id'] or v == f"block_{task['block']['id']}" or v == f"block_{bot['id']}"]
                for k in keys_to_remove:
                    del global_reservations[k]

                prev_bot, prev_block = bot['pos'], task['block']['pos']
                for (curr_bot, curr_dir, curr_block, time_step) in delivery_path:
                    global_reservations[(curr_bot[0], curr_bot[1], time_step)] = bot['id']
                    global_reservations[(curr_block[0], curr_block[1], time_step)] = f"block_{bot['id']}"
                    if curr_bot != prev_bot: 
                        global_reservations[(prev_bot[0], prev_bot[1], curr_bot[0], curr_bot[1], time_step - 1)] = bot['id']
                    if curr_block != prev_block: 
                        global_reservations[(prev_block[0], prev_block[1], curr_block[0], curr_block[1], time_step - 1)] = f"block_{bot['id']}"
                    prev_bot, prev_block = curr_bot, curr_block

                # Handle tracking variables at completion spot
                final_bot, final_dir, final_block, final_time = delivery_path[-1]
                
                # Lock the dropped box position permanently into the future matrix
                for future_time in range(final_time, 100):
                    global_reservations[(final_block[0], final_block[1], future_time)] = f"block_{bot['id']}"

                # UNDERSTAND IF IT IS IN SOMEONE ELSE'S PATH
                is_in_others_path = False
                for t in range(final_time + 1, 100):
                    if (final_bot[0], final_bot[1], t) in global_reservations:
                        is_in_others_path = True
                        break

                escape_path = []
                if is_in_others_path:
                    # IF YES: Find a safe parking cell sorted by minimum Manhattan distance (minimum blocks away)
                    candidates = []
                    for x in range(self.grid_width):
                        for y in range(self.grid_height):
                            if (x, y) != final_block and (x, y) != final_bot:
                                # A cell is only safe if no other agent reserves it from this point forward
                                safe_cell = True
                                for t in range(final_time + 1, 100):
                                    if (x, y, t) in global_reservations:
                                        safe_cell = False
                                        break
                                if safe_cell:
                                    d = abs(final_bot[0] - x) + abs(final_bot[1] - y)
                                    candidates.append((d, (x, y)))
                    
                    # Sort candidates by proximity to move minimum blocks to get out of the way
                    candidates.sort(key=lambda item: item[0])
                    
                    for d, spot in candidates:
                        path_attempt = planner.plan_cooperative_escape(final_bot, final_dir, spot, final_block, final_time, global_reservations)
                        if path_attempt:
                            escape_path = path_attempt
                            break

                if escape_path:
                    # Combine original task delivery tracking with cooperative parking steps
                    combined_path = delivery_path + escape_path[1:]
                    self.global_paths[bot['id']] = combined_path
                    
                    # Log escape route configurations into master reservation map
                    for (c_bot, c_dir, _, t_step) in escape_path:
                        global_reservations[(c_bot[0], c_bot[1], t_step)] = bot['id']
                    end_bot, _, _, end_time = escape_path[-1]
                    for future_time in range(end_time + 1, 100):
                        global_reservations[(end_bot[0], end_bot[1], future_time)] = bot['id']
                else:
                    # IF NO (or if escape fallback fails): Do nothing, stay right there
                    self.global_paths[bot['id']] = delivery_path
                    for future_time in range(final_time + 1, 100):
                        global_reservations[(final_bot[0], final_bot[1], future_time)] = bot['id']

                ideal = abs(bot['pos'][0] - task['block']['pos'][0]) + abs(bot['pos'][1] - task['block']['pos'][1]) + \
                        abs(task['block']['pos'][0] - task['goal']['pos'][0]) + abs(task['block']['pos'][1] - task['goal']['pos'][1])
                self.render_report_card(bot['id'], display_idx, evaluate_fitness(delivery_path, ideal))
                self.draw_route_line(display_idx, delivery_path)
            else:
                self.render_failed_card(display_idx)

    def render_report_card(self, bot_id, index, report):
        color = BOT_COLORS[index % len(BOT_COLORS)]
        card = tk.Frame(self.score_inner, bg="#222226", bd=1, relief="solid", padx=6, pady=6)
        card.pack(fill='x', pady=3)
        top_bar = tk.Frame(card, bg="#222226")
        top_bar.pack(fill='x')
        tk.Label(top_bar, text=f" AGENT #{index + 1}", fg=color, bg="#222226", font=('Arial', 9, 'bold')).pack(side='left')
        tk.Label(top_bar, text=f"FITNESS: {report['score']}", fg="#2ecc71" if report['score'] > 750 else "#f1c40f", bg="#222226", font=('Courier', 9, 'bold')).pack(side='right')
        tk.Label(card, text=f"Moves: {report['moves']} | Waits: {report['waits']} | Turns: {report['turns']}", fg="#aaa", bg="#222226", font=('Arial', 8)).pack(anchor='w', pady=2)

    def render_failed_card(self, index):
        card = tk.Frame(self.score_inner, bg="#2c1e1e", bd=1, relief="solid", padx=6, pady=6)
        card.pack(fill='x', pady=3)
        tk.Label(card, text=f" AGENT #{index + 1} BLOCKED / IDLE", fg="#e74c3c", bg="#2c1e1e", font=('Arial', 9, 'bold')).pack(anchor='w')

    def draw_route_line(self, index, path):
        points = []
        for (bot_position, _, _, _) in path:
            points.extend([self.margin + bot_position[0] * self.spacing, self.margin + bot_position[1] * self.spacing])
        if len(points) > 2:
            self.canvas.create_line(*points, fill=BOT_COLORS[index % len(BOT_COLORS)], width=2, dash=(5, 3), arrow=tk.LAST, tags=("path_line", "decorative"))

    def run_simulation_tick(self):
        if not self.is_simulating: 
            return
        moves_active = False
        self.tick_count += 1

        for bot_id, timeline in list(self.global_paths.items()):
            if self.tick_count < len(timeline):
                moves_active = True
                curr_bot, curr_dir, curr_block, _ = timeline[self.tick_count - 1]
                next_bot, next_dir, next_block, _ = timeline[self.tick_count]

                delta_x = (next_bot[0] - curr_bot[0]) * self.spacing
                delta_y = (next_bot[1] - curr_bot[1]) * self.spacing
                for element in self.canvas.find_withtag(bot_id):
                    self.canvas.move(element, delta_x, delta_y)

                center_x = self.margin + next_bot[0] * self.spacing
                center_y = self.margin + next_bot[1] * self.spacing
                nose = self.canvas.find_withtag(f"nose_{bot_id}")
                if nose: 
                    indicator_x = center_x + (next_dir[0] * 8)
                    indicator_y = center_y + (next_dir[1] * 8)
                    self.canvas.coords(nose[0], indicator_x - 3, indicator_y - 3, indicator_x + 3, indicator_y + 3)

                if curr_block != next_block:
                    old_x = self.margin + curr_block[0] * self.spacing
                    old_y = self.margin + curr_block[1] * self.spacing
                    for item_id in self.canvas.find_overlapping(old_x - 2, old_y - 2, old_x + 2, old_y + 2):
                        tags = self.canvas.gettags(item_id)
                        if "block" in tags:
                            group_tag = next((t for t in tags if t.startswith("group_")), None)
                            if group_tag: 
                                self.canvas.move(group_tag, (next_block[0] - curr_block[0]) * self.spacing, (next_block[1] - curr_block[1]) * self.spacing)

        if moves_active:
            self.root.after(SIMULATION_SPEED, self.run_simulation_tick)
        else:
            self.toggle_simulation()

    def toggle_simulation(self):
        if self.is_simulating:
            self.is_simulating = False
            self.run_btn.config(text=" EXECUTE SCHEDULE", bg="#27ae60")
        else:
            self.compute_global_schedules()
            if not self.global_paths: 
                return
            self.tick_count = 0
            self.is_simulating = True
            self.run_btn.config(text=" STOP RUN", bg="#c0392b")
            self.run_simulation_tick()

    def on_canvas_click(self, event):
        if self.is_simulating: 
            return
        grid_x = round((self.canvas.canvasx(event.x) - self.margin) / self.spacing)
        grid_y = round((self.canvas.canvasy(event.y) - self.margin) / self.spacing)
        if not (0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height): 
            return
        position = (grid_x, grid_y)

        if self.selected_tool == "DELETE":
            pixel_x = self.margin + grid_x * self.spacing
            pixel_y = self.margin + grid_y * self.spacing
            for target in self.canvas.find_overlapping(pixel_x - 3, pixel_y - 3, pixel_x + 3, pixel_y + 3):
                tags = self.canvas.gettags(target)
                if "track" in tags or "decorative" in tags: 
                    continue
                group_tag = next((t for t in tags if t.startswith("group_")), None)
                if group_tag:
                    self.canvas.delete(group_tag)
                    self.occupancy_map.pop(position, None)
                    self.compute_global_schedules()
                    return
            return

        if position in self.occupancy_map: 
            return
            
        group_tag = f"group_{grid_x}_{grid_y}_{int(time.time() * 1000)}"
        pixel_x = self.margin + grid_x * self.spacing
        pixel_y = self.margin + grid_y * self.spacing
        radius = 11
        item_index = len(self._collect_entities()['bots']) if self.selected_tool == "BOT" else len(self.occupancy_map)

        if self.selected_tool == "BOT":
            self.canvas.create_oval(pixel_x - radius, pixel_y - radius, pixel_x + radius, pixel_y + radius, fill=BOT_COLORS[item_index % len(BOT_COLORS)], outline="#fff", tags=("bot", group_tag, f"chassis_{group_tag}"))
            self.canvas.create_oval(pixel_x + 8 - 3, pixel_y - 3, pixel_x + 8 + 3, pixel_y + 3, fill="#fff", tags=("bot", "decorative", group_tag, f"nose_{group_tag}"))
        elif self.selected_tool == "BLOCK":
            self.canvas.create_rectangle(pixel_x - radius, pixel_y - radius, pixel_x + radius, pixel_y + radius, fill=BLOCK_COLORS[item_index % len(BLOCK_COLORS)], outline="#fff", tags=("block", group_tag))
        elif self.selected_tool == "GOAL":
            self.canvas.create_line(pixel_x - radius, pixel_y - radius, pixel_x + radius, pixel_y + radius, fill="#2ecc71", width=3, tags=("goal", group_tag))
            self.canvas.create_line(pixel_x + radius, pixel_y - radius, pixel_x - radius, pixel_y + radius, fill="#2ecc71", width=3, tags=("goal", group_tag))

        self.occupancy_map[position] = group_tag
        self.compute_global_schedules()

    def _collect_entities(self):
        results = {'bots': [], 'blocks': [], 'goals': []}
        seen_groups = set()
        for item_id in self.canvas.find_all():
            tags = self.canvas.gettags(item_id)
            group = next((t for t in tags if t.startswith("group_")), None)
            if not group or group in seen_groups: 
                continue
            coords = self.canvas.coords(item_id)
            grid_position = (round(((coords[0] + coords[-2]) / 2 - self.margin) / self.spacing), round(((coords[1] + coords[-1]) / 2 - self.margin) / self.spacing))
            for key in results:
                if key[:-1] in tags:
                    color = self.canvas.itemcget(item_id, "fill")
                    display_idx = BOT_COLORS.index(color) if color in BOT_COLORS else 0
                    results[key].append({'id': group, 'pos': grid_position, 'color_idx': display_idx})
                    seen_groups.add(group)
                    break
        return results

    def setup_ui(self):
        side_panel = tk.Frame(self.root, width=260, bg="#1a1a1e", padx=10, pady=10)
        side_panel.pack(side='left', fill='y')
        side_panel.pack_propagate(False)

        self.run_btn = tk.Button(side_panel, text=" EXECUTE SCHEDULE", command=self.toggle_simulation, bg="#27ae60", fg="white", font=('Arial', 9, 'bold'), relief="flat")
        self.run_btn.pack(fill='x', pady=4)
        tk.Button(side_panel, text=" CLEAR GRID NETWORK", command=self.clear_grid, bg="#c0392b", fg="white", relief="flat").pack(fill='x', pady=2)
        tk.Button(side_panel, text=" ERASER TOOL", command=lambda: self.select_tool("DELETE"), bg="#d35400", fg="white", relief="flat").pack(fill='x', pady=4)

        tk.Label(side_panel, text="INTERSECTION WORKER MODES", fg="#666", bg="#1a1a1e", font=('Arial', 8, 'bold')).pack(pady=(15, 2))
        self.b1 = tk.Button(side_panel, text=" TRACK ROBOT NODE", bg="#3498db", fg="white", command=lambda: self.select_tool("BOT"), relief="flat")
        self.b1.pack(fill='x', pady=2)
        self.b2 = tk.Button(side_panel, text=" SOLID CARGO BOX", bg="#e67e22", fg="white", command=lambda: self.select_tool("BLOCK"), relief="flat")
        self.b2.pack(fill='x', pady=2)
        self.b3 = tk.Button(side_panel, text=" LINE TARGET GOAL", bg="#2ecc71", fg="white", command=lambda: self.select_tool("GOAL"), relief="flat")
        self.b3.pack(fill='x', pady=2)

        score_frame = tk.LabelFrame(side_panel, text=" RUNTIME HEALTH MONITOR ", bg="#1a1a1e", fg="#00ffcc", font=('Arial', 8, 'bold'), padx=4, pady=4)
        score_frame.pack(fill='both', expand=True, pady=15)
        self.score_inner = tk.Frame(score_frame, bg="#1a1a1e")
        self.score_inner.pack(fill='both', expand=True)

        self.canvas = tk.Canvas(self.root, bg="#111113", highlightthickness=0)
        self.canvas.pack(side='right', expand=True, fill='both')
        self.canvas.bind("<Button-1>", self.on_canvas_click)

    def draw_track_network(self):
        width = (self.grid_width - 1) * self.spacing
        height = (self.grid_height - 1) * self.spacing
        for i in range(self.grid_width): 
            self.canvas.create_line(self.margin + i * self.spacing, self.margin, self.margin + i * self.spacing, self.margin + height, fill="#25252b", width=2, tags="track")
        for j in range(self.grid_height): 
            self.canvas.create_line(self.margin, self.margin + j * self.spacing, self.margin + width, self.margin + j * self.spacing, fill="#25252b", width=2, tags="track")
        self.canvas.tag_lower("track")

    def select_tool(self, tool):
        self.selected_tool = tool
        for btn, name in [(self.b1, "BOT"), (self.b2, "BLOCK"), (self.b3, "GOAL")]:
            btn.config(bd=3 if tool == name else 1, relief="sunken" if tool == name else "flat")

    def clear_grid(self):
        self.canvas.delete("all")
        self.draw_track_network()
        self.occupancy_map.clear()
        self.compute_global_schedules()

if __name__ == "__main__":
    window = tk.Tk()
    platform = SolidSwarmPlatform(window)
    window.mainloop()