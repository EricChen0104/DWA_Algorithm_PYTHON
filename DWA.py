import pygame
import math
import numpy as np

class Config:
    def __init__(self):

        self.max_speed = 1.0  
        self.min_speed = -0.5 
        self.max_omega = 40.0 * math.pi / 180.0  
        self.max_accel = 0.2  
        self.max_omega_accel = 40.0 * math.pi / 180.0  

        self.v_resolution = 0.01  
        self.omega_resolution = 0.1 * math.pi / 180.0  
        self.dt = 0.5  
        self.predict_time = 3.0  
        self.robot_radius = 0.5  

        self.to_goal_cost_gain = 0.1
        self.speed_cost_gain = 1.0 
        self.obstacle_cost_gain = 0.5

class RobotState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0):
        self.x = x          
        self.y = y          
        self.yaw = yaw      
        self.v = v          
        self.omega = omega  

def dwa_control(state, config, goal, obstacles, screen):

    v_min = state.v - config.max_accel * config.dt
    v_max = state.v + config.max_accel * config.dt
    omega_min = state.omega - config.max_omega_accel * config.dt
    omega_max = state.omega + config.max_omega_accel * config.dt

    dw = {
        "v_min": max(config.min_speed, v_min),
        "v_max": min(config.max_speed, v_max),
        "omega_min": max(-config.max_omega, omega_min),
        "omega_max": min(config.max_omega, omega_max),
    }

    best_cost = float("inf")
    best_u = [0.0, 0.0]

    best_trajectory = np.array([[state.x, state.y]])

    for v in np.arange(dw["v_min"], dw["v_max"], config.v_resolution):
        for omega in np.arange(dw["omega_min"], dw["omega_max"], config.omega_resolution):
            trajectory = predict_trajectory(state, v, omega, config)

            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            obstacle_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, obstacles, config)
            
            final_cost = to_goal_cost + speed_cost + obstacle_cost

            if final_cost != float("inf"):
                pygame_traj_points = [to_pygame(pos) for pos in trajectory[:, 0:2]]
                pygame.draw.lines(screen, LIGHT_GRAY, False, pygame_traj_points, 1)

            if final_cost < best_cost:
                best_cost = final_cost
                best_u = [v, omega]
                best_trajectory = trajectory
    
    return best_u, best_trajectory

def predict_trajectory(state, v, omega, config):
    current_state = np.array([state.x, state.y, state.yaw, state.v, state.omega])
    trajectory = np.array(current_state)
    time = 0
    
    while time <= config.predict_time:
        current_state[2] += omega * config.dt  # yaw
        current_state[0] += v * math.cos(current_state[2]) * config.dt  # x
        current_state[1] += v * math.sin(current_state[2]) * config.dt  # y
        current_state[3] = v  # v
        current_state[4] = omega  # omega
        
        trajectory = np.vstack((trajectory, current_state))
        time += config.dt
    
    return trajectory

def calc_obstacle_cost(trajectory, obstacles, config):
    min_dist = float("inf")
    for i in range(len(trajectory)):
        for ox, oy, radius in obstacles:
            dist = math.sqrt((trajectory[i, 0] - ox)**2 + (trajectory[i, 1] - oy)**2)
            
            if dist <= config.robot_radius + radius:
                return float("inf")  
            
            min_dist = min(min_dist, dist)

    return 1.0 / min_dist  

def calc_to_goal_cost(trajectory, goal):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2] 
    
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
    return cost

def motion(state, u, dt):
    state.yaw += u[1] * dt
    state.x += u[0] * math.cos(state.yaw) * dt
    state.y += u[0] * math.sin(state.yaw) * dt
    state.v = u[0]
    state.omega = u[1]
    return state


WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (200, 200, 200)
LIGHT_GRAY = (220, 220, 220)

WIDTH, HEIGHT = 800, 600
PPM = 20  

def to_pygame(pos):
    return int(pos[0] * PPM + WIDTH / 2), int(-pos[1] * PPM + HEIGHT / 2)

def from_pygame(pos):
    return (pos[0] - WIDTH / 2) / PPM, -(pos[1] - HEIGHT / 2) / PPM

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Dynamic Window Approach (DWA)")
    clock = pygame.time.Clock()

    config = Config()
    state = RobotState(x=-10.0, y=-5.0, yaw=math.pi / 4.0, v=0.0, omega=0.0)
    
    goal = [10.0, 10.0]

    obstacles = [
        (0, 2, 1),
        (2, 4, 1),
        (5, 5, 1),
        (8, 7, 1),
        (6, 10, 1),
        (10, 4, 1),
        (-2, 8, 1.5)
    ]

    best_trajectory_to_draw = None
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    pos = pygame.mouse.get_pos()
                    goal = list(from_pygame(pos))
                elif event.button == 3:
                    pos = pygame.mouse.get_pos()
                    obs_pos = list(from_pygame(pos))
                    obstacles.append((obs_pos[0], obs_pos[1], 1.0)) 

        screen.fill(WHITE)

        u, best_trajectory_to_draw = dwa_control(state, config, goal, obstacles, screen)
        
        state = motion(state, u, config.dt)

        dist_to_goal = math.sqrt((state.x - goal[0])**2 + (state.y - goal[1])**2)
        if dist_to_goal <= config.robot_radius:
            print("Goal reached!")
            running = False 

        pygame.draw.circle(screen, GREEN, to_pygame(goal), 10)

        for ox, oy, r in obstacles:
            pygame.draw.circle(screen, BLACK, to_pygame((ox, oy)), int(r * PPM))

        if best_trajectory_to_draw is not None:
            pygame_best_traj_points = [to_pygame(pos) for pos in best_trajectory_to_draw[:, 0:2]]
            
            if len(pygame_best_traj_points) >= 2:
                pygame.draw.lines(screen, RED, False, pygame_best_traj_points, 2)
            elif len(pygame_best_traj_points) == 1:
                pygame.draw.circle(screen, RED, pygame_best_traj_points[0], 2)


        robot_pos = to_pygame((state.x, state.y))
        pygame.draw.circle(screen, BLUE, robot_pos, int(config.robot_radius * PPM))
        
        heading_line_end = (
            state.x + config.robot_radius * math.cos(state.yaw),
            state.y + config.robot_radius * math.sin(state.yaw)
        )
        pygame.draw.line(screen, RED, robot_pos, to_pygame(heading_line_end), 3)

        pygame.display.flip()
        clock.tick(30) 

    pygame.quit()

if __name__ == '__main__':
    main()