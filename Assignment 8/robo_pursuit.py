import pygame
import pygame.gfxdraw
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import csv
import random
from collections import deque, defaultdict
from itertools import product

# ------------------ Constants ------------------
WIDTH, HEIGHT = 800, 600
FPS = 60
SHOW_VISUAL = True  # Set to False for silent simulation

# ------------------ Target Class ------------------
class Target:
    def __init__(self, x, y, speed, wave_amplitude=0, wave_length=1, mode='sinusoidal'):
        self.x = x
        self.y = y
        self.init_y = y
        self.speed = speed
        self.amp = wave_amplitude
        self.wavelength = wave_length
        self.direction = 1
        self.mode = mode
        self.trajectory = []
        self.captured = False

    def update(self, t):
        if self.captured:
            return

        self.x += self.direction * self.speed / FPS
        # self.y += self.direction * self.speed[1] / FPS

        if self.mode == 'sinusoidal':
            self.y = self.init_y + self.amp * math.sin(2 * math.pi * self.x / self.wavelength)
        elif self.mode == 'linear':
            self.y = self.init_y  # constant y

        if self.x > WIDTH - 20 or self.x < 20:
            self.direction *= -1

        self.trajectory.append((self.x, self.y))


# ------------------ Agent Class ------------------
class Agent:
    def __init__(self, x, y, speed, strategy="simple", theta_set_deg=30, 
                Kp=2.0, Ki=0.5, Kd=4, camouflage_point=(0, 0), **kwargs):
        self.x = x
        self.y = y
        self.heading = 0
        self.speed = speed
        self.strategy = strategy
        self.theta_set = math.radians(theta_set_deg)  # for constant bearing
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd 
        self.integral_error = 0 # 
        self.trajectory = []
        self.captured = False
        self.camouflage_point = camouflage_point
        self.last_theta_r = None  # for derivative calculation
        self.theta_r_log = []  # for logging theta_r over time
        self.angle_noise_std = kwargs.get("angle_noise_std", 0)

    def update(self, target_x, target_y):
        if self.captured:
            return

        dx = target_x - self.x
        dy = target_y - self.y

        if self.strategy in ["simple", "constant_bearing", "parallel_navigation", "proportional_navigation"]:
            if hasattr(self, 'angle_noise_std'):
                dx += np.random.normal(0, self.angle_noise_std)
                dy += np.random.normal(0, self.angle_noise_std)

        theta_r = math.atan2(dy, dx)
        dt = 1 / FPS

        if self.strategy == "simple":
            error = theta_r - self.heading
            error = (error + math.pi) % (2 * math.pi) - math.pi
            self.heading += (self.Kp * error + self.Ki * self.integral_error) * dt

        elif self.strategy == "constant_bearing":
            error = theta_r - self.heading - self.theta_set
            error = (error + math.pi) % (2 * math.pi) - math.pi
            self.integral_error += error * dt
            self.heading += (self.Kp * error + self.Ki * self.integral_error) * dt

        elif self.strategy == "proportional_navigation":
            if self.last_theta_r is not None:
                d_theta_r = (theta_r - self.last_theta_r) / dt
                d_theta_r = (d_theta_r + math.pi) % (2 * math.pi) - math.pi
                self.heading += self.Kd * d_theta_r * dt # Kd is the navigation constant
            self.last_theta_r = theta_r

        elif self.strategy == "parallel_navigation":
            if self.last_theta_r is not None:
                d_theta_r = (theta_r - self.last_theta_r) / dt
                d_theta_r = (d_theta_r + math.pi) % (2 * math.pi) - math.pi
                self.heading += self.Kd * d_theta_r * dt 
            self.last_theta_r = theta_r
            self.theta_r_log.append(theta_r)  

        elif self.strategy == "motion_camouflage":
            # Line between camouflage point and target
            x_c, y_c = self.camouflage_point
            x_t, y_t = target_x, target_y

            # Compute lambda along the line (projection scalar)
            dx = x_t - x_c
            dy = y_t - y_c
            if dx == 0 and dy == 0:
                desired_x, desired_y = x_t, y_t
            else:
                # Find lambda such that agent lies on the line between target and camo point
                vec_ct = np.array([dx, dy])
                vec_ca = np.array([self.x - x_c, self.y - y_c])
                lambda_proj = np.dot(vec_ct, vec_ca) / (np.dot(vec_ct, vec_ct))

                # Clamp lambda between 0 and 1 (stay between camo and target)
                lambda_proj = max(0.0, min(1.0, lambda_proj))

                # Desired point on that line
                desired_x = x_c + lambda_proj * dx
                desired_y = y_c + lambda_proj * dy

            # Steer towards the desired point
            theta_goal = math.atan2(desired_y - self.y, desired_x - self.x)
            error = theta_goal - self.heading
            error = (error + math.pi) % (2 * math.pi) - math.pi
            self.heading += (self.Kp * error + self.Ki * self.integral_error) * dt

        # Update position
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt
        self.trajectory.append((self.x, self.y))

    def draw(self, screen):
        body_width, body_length = 24, 36  # Larger
        angle_deg = -math.degrees(self.heading)
        surf = pygame.Surface((body_length, body_width), pygame.SRCALPHA)
        surf.fill((255, 215, 0))  # Yellow body
        pygame.draw.rect(surf, (0, 102, 204), (0, 0, body_length, 6))  # Blue sensor strip
        pygame.draw.circle(surf, (255, 255, 255), (body_length - 4, body_width // 2), 3)  # White camera

        rotated = pygame.transform.rotate(surf, angle_deg)
        rect = rotated.get_rect(center=(self.x, self.y))
        screen.blit(rotated, rect)


# ------------------ Simulation Controller ------------------

def run_single_simulation(strat, agent_start=(100, 300), target_start=(300, 300), target_path='sinusoidal', visualize=False,
                          duration=5, frame_delay=0, theta_CB=30, Kp=2.0, Ki=0.5, Kd=4, agent_kwargs=None):
    if agent_kwargs is None:
        agent_kwargs = {}

    agent = Agent(*agent_start, speed=100, strategy=strat, theta_set_deg=theta_CB, Kp=Kp, Ki=Ki, Kd=Kd,
                  camouflage_point=agent_start, **agent_kwargs)

    if visualize:
        pygame.init()
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        font = pygame.font.SysFont("arial", 24)
    else:
        screen = None
        font = None

    if frame_delay > 0:
        target_history = deque(maxlen=frame_delay + 1)

    clock = pygame.time.Clock()

    target = Target(*target_start, speed=70, wave_amplitude=60, wave_length=120, mode=target_path)

    capture_radius = 15
    t = 0
    time_to_capture = None
    running = True
    final_elapsed_time = None

    max_steps = int(duration * FPS)
    while running and t < max_steps:
        if visualize:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

        target.update(t / FPS)
        if frame_delay > 0:
            target_history.append((target.x, target.y))
            if len(target_history) > frame_delay:
                delayed_x, delayed_y = target_history[0]
            else:
                delayed_x, delayed_y = target.x, target.y
        else:
            delayed_x, delayed_y = target.x, target.y

        agent.update(delayed_x, delayed_y)

        dx, dy = target.x - agent.x, target.y - agent.y
        dist = math.hypot(dx, dy)
        if dist < capture_radius and not agent.captured:
            agent.captured = True
            running = False
            target.captured = True
            time_to_capture = t / FPS
            final_elapsed_time = time_to_capture

        if visualize:
            screen.fill((240, 240, 240))
            pygame.draw.circle(screen, (200, 0, 0), (int(target.x), int(target.y)), 12)
            agent.draw(screen)

            elapsed_time = t / FPS
            display_time = final_elapsed_time if final_elapsed_time else elapsed_time

            time_display = f"Time: {display_time:.2f}s"
            if time_to_capture:
                time_display += f" | Time to capture = {time_to_capture:.2f}s"

            text = font.render(time_display, True, (0, 0, 0))
            screen.blit(text, (20, 20))
            pygame.display.flip()

        t += 1
        clock.tick(FPS)

    # Auto-close delay after simulation ends
    if visualize:
        pygame.time.wait(2000)  # Wait 2 seconds
        pygame.quit()

    success = time_to_capture is not None

    print(f"Strategy: {strat} | Time to capture: {time_to_capture}s | Success: {success}")

    return {
        # "agent_traj": agent.trajectory,
        # "target_traj": target.trajectory,
        "time_to_capture": time_to_capture,
        "agent_start": agent_start,
        "target_start": target_start,
        "success": success,
        "strategy": agent.strategy,
        "theta_r_log": agent.theta_r_log,
        "camouflage_point": agent.camouflage_point,
        "frame_delay": frame_delay,
        "target_path": target_path,
        "angle_noise_std": getattr(agent, 'angle_noise_std', 0)
    }


def run_batch_simulations(strat, scenarios, visual=False, frame_delay=0, theta_CB=30, 
                            Kp=2.0, Ki=0.5, Kd=4, target_path='sinusoidal', duration=5):
    results = []
    for i, (a_start, t_start) in enumerate(scenarios):
        print(f"\nRunning scenario {i+1}: Agent@{a_start}, Target@{t_start}")
        result = run_single_simulation(strat, agent_start=a_start, target_start=t_start, Kp=Kp, Ki=Ki, Kd=Kd,
                                       visualize=visual, frame_delay=frame_delay, target_path=target_path,
                                       duration=duration, theta_CB=theta_CB)
        results.append(result)
        print(f"Run {i+1} | Time to capture: {result['time_to_capture']}s | Success: {result['success']}")
    return results

def plot_motion_camouflage_lines(result):
    agent_traj = np.array(result["agent_traj"])
    target_traj = np.array(result["target_traj"])
    camo = np.array(result["camouflage_point"])
    num_points = len(agent_traj)

    plt.figure(figsize=(10, 8))

    # Plot camouflage lines (dashed green)
    for i in range(0, num_points, max(1, num_points // 10)):
        tgt = target_traj[i]
        plt.plot([camo[0], tgt[0]], [camo[1], tgt[1]], 'g--', alpha=0.5)

    # Plot paths
    plt.plot(target_traj[:, 0], target_traj[:, 1], 'r-', label="Target Path")
    plt.plot(agent_traj[:, 0], agent_traj[:, 1], 'g-', label="Motion Camouflage Path")

    # Start and end
    plt.scatter(*target_traj[0], c='red', s=100, marker='o', label="Target Start")
    plt.scatter(*agent_traj[0], c='green', s=100, marker='o', label="Agent Start")
    plt.scatter(*camo, c='black', s=80, marker='*', label="Camouflage Point")

    plt.title("Motion Camouflage Trajectory and Lines of Sight")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.show()

def plot_all_trajectories(results, name="trial", save=False, show=True):
    plt.figure(figsize=(10, 8))
    colors = plt.get_cmap('Dark2', len(results))  # or try 'Set1', 'Set3', 'Pastel1'
    # colors = cm.get_cmap('tab10', len(results))  # distinct colors

    for i, res in enumerate(results):
        ax, ay = zip(*res["agent_traj"])
        tx, ty = zip(*res["target_traj"])
        color = colors(i)

        # Plot target path
        plt.plot(tx, ty, '--', label=f"Target {i+1}", color=color, alpha=0.5)

        # Plot agent path (dashed, semi-transparent)
        plt.plot(ax, ay, label=f"Agent {i+1}", color=color)

        # Mark start positions
        plt.scatter(*res["agent_start"], color=color, marker='o', s=50, edgecolors='black')#, label=f"Start A{i+1}")
        plt.scatter(*res["target_start"], color=color, marker='o', s=50, edgecolors='white')#, label=f"Start T{i+1}")

        # Mark intercept point if captured
        if res["time_to_capture"]:
            intercept = res["agent_traj"][-1]
            plt.scatter(*intercept, color=color, marker='X', s=80)#, label=f"Capture {i+1}")
        else:
            # Timeout marker
            final = res["agent_traj"][-1]
            plt.scatter(*final, color=color, marker='^', s=80, label=f"Timeout {i+1}")


    # plt.title("Multi-Scenario Pursuit Trajectories")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    if save:
        plt.savefig(f"{name}.pdf", dpi=300)
    if show:
        plt.show()

def save_results_to_csv(results, filename="pursuit_results.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Run", "Strategy", "TimeToCapture_s", "PathLength_cm", "StartDistance_cm", "Success"])

        for i, res in enumerate(results):
            strategy = res.get("strategy", "SimplePursuit")
            time_to_capture = res["time_to_capture"]
            traj = res["agent_traj"]

            # Path length calculation
            path_length = 0
            for j in range(1, len(traj)):
                dx = traj[j][0] - traj[j-1][0]
                dy = traj[j][1] - traj[j-1][1]
                path_length += math.hypot(dx, dy)

            # Start distance
            dx = res["target_start"][0] - res["agent_start"][0]
            dy = res["target_start"][1] - res["agent_start"][1]
            start_distance = math.hypot(dx, dy)

            writer.writerow([
                i + 1,
                strategy,
                round(time_to_capture or -1, 2),
                round(path_length, 2),
                round(start_distance, 2),
                res["success"]
            ])

def plot_sine_line(all_times, x_axis, x_label, save=False, name=''):
    plt.figure(figsize=(6, 5))
    plt.plot(x_axis, all_times[0], "-x")
    plt.plot(x_axis, all_times[1], "-x")
    plt.xlabel(f'{x_label}')
    plt.ylabel('Mean Time Taken to Capture (s)')

    plt.legend(["Sinusoidal", "Linear"])

    plt.tight_layout()
    if save:
        plt.savefig(f'{name}.pdf', dpi=300) 
    plt.show()

    

# ------------------ Run ------------------
if __name__ == "__main__":
    # Collect all results across Ki values
    all_metrics = []
    random.seed(42)

    # scenarios = [
    #     ((100, 300), (300, 300)),
    #     ((50, 100), (400, 100)),
    #     ((700, 500), (200, 200)),
    #     ((150, 500), (600, 100)),
    #     ((100, 500), (100, 100)),
    # ]
    
    scenarios = [
        ((random.randint(0, 800), random.randint(0, 600)),
        (random.randint(0, 800), random.randint(0, 600)))

        for i in range(5)
    ]

    # "simple", "constant_bearing", "proportional_navigation", "parallel_navigation", "motion_camouflage"
    # x, y, speed, strategy="simple", theta_set_deg=30, Kp=2.0, Ki=0.5, Kd=4, camouflage_point=(0, 0), **kwargs
    # strategy = "simple"  

    ########## Compairing different params - Kp, Ki, DELAY ##########
    strategy = "simple"    
    
    params = [0.8, 1, 2, 5, 10, 15, 20, 40, 60, 80, 100] # kp - 15
    # params = [2, 5, 10, 20, 30, 40, 50, 60] # theta_cb - 2/50
    # params = [0.8, 1, 2, 3, 4, 5, 7, 10, 15, 20] # kd - 10
    # params = [0.5, 1, 2, 5, 7, 10, 15, 20] # motion camouflage kp - 7

    all_times = []
    for tpath in ["sinusoidal", "linear"]:
        time_taken = []
        for k in params:
            results = run_batch_simulations(strategy, scenarios, visual=False, frame_delay=0,
                                            Kp=k, Ki=0, Kd=0,
                                            target_path=tpath, duration=60, theta_CB=50)
            
            time_to_captureS = [item["time_to_capture"] for item in results]
            mean_time = np.mean(time_to_captureS)
            print(mean_time)
            time_taken.append(mean_time)
            # save_results_to_csv(results, filename=f'{strategy}_Kp{k}.csv')
            # plot_all_trajectories(results, name=f"{strategy}_Kp{k}", save=True, show=False)
        all_times.append(time_taken)
    
    print(all_times, time_taken, mean_time)
    plot_sine_line(all_times, params, x_label="Angle (degrees)", save=True, name="simple_tuning")

    ########################### BOXPLOT #########################################

    # all_strats = ["simple", "constant_bearing", "parallel_navigation", "motion_camouflage"]
    # final_results = {st: [] for st in all_strats}  
    # for strat in ["simple", "constant_bearing", "parallel_navigation", "motion_camouflage"]:
    #     # all_times = []
    #     for tpath in ["linear"]:
    #         results = run_batch_simulations(strat, scenarios, visual=False, frame_delay=50,
    #                                         Kp=7 if strat=="motion camouflage" else 15,
    #                                         Ki=0, 
    #                                         Kd=10 if strat=="parallel_navigation" else 0,
    #                                         target_path=tpath, duration=60, 
    #                                         theta_CB=40 if tpath == "sinusoidal" else 2)
    #         time_to_captureS = [item["time_to_capture"] for item in results]
           
    #     final_results[strat].extend(time_to_captureS)
    # # plot_sine_line(all_times, params, x_label="Kp", save=False, name="motion_camouflage_tuning")
    
    # data = list(final_results.values())
    # labels = list(final_results.keys())
    # means = [np.mean(sub_data) for sub_data in data]
    # print('here')
    # print(final_results)
    # print(data)
    # box = plt.boxplot(data, labels=labels, patch_artist=True)
    # for patch in box['boxes']:
    #     patch.set_facecolor('green')
    # plt.xticks([1, 2, 3, 4], ["Simple\nPursuit", "Constant\nBearing", "Parallel\nNavigation", "Motion\nCamouflage"])
    # plt.plot(range(1, 5), means, '--*', color='black')
    # plt.ylabel('Mean Time Taken to Capture (s)')
    # plt.savefig("summary_boxplot_linear_delay50.pdf", dpi=300)
    # plt.show()

    