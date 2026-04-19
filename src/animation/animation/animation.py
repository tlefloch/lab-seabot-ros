import rclpy
from rclpy.node import Node
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
import numpy as np
from seabot_msgs.msg import SimulationState



class AnimationNode(Node):
    def __init__(self):
        super().__init__('matplotlib_animation_node')

        self.init_interfaces()

        # Dimensions
        robot_size = 1.1 # m
        a = robot_size * 0.7
        b = robot_size * 0.1
        c = robot_size * 0.2
        d = robot_size * 0.1
        e = d
        f = d * 0.3
        g = e * 0.8
        h = f * 1.5
        i = d * 0.5

        self.rob_dimensions=[a, b, c, d, e, f, g, h, i]

        # Create and configure the figure and axes
        fig, ax=plt.subplots(figsize=(8, 8))
        self.ax=ax
        ax_depth_size=15
        self.param_fig(self.ax,ax_depth_size)

        # Show time
        self.time_text = ax.text(0.02, 0.96, '', transform=ax.transAxes, fontsize=12, bbox=dict(facecolor='white', alpha=0.8))

        # Water surface
        ax.plot([-10,10],[0,0], color="blue")

        # Artists
        ## Robot artists
        robot_color = "coral"
        body = Polygon(np.zeros((4,2)), closed=True, facecolor=robot_color, edgecolor="black")
        head = Line2D([], [], color="black", linewidth=1)
        left_arm = Polygon(np.zeros((4,2)), closed=True, facecolor=robot_color, edgecolor="black")
        right_arm = Polygon(np.zeros((4,2)), closed=True, facecolor=robot_color, edgecolor="black")
        left_motor = Polygon(np.zeros((4,2)), closed=True, color="black")
        right_motor = Polygon(np.zeros((4,2)), closed=True, color="black")
        piston = Polygon(np.zeros((4,2)), closed=True, facecolor="gray", edgecolor="black")
        
        ax.add_patch(body)
        ax.add_line(head)
        ax.add_patch(left_arm)
        ax.add_patch(right_arm)
        ax.add_patch(left_motor)
        ax.add_patch(right_motor)
        ax.add_patch(piston)
        
        self.robot_artists=[body, head, left_arm, left_motor, right_arm, right_motor, piston]

        # ## Meta artists
        # depth_line = Line2D([], [], color="blue", linestyle="--", linewidth=1)
        # depth_filter_line = Line2D([], [], color="red", linestyle="--", linewidth=1)
        # depth_setpoint_line = Line2D([], [], color="green", linestyle="--", linewidth=1)
        
        # ax.add_line(depth_line)
        # ax.add_line(depth_filter_line)
        # ax.add_line(depth_setpoint_line)
        
        # self.meta_artists=[depth_line, depth_filter_line, depth_setpoint_line]

        self.t0 = 0
        self.t = 0
        self.simu_started = False

        # Display the animation
        plt.ion()
        plt.show()
    
    def simulation_state_callback(self, msg):
        if not self.simu_started:
            self.t0 = msg.time
            self.simu_started = True
        else:
            self.t = msg.time - self.t0
        self.z = msg.depth
        self.vz = msg.speed
        self.Vp = msg.piston_volume

        self.update()

        plt.pause(0.001)  # Pause pour mettre à jour le graphique

    def init_interfaces(self):
        self.subscription = self.create_subscription(
                    SimulationState,
                    'simulation_state',
                    self.simulation_state_callback,
                    10)
        self.subscription  # prevent unused variable warning


    def update(self):
        self.ax.set_ylim(self.z - 2, self.z + 2)

        self.time_text.set_text(f'Time = {self.t:.1f} s')

        self.draw_robot()
        #self.draw_meta()
        
        plt.pause(0.001)

    def param_fig(self,ax,ax_depth_size,x0=0,y0=0):
        ax.set_title("Simulation Seabot")
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_xlim(-2, 2)

    def draw_robot(self):
        body, head, left_arm, left_motor, right_arm, right_motor, piston = self.robot_artists

        # Dimensions
        a, b, c, d, e, f, g, h, i = self.rob_dimensions
        
        # State variables
        z, vz, Vp = self.z, self.vz, self.Vp

        # Body (rectangle with rounded top)
        rect = np.array([
            [-d/2, 0],
            [ d/2, 0],
            [ d/2, a],
            [-d/2, a]
        ])
        rect = rect + np.array([0, z - a])
        body.set_xy(rect)

        # Rounded top (semicircle)
        theta_arc = np.linspace(0, np.pi, 20)
        arc = np.vstack([ (d/2)*np.cos(theta_arc),
                          (a) + b*np.sin(theta_arc)]).T
        arc = arc + np.array([0, z - a])
        head.set_data(arc[:,0], arc[:,1])

        # Left arm
        arm = np.array([
            [-d/2, a*0.5 - f/2],
            [-d/2 - e, a*0.5 - f/2],
            [-d/2 - e, a*0.5 + f/2],
            [-d/2, a*0.5 + f/2]
        ])
        arm = arm + np.array([0, z - a])
        left_arm.set_xy(arm)

        # Right arm
        arm = np.array([
            [d/2, a*0.5 - f/2],
            [d/2 + e, a*0.5 - f/2],
            [d/2 + e, a*0.5 + f/2],
            [d/2, a*0.5 + f/2]
        ])
        arm = arm + np.array([0, z - a])
        right_arm.set_xy(arm)

        # Left motor (rectangle)
        motor = np.array([
            [-d/2 - e - g, a*0.5 - h/2],
            [-d/2 - e, a*0.5 - h/2],
            [-d/2 - e, a*0.5 + h/2],
            [-d/2 - e - g, a*0.5 + h/2]
        ])
        motor = motor + np.array([0, z - a])
        left_motor.set_xy(motor)

        # Right motor (rectangle)
        motor = np.array([
            [d/2 + e, a*0.5 - h/2],
            [d/2 + e + g, a*0.5 - h/2],
            [d/2 + e + g, a*0.5 + h/2],
            [d/2 + e, a*0.5 + h/2]
        ])
        motor = motor + np.array([0, z - a])
        right_motor.set_xy(motor)

        # Piston (rectangle)
        piston_rect = np.array([
            [-i/2, 0],
            [ i/2, 0],
            [ i/2, c],
            [-i/2, c]
        ])
        piston_pos = self.get_piston_position_from_volume(Vp)
        piston_rect = piston_rect + np.array([0, z - a - piston_pos])
        piston.set_xy(piston_rect)

    def get_piston_position_from_volume(self, V):
        a, b, c, d, e, f, g, h, i = self.rob_dimensions
        piston_pos = V / (np.pi * (i/2)**2)
        return piston_pos

def main(args=None):
    rclpy.init(args=args)
    node = AnimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
