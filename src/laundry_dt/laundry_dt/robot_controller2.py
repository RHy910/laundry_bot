import rclpy
import doctest
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rclpy.node import Node
from enum import Enum


class RobotState(Enum):
    IDLE            = 0
    ADVANCING       = 1
    LIFTING         = 2
    SEGMENT_FORWARD = 3
    BACK_WINCHING   = 4
    COMPLETE        = 5


class Robot_controller(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.state = RobotState.IDLE
        self.segments  = ['front', 'mid', 'back']

        self.level    = {'front': 0, 'mid': 0, 'back': 0}
        self.active_segment = 0

        self.stair_heights   = {}
        self.scouting_level  = 0 

        self.stopped  = {'front': True, 'mid': True, 'back': True}

         # ── tuning ────────────────────────────────────────────────────
        self.stair_threshold  = 10.0
        self.clearance_offset = 20
        self.winch_steps      = 200
        self.winch_count      = 0

        # ── publishers / subscribers ──────────────────────────────────
        self.motor_pub    = {}
        self.stepper_pub  = {}
        self.stepper_sub  = {}
        self.ultra_sub    = {}

        for seg in self.segments:
            self.motor_pub[seg] = self.create_publisher(Twist, f'{seg}/cmd_vel', 10)
            self.ultra_sub[seg] = self.create_subscription( Float32, f'ultrasonic/{seg}/distance',lambda msg, s=seg: self.on_ultrasonic(msg, s), 10)
            if seg != 'mid':#for stepper
                    self.stepper_pub[stp] = self.create_publisher( Float32, f'stepper/{stp}/cmd', 10)
                    self.stepper_sub[stp] = self.create_subscription(Float32,f'stepper/{stp}/steps',lambda msg, s=stp: self.on_stepper(msg, s),10)

        self.winch_pub = self.create_publisher(Float32, '/mid/winch/cmd', 10)

        self.get_logger().info('Robot Controller Started')

    # ── helpers ───────────────────────────────────────────────────────

    def drive(self, seg, speed=0.5):
        cmd = Twist()
        cmd.linear.x = speed
        self.stopped[seg] = False
        self.motor_pub[seg].publish(cmd)
    
    def stop(self):
        for seg in self.segments:
            self.motor_pub[seg].publish(Twist()) #make sure this twist stops all forward movement
            self.stopped[seg] = True
            self.get_logger().info(f'*** bot stopped***')



    def halt(self, seg):
        self.motor_pub[seg].publish(Twist())
        self.stopped[seg] = True
        self.get_logger().info(f'***{seg} stopped***')


    def step_up(self, stp, steps):
        cmd = Float32()
        cmd.data = float(steps)
        self.stepper_pub[stp].publish(cmd)

    def step_down(self, stp, steps):
        cmd = Float32()
        cmd.data = float(-steps)
        self.stepper_pub[stp].publish(cmd)

    def fire_winch(self):
        cmd = Float32()
        cmd.data = float(self.winch_steps)
        self.winch_pub.publish(cmd)
        self.get_logger().info(f'Winch activated — {self.winch_steps} steps')

    def do_lift(self, seg):
        """Execute a lift for the given segment using stored stair height."""
        if self.level[seg] not in self.stair_heights:
            self.get_logger().error(
                f'[{seg}] cannot lift — no height stored for level {self.level[seg]}'
            )
            self.waiting[seg] = True # throw error/ stop chaneg bot stay to broken or somethingwhen this happens should never happen can cause waiting loop
            return

        steps = self.stair_heights[self.level[seg]]

        if seg == 'front':
            self.step_up('front', steps)
        elif seg == 'mid':
            self.step_down('front', steps)   # front stepper lowers front segment
            self.step_up('back', steps)      # back stepper raises back segment (lifts mid)
        else:
            self.step_down('back', steps)    # back stepper lowers back segment

        self.level[seg] += 1
        self.get_logger().info(
            f'[{seg}] lifting → level {self.level[seg]} | steps: {steps}'
        )
    def get_active_seg(self):
        '''
        {f:1,m:1,b:1} = front ==> front = mid > back === -1
        {f:2,m:1,b:1} = mid   ==> front > mid and mid == back === 0
        {f:2,m:2,b:1} = front ==> front = mid > back === -1
        {f:3,m:2,b:1} = back  ==> front > mid > b == 0
        {f:3,m:2,b:2} = mid   ==> front > mid and mid == back == 1
        {f:3,m:3,b:2} = front ==> front = mid > back === -1
        {f:4,m:3,b:2} = back  ==> front > mid > b == 0
        >>> 
        '''
        front = self.level.get('front',0)
        mid = self.level.get('mid',0)
        back = self.level.get('back',0)
        
        
        if not (front >= mid >= back):
            
            self.get_logger().error(
                f'***invalid level order levels must be front>=mid>=back'
            )
            return
        
        if front == mid:
            return 'front'
        else:
            if mid == back:
                return 'mid'
            else:
                return 'back'
        

        


        
            

    # ── stepper callback ──────────────────────────────────────────────
    def on_stepper(self, msg, stp):
        self.live_steps[stp] = msg.data

        # front stepper scouts stair height while lifting front segment
        if stp == 'front' and self.lifting['front']:
            if self.scouting_level not in self.stair_heights:
                measured = msg.data + self.clearance_offset
                self.stair_heights[self.scouting_level] = measured
                self.get_logger().info(
                    f'Stair {self.scouting_level} scouted: {measured} steps | all heights: {self.stair_heights}'
                )

    
    def on_ultrasonic(self, msg, seg):
        #moved based on which segment shoudl be active at the time
        distance = msg.data
        active = self.get_active_seg() 

        if self.state == RobotState.IDLE:
            self.state = RobotState.ADVANCING
            for s in self.segments:
                self.drive(s)
            return
        
        # ── ADVANCING ────────────────────────────────────────────────
        if self.state == RobotState.ADVANCING and seg == acctive:

            if distance < self.stair_threshold:
                self.halt(seg)
                self.stop()
                self.get_logger().info(
                    f'[{seg}] hit stair | level: {self.level[seg]}'
                )
            
            if seg == 'front':
                self.state = RobotState.LIFTING
                self.step_up('front', 999.0)
                self.get_logger().info(f'[{seg}] liftinging')
        
        # ── lifting ────────────────────────────────────────────────
        if self.state == RobotState.lifting and seg == active:
            if distance > self.stair_threshold:
                self.state = RobotState.SEGMENT_FORWARD
                for s in self.segments:
                    self.drive(s)
                    self.get_logger().info(
                        f'[{seg}] moving forward on level: {self.level[seg]}'
                    )
        
        # ── Segments forward ────────────────────────────────────────────────
            if self.state == RobotState.SEGMENT_FORWARD:
                if distance < self.stair_threshold: # ADD A CHECK FOR WHEN THE DISTACN IS BEYONG THE LEGHTH OF THE ETIRE BO THEN WE CAN ADJUSTC THE LEVELS AS NESSESARY TO GET THE RIGHT ACTIVE PICES TO MOVE UP THE EST OF TEH STAIRS
                    self.stop(seg)
                    self.stopped[seg] = True
                if set(self.stopped.values()) == set(True): 
                    self.level[active]+=1
                    self.state = RobotState.LIFTING













    
