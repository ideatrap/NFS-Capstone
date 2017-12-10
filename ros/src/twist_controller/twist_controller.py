#!/usr/bin/env python

import rospy
from pid import *
from yaw_controller import YawController
from lowpass import LowPassFilter
from std_msgs.msg   import Float32
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Magic numbers
BRAKE_FACTOR = .6
BRAKE_MIN = -.05

PID_BRAKE_ADJ =  .3
PID_THROTTLE_ADJ =  .6

THROTTLE_MAX = .70
THROTTLE_MIN = .08
THROTTLE_MAX_CHANGE = .05

class Controller(object):
    def __init__(self, *args, **kwargs):
            
        # Define the 2 PIDs: throttle/brake control and steering (Trial and error galore)
        self.pid_control = PID(1.8, .005, .0)
        self.pid_steering = PID(.70 , 1.2, .0282, mn = -kwargs["max_steer_angle"], mx = kwargs["max_steer_angle"])
        
        # LPF to be applied to steering target value (Trial and error galore): set all values steering PID values below 0.3 to zero
        self.steer_target_lpf = LowPassFilter(.3, .1)

        self.max_steer_angle = kwargs["max_steer_angle"]
        self.yaw_controller = YawController(kwargs["wheel_base"], kwargs["steer_ratio"], kwargs["min_speed"], kwargs["max_lat_accel"], self.max_steer_angle)
        self.brake_deadband = kwargs["brake_deadband"]
        self.time = None
        self.last_speed_target = 0.0
        self.last_steer_target = 0.0
        self.last_throttle = 0.0

        # Simple torque calc: vehicle weigt*decelaration*wheel radius
        self.max_brake_torque   = (BRAKE_FACTOR * float(kwargs["vehicle_mass"]) + float(kwargs["fuel_capacity"]) * GAS_DENSITY) * abs(float(kwargs["decel_limit"])) * float(kwargs["wheel_radius"])
     
    def control(self, *args, **kwargs):

        current_time = rospy.get_time()
        twist = kwargs['twist_cmd']
        current_velocity = kwargs['current_vel']
    
        # Get current/target velocities 	
        target_lin_vel = twist.twist.linear.x
        target_ang_vel = twist.twist.angular.z
        current_lin_vel = current_velocity.twist.linear.x
    
        # Convert angular speed to steering angle
        target_steer = self.yaw_controller.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)
        target_steer = self.steer_target_lpf.filt(target_steer)
        self.last_steer_target = target_steer

        # Reset PIDs given target changes: i.e. make sure error signals aren't saturated given abrupt changes
        self.check_PID_error_saturation(target_lin_vel, target_steer)

        if(self.time is not None):
            delta_t = current_time - self.time
            speed_err = target_lin_vel - current_lin_vel
            throttle_brake = self.pid_control.step(speed_err, delta_t)
            
            # PID output > 0: accelerate
            if throttle_brake >= 0.0:
                throttle = max(0.0,throttle_brake*PID_THROTTLE_ADJ)
                throttle = self.SmoothenActuation(throttle_brake, THROTTLE_MAX)
                if throttle - self.last_throttle > THROTTLE_MAX_CHANGE:
                    throttle = self.last_throttle + THROTTLE_MAX_CHANGE
                    self.last_throttle = throttle
                if throttle < THROTTLE_MIN:
                    throttle = 0.0
                brake = 0.0

            # BRAKE_MIN < PID output < 0: coast: stop accelarating but don't brake
            elif throttle_brake >= BRAKE_MIN:
                throttle = self.last_throttle = 0.0
                brake = 0.0

            # PID output < BRAKE_MIN: brake
            else:
                throttle = self.last_throttle = 0.0
                brake = max(0.0, -throttle_brake*PID_BRAKE_ADJ)
                
                #Values lower than deadband are filtered out
                if(brake < self.brake_deadband):
                    brake = 0.0
                else:
                    brake = self.SmoothenActuation(brake, self.max_brake_torque) 
            
            self.time = current_time
            return throttle, brake, target_steer
        else:
            self.time = current_time
            return 0.0, 0.0, 0.0
        
    def check_PID_error_saturation(self, target_lin_vel, target_steer):
        #If PID throttle error signal is saturated incorrectedly: i.e. ask for accelaration when target_lin_vel is lower than current velocity
        # or vice versa
        if(((target_lin_vel > self.last_speed_target and self.pid_control.int_val < 0) or \
         (target_lin_vel < self.last_speed_target and self.pid_control.int_val > 0))):
            self.pid_control.reset()

        # If PID steering error signal is saturated incorrectedly: i.e. ask for left turn when target_steer is right, or vice versa, go set pid signal to drive in 
        # in straight line
        if(((target_steer > self.last_steer_target and self.pid_steering.int_val < 0) or \
        (target_steer < self.last_steer_target and self.pid_steering.int_val > 0))):
            self.pid_steering.int_val /= 2
    
    # Smoothen force to decrease jerk params
    def SmoothenActuation(self, value, scale):
        return scale * math.tanh(value)