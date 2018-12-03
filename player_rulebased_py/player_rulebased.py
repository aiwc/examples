#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

from __future__ import print_function

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random
import math
import sys

import base64
import numpy as np

import helper

#reset_reason
NONE = 0
GAME_START = 1
SCORE_MYTEAM = 2
SCORE_OPPONENT = 3
GAME_END = 4
DEADLOCK = 5
GOALKICK = 6
FREEKICK = 7
PENALTYKICK = 8

#game_state
STATE_DEFAULT = 0
STATE_BACKPASS = 1
STATE_GOALKICK = 2
STATE_FREEKICK = 3
STATE_PENALTYKICK = 4

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2
ACTIVE = 3
TOUCH = 4

class Received_Image(object):
    def __init__(self, resolution, colorChannels):
        self.resolution = resolution
        self.colorChannels = colorChannels
        # need to initialize the matrix at timestep 0
        self.ImageBuffer = np.zeros((resolution[1], resolution[0], colorChannels)) # rows, columns, colorchannels
    def update_image(self, received_parts):
        self.received_parts = received_parts
        for i in range(0,len(received_parts)):
           dec_msg = base64.b64decode(self.received_parts[i].b64, '-_') # decode the base64 message
           np_msg = np.fromstring(dec_msg, dtype=np.uint8) # convert byte array to numpy array
           reshaped_msg = np_msg.reshape((self.received_parts[i].height, self.received_parts[i].width, 3))
           for j in range(0, self.received_parts[i].height): # y axis
               for k in range(0, self.received_parts[i].width): # x axis
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 0] = reshaped_msg[j, k, 0] # blue channel
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 1] = reshaped_msg[j, k, 1] # green channel
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 2] = reshaped_msg[j, k, 2] # red channel

class SubImage(object):
    def __init__(self, x, y, width, height, b64):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.b64 = b64

class Frame(object):
    def __init__(self):
        self.time = None
        self.score = None
        self.reset_reason = None
        self.game_state = None
        self.subimages = None
        self.coordinates = None

class Component(ApplicationSession):
    """
    AI Base + Rule Based Algorithm
    """

    def __init__(self, config):
        ApplicationSession.__init__(self, config)

    def printConsole(self, message):
        print(message)
        sys.__stdout__.flush()

    def onConnect(self):
        self.join(self.config.realm)

    @inlineCallbacks
    def onJoin(self, details):

##############################################################################
        def init_variables(self, info):
            # Here you have the information of the game (virtual init() in random_walk.cpp)
            # List: game_time, number_of_robots
            #       field, goal, penalty_area, goal_area, resolution Dimension: [x, y]
            #       ball_radius, ball_mass,
            #       robot_size, robot_height, axle_length, robot_body_mass, ID: [0, 1, 2, 3, 4]
            #       wheel_radius, wheel_mass, ID: [0, 1, 2, 3, 4]
            #       max_linear_velocity, max_torque, codewords, ID: [0, 1, 2, 3, 4]
            # self.game_time = info['game_time']
            self.number_of_robots = info['number_of_robots']

            self.field = info['field']
            self.goal = info['goal']
            self.penalty_area = info['penalty_area']
            # self.goal_area = info['goal_area']
            self.resolution = info['resolution']

            self.ball_radius = info['ball_radius']
            # self.ball_mass = info['ball_mass']

            self.robot_size = info['robot_size']
            # self.robot_height = info['robot_height']
            # self.axle_length = info['axle_length']
            # self.robot_body_mass = info['robot_body_mass']

            # self.wheel_radius = info['wheel_radius']
            # self.wheel_mass = info['wheel_mass']

            self.max_linear_velocity = info['max_linear_velocity']
            # self.max_torque = info['max_torque']
            # self.codewords = info['codewords']

            self.colorChannels = 3
            self.end_of_frame = False
            self.image = Received_Image(self.resolution, self.colorChannels)
            self.cur_posture = []
            self.cur_ball = []
            self.prev_posture = []
            self.prev_ball = []
            self.previous_frame = Frame()
            self.def_idx = 0
            self.atk_idx = 0
            self.wheels = [0 for _ in range(10)]
            return
##############################################################################

        try:
            info = yield self.call(u'aiwc.get_info', args.key)
        except Exception as e:
            self.printConsole("Error: {}".format(e))
        else:
            try:
                self.sub = yield self.subscribe(self.on_event, args.key)
            except Exception as e2:
                self.printConsole("Error: {}".format(e2))

        init_variables(self, info)

        try:
            yield self.call(u'aiwc.ready', args.key)
        except Exception as e:
            self.printConsole("Error: {}".format(e))
        else:
            self.printConsole("I am ready for the game!")

    def get_coord(self, received_frame):
        self.cur_ball = received_frame.coordinates[BALL]
        self.cur_posture = received_frame.coordinates[MY_TEAM]
        self.prev_ball = self.previous_frame.coordinates[BALL]
        self.prev_posture = self.previous_frame.coordinates[MY_TEAM]

    def find_closest_robot(self):
        # find the closest defender
        min_idx = 0
        min_distance = 9999.99

        for i in [1, 2]:
            measured_distance = helper.distance(self.cur_ball[X], self.cur_posture[i][X], self.cur_ball[Y], self.cur_posture[i][Y])
            if (measured_distance < min_distance):
                min_distance = measured_distance
                min_idx = i
        self.def_idx = min_idx

        # find the closest attacker
        min_idx = 0
        min_distance = 9999.99
        for i in [3, 4]:
            measured_distance = helper.distance(self.cur_ball[X], self.cur_posture[i][X], self.cur_ball[Y], self.cur_posture[i][Y])
            if (measured_distance < min_distance):
                min_distance = measured_distance
                min_idx = i
        self.atk_idx = min_idx

    def predict_ball_location(self, steps):
        dx = self.cur_ball[X] - self.prev_ball[X]
        dy = self.cur_ball[Y] - self.prev_ball[Y]
        return [self.cur_ball[X]+steps*dx, self.cur_ball[Y]+steps*dy]

    def set_wheel_velocity(self, id, left_wheel, right_wheel, max_speed):
        multiplier = 1

        if(abs(left_wheel) > self.max_linear_velocity[id] or abs(right_wheel) > self.max_linear_velocity[id] or max_speed):
            if (abs(left_wheel) > abs(right_wheel)):
                multiplier = self.max_linear_velocity[id] / abs(left_wheel)
            else:
                multiplier = self.max_linear_velocity[id] / abs(right_wheel)


        self.wheels[2*id] = left_wheel*multiplier
        self.wheels[2*id + 1] = right_wheel*multiplier

    def position(self, id, x, y, scale, max_speed):
        damping = 0.35
        mult_lin = 3.5
        mult_ang = 0.4
        ka = 0
        sign = 1

        dx = x - self.cur_posture[id][X]
        dy = y - self.cur_posture[id][Y]
        d_e = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        desired_th = (math.pi/2) if (dx == 0 and dy == 0) else math.atan2(dy, dx)

        d_th = desired_th - self.cur_posture[id][TH]
        while(d_th > math.pi):
            d_th -= 2*math.pi
        while(d_th < -math.pi):
            d_th += 2*math.pi

        if (d_e > 1):
            ka = 17/90
        elif (d_e > 0.5):
            ka = 19/90
        elif (d_e > 0.3):
            ka = 21/90
        elif (d_e > 0.2):
            ka = 23/90
        else:
            ka = 25/90

        if (d_th > helper.degree2radian(95)):
            d_th -= math.pi
            sign = -1
        elif (d_th < helper.degree2radian(-95)):
            d_th += math.pi
            sign = -1

        if (abs(d_th) > helper.degree2radian(85)):
            self.set_wheel_velocity(id, -mult_ang*d_th, mult_ang*d_th, False)
        else:
            if (d_e < 5 and abs(d_th) < helper.degree2radian(40)):
                ka = 0.1
            ka *= 4
            self.set_wheel_velocity(id,
                                    sign * scale * (mult_lin * (1 / (1 + math.exp(-3*d_e)) - damping) - mult_ang * ka * d_th),
                                    sign * scale * (mult_lin * (1 / (1 + math.exp(-3*d_e)) - damping) + mult_ang * ka * d_th), max_speed)

    def face_specific_position(self, id, x, y):
        dx = x - self.cur_posture[id][X]
        dy = y - self.cur_posture[id][Y]

        desired_th = (math.pi/2) if (dx == 0 and dy == 0) else math.atan2(dy, dx)

        self.angle(id, desired_th)

    # returns the angle toward a specific position from current robot posture
    def direction_angle(self, id, x, y):
        dx = x - self.cur_posture[id][X]
        dy = y - self.cur_posture[id][Y]

        return ((math.pi/2) if (dx == 0 and dy == 0) else math.atan2(dy, dx))

    def angle(self, id, desired_th):
        mult_ang = 0.4

        d_th = desired_th - self.cur_posture[id][TH]
        d_th = helper.trim_radian(d_th)

        if (d_th > helper.degree2radian(95)):
            d_th -= math.pi
            sign = -1
        elif (d_th < helper.degree2radian(-95)):
            d_th += math.pi
            sign = -1

        self.set_wheel_velocity(id, -mult_ang*d_th, mult_ang*d_th, False)

    def in_penalty_area(self, obj, team):
        if (abs(obj[Y]) > self.penalty_area[Y]/2):
            return False

        if (team == MY_TEAM):
            return (obj[X] < -self.field[X]/2 + self.penalty_area[X])
        else:
            return (obj[X] > self.field[X]/2 - self.penalty_area[X])

    def ball_coming_toward_robot(self, id):
        x_dir = abs(self.cur_posture[id][X] - self.prev_ball[X]) > abs(self.cur_posture[id][X] - self.cur_ball[X])
        y_dir = abs(self.cur_posture[id][Y] - self.prev_ball[Y]) > abs(self.cur_posture[id][Y] - self.cur_ball[Y])

        # ball is coming closer
        if (x_dir and y_dir):
            return True
        else:
            return False

    def shoot_chance(self, id):
        dx = self.cur_ball[X] - self.cur_posture[id][X]
        dy = self.cur_ball[Y] - self.cur_posture[id][Y]

        if (dx < 0):
            return False

        y = (self.field[X]/2 - self.cur_ball[X])*dy/dx + self.cur_posture[id][Y]

        if (abs(y) < self.goal[Y]/2 + 0.1):
            return True
        else:
            return False

    @inlineCallbacks
    def on_event(self, f):

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return

        def goalie(self, id):
            # default desired position
            x = (-self.field[X]/2) + (self.robot_size[id]/2) + 0.05
            y = max(min(self.cur_ball[Y], (self.goal[Y]/2 - self.robot_size[id]/2)), -self.goal[Y]/2 + self.robot_size[id]/2)

            # if the robot is inside the goal, try to get out
            if (self.cur_posture[id][X] < -self.field[X]/2):
                if (self.cur_posture[id][Y] < 0):
                    self.position(id, x, self.cur_posture[id][Y] + 0.2, 1.4, False)
                else:
                    self.position(id, x, self.cur_posture[id][Y] - 0.2, 1.4, False)
            # if the goalie is outside the penalty area
            elif (not self.in_penalty_area(self.cur_posture[id], MY_TEAM)):
                # return to the desired position
                self.position(id, x, y, 1.4, True)
            # if the goalie is inside the penalty area
            else:
                # if the ball is inside the penalty area
                if (self.in_penalty_area(self.cur_ball, MY_TEAM)):
                    # if the ball is behind the goalie
                    if (self.cur_ball[X] < self.cur_posture[id][X]):
                        # if the ball is not blocking the goalie's path
                        if (abs(self.cur_ball[Y] - self.cur_posture[id][Y]) > 2*self.robot_size[id]):
                            # try to get ahead of the ball
                            self.position(id, self.cur_ball[X] - self.robot_size[id], self.cur_posture[id][Y], 1.4, False)
                        else:
                            # just give up and try not to make a suicidal goal
                            self.angle(id, math.pi/2)
                    # if the ball is ahead of the goalie
                    else :
                        desired_th = self.direction_angle(id, self.cur_ball[X], self.cur_ball[Y])
                        rad_diff = helper.trim_radian(desired_th - self.cur_posture[id][TH])
                        # if the robot direction is too away from the ball direction
                        if (rad_diff > math.pi/3):
                            # give up kicking the ball and block the goalpost
                            self.position(id, x, y, 1.4, False)
                        else:
                            # try to kick the ball away from the goal
                            pred_ball = self.predict_ball_location(2)
                            self.position(id, max(pred_ball[X], x), pred_ball[Y], 1.4, True)
                # if the ball is not in the penalty area
                else:
                    # if the ball is within alert range and y position is not too different
                    if (self.cur_ball[X] < -self.field[X]/2 + 1.5*self.penalty_area[X] and abs(self.cur_ball[Y]) < 1.5*self.penalty_area[Y]/2 and abs(self.cur_ball[Y] - self.cur_posture[id][Y]) < 0.2):
                        self.face_specific_position(id, self.cur_ball[X], self.cur_ball[Y])
                    # otherwise
                    else:
                        self.position(id, x, y, 1.4, True)

        def defender(self, id, offset_y):
            ox = 0.1
            oy = 0.075
            min_x = (-self.field[X]/2) + (self.robot_size[id]/2) + 0.05

            # If ball is on offense
            if (self.cur_ball[X] > 0):
                # If ball is in the upper part of the field (y>0)
                if (self.cur_ball[Y] > 0):
                    self.position(id,
                                  (self.cur_ball[X]-self.field[X]/2)/2,
                                  (min(self.cur_ball[Y],self.field[Y]/3))+offset_y, 1.4, False)
                # If ball is in the lower part of the field (y<0)
                else:
                    self.position(id,
                                  (self.cur_ball[X]-self.field[X]/2)/2,
                                  (max(self.cur_ball[Y],-self.field[Y]/3))+offset_y, 1.4, False)
            # If ball is on defense
            else:
                # If robot is in front of the ball
                if (self.cur_posture[id][X] > self.cur_ball[X] - ox):
                    # If this defender is the nearest defender from the ball
                    if (id == self.def_idx):
                        self.position(id,
                                      (self.cur_ball[X]-ox),
                                      ((self.cur_ball[Y]+oy) if (self.cur_posture[id][Y]<0) else (self.cur_ball[Y]-oy)), 1.4, False)
                    else:
                        self.position(id,
                                      (max(self.cur_ball[X]-0.03, min_x)),
                                      ((self.cur_posture[id][Y]+0.03) if (self.cur_posture[id][Y]<0) else (self.cur_posture[id][Y]-0.03)), 1.4, False)
                # If robot is behind the ball
                else:
                    if (id == self.def_idx):
                        self.position(id,
                                      self.cur_ball[X],
                                      self.cur_ball[Y], 1.4, False)
                    else:
                        self.position(id,
                                      (max(self.cur_ball[X]-0.03, min_x)),
                                      ((self.cur_posture[id][Y]+0.03) if (self.cur_posture[id][Y]<0) else (self.cur_posture[id][Y]-0.03)), 1.4, False)

        def attacker(self, id):
            ball_dist = helper.distance(self.cur_posture[id][X], self.cur_ball[X], self.cur_posture[id][Y], self.cur_ball[Y])
            goal_dist = helper.distance(self.cur_posture[id][X], self.field[X]/2, self.cur_posture[id][Y], 0)

            # if the robot is blocking the ball's path toward opponent side
            if (self.cur_ball[X] > -0.3*self.field[X] and self.cur_ball[X] < 0.3*self.field[X] and self.cur_posture[id][X] > self.cur_ball[X] + 0.1 and abs(self.cur_posture[id][Y] - self.cur_ball[Y]) < 0.2):
                if (self.cur_ball[Y] < 0):
                    self.position(id, self.cur_posture[id][X] - 0.5, self.cur_ball[Y] + 0.5, 1.0, True)
                else:
                    self.position(id, self.cur_posture[id][X] - 0.5, self.cur_ball[Y] - 0.5, 1.0, True)
                return

            # if the robot can shoot from current position
            if (id == self.atk_idx and self.shoot_chance(id)):
                self.position(id, self.cur_ball[X], self.cur_ball[Y], 1.4, True)
                return

            # if the ball is coming toward the robot, seek for shoot chance
            if (id == self.atk_idx and self.ball_coming_toward_robot(id)):
                dx = self.cur_ball[X] - self.prev_ball[X]
                dy = self.cur_ball[Y] - self.prev_ball[Y]
                pred_x = (self.cur_posture[id][Y] - self.cur_ball[Y])*dx/dy + self.cur_ball[X]
                steps = (self.cur_posture[id][Y] - self.cur_ball[Y])/dy

                # if the ball will be located in front of the robot
                if (pred_x > self.cur_posture[id][X]):
                    pred_dist = pred_x - self.cur_posture[id][X]
                    # if the predicted ball location is close enough
                    if (pred_dist > 0.1 and pred_dist < 0.3 and steps < 10):
                        # find the direction towards the opponent goal and look toward it
                        goal_angle = self.direction_angle(id, self.field[X]/2, 0)
                        self.angle(id, goal_angle)
                        return

            if (id == self.atk_idx):
                if (self.cur_ball[X] > -0.3*self.field[X]):
                    if (self.cur_posture[id][X] < self.cur_ball[X] - self.ball_radius):
                        self.position(id, self.cur_ball[X], self.cur_ball[Y], 1.4, False)
                    else:
                        if (abs(self.cur_ball[Y] - self.cur_posture[id][Y]) > 0.3):
                            self.position(id, self.cur_ball[X] - 0.1, self.cur_ball[Y], 1.4, False)
                        else:
                            self.position(id, self.cur_ball[X] - 0.1, self.cur_posture[id][Y], 1.4, False)
                else:
                    self.position(id, -0.3*self.field[X], self.cur_ball[Y], 1.4, False)
            else:
                if (self.cur_ball[X] > -0.3*self.field[X]):
                    if (self.cur_ball[Y] < 0):
                        self.position(id, self.cur_ball[X] - 0.5, self.goal[Y]/2, 1.4, False)
                    else:
                        self.position(id, self.cur_ball[X] - 0.5, -self.goal[Y]/2, 1.4, False)
                else:
                    if (self.cur_ball[Y] < 0):
                        self.position(id, -0.3*self.field[X], min(self.cur_ball[Y] + 0.5, self.field[Y]/2 - self.robot_size[id]/2), 1.4, False)
                    else:
                        self.position(id, -0.3*self.field[X], max(self.cur_ball[Y] - 0.5, -self.field[Y]/2 + self.robot_size[id]/2), 1.4, False)

        # initiate empty frame
        received_frame = Frame()
        received_subimages = []

        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'game_state' in f:
            received_frame.game_state = f['game_state']
        if 'ball_ownership' in f:
            received_frame.ball_ownership = f['ball_ownership']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
            for s in received_frame.subimages:
                received_subimages.append(SubImage(s['x'],
                                                   s['y'],
                                                   s['w'],
                                                   s['h'],
                                                   s['base64'].encode('utf8')))
            self.image.update_image(received_subimages)
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']
        if 'EOF' in f:
            self.end_of_frame = f['EOF']

        #self.printConsole(received_frame.time)
        #self.printConsole(received_frame.score)
        #self.printConsole(received_frame.reset_reason)
        #self.printConsole(self.end_of_frame)
        #self.printConsole(received_frame.subimages)

        if (self.end_of_frame):
            # To get the image at the end of each frame use the variable:
            # self.image.ImageBuffer

            if(received_frame.reset_reason != NONE):
                self.previous_frame = received_frame

            self.get_coord(received_frame)
            self.find_closest_robot()

##############################################################################
            if(received_frame.game_state == STATE_DEFAULT):
                #(update the robots' wheels)
                # Robot Functions in STATE_DEFAULT
                goalie(self, 0)
                defender(self, 1, 0.2)
                defender(self, 2, -0.2)
                attacker(self, 3)
                attacker(self, 4)

                set_wheel(self, self.wheels)
##############################################################################
            elif(received_frame.game_state == STATE_BACKPASS):
                #(update the robots' wheels)
                # Robot Functions in STATE_BACKPASS
                # Drive the attacker to the center of the field to kick the ball
                if (received_frame.ball_ownership):
                    self.position(4, 0, 0, 1.4, False)

                set_wheel(self, self.wheels)
##############################################################################
            elif(received_frame.game_state == STATE_GOALKICK):
                #(update the robots' wheels)
                # Robot Functions in STATE_GOALKICK
                # Driver the goalie forward to kick the ball
                if (received_frame.ball_ownership):
                    self.set_wheel_velocity(0, self.max_linear_velocity[0], self.max_linear_velocity[0], True)

                set_wheel(self, self.wheels)
##############################################################################
            elif(received_frame.game_state == STATE_FREEKICK):
                #(update the robots' wheels)
                # Robot Functions in STATE_DEFAULT
                goalie(self, 0)
                defender(self, 1, 0.2)
                defender(self, 2, -0.2)
                attacker(self, 3)
                attacker(self, 4)

                set_wheel(self, self.wheels)
##############################################################################
            elif(received_frame.game_state == STATE_PENALTYKICK):
                #(update the robots' wheels)
                # Robot Functions in STATE_PENALTYKICK
                # Driver the attacker forward to kick the ball
                if (received_frame.ball_ownership):
                    self.set_wheel_velocity(4, self.max_linear_velocity[0], self.max_linear_velocity[0], True)

                set_wheel(self, self.wheels)
##############################################################################
            if(received_frame.reset_reason == GAME_END):
                #(virtual finish() in random_walk.cpp)
                #save your data
                with open(args.datapath + '/result.txt', 'w') as output:
                    #output.write('yourvariables')
                    output.close()
                #unsubscribe; reset or leave
                yield self.sub.unsubscribe()
                try:
                    yield self.leave()
                except Exception as e:
                    self.printConsole("Error: {}".format(e))
##############################################################################

            self.end_of_frame = False
            self.previous_frame = received_frame


    def onDisconnect(self):
        if reactor.running:
            reactor.stop()


if __name__ == '__main__':

    try:
        unicode
    except NameError:
        # Define 'unicode' for Python 3
        def unicode(s, *_):
            return s

    def to_unicode(s):
        return unicode(s, "utf-8")

    parser = argparse.ArgumentParser()
    parser.add_argument("server_ip", type=to_unicode)
    parser.add_argument("port", type=to_unicode)
    parser.add_argument("realm", type=to_unicode)
    parser.add_argument("key", type=to_unicode)
    parser.add_argument("datapath", type=to_unicode)

    args = parser.parse_args()

    ai_sv = "rs://" + args.server_ip + ":" + args.port
    ai_realm = args.realm

    # create a Wamp session object
    session = Component(ComponentConfig(ai_realm, {}))

    # initialize the msgpack serializer
    serializer = MsgPackSerializer()

    # use Wamp-over-rawsocket
    runner = ApplicationRunner(ai_sv, ai_realm, serializers=[serializer])

    runner.run(session, auto_reconnect=True)
