#!/usr/bin/python3

# File: commentator_skeleton.py
# Date: Jan. 23, 2018
# Description: AI commentator skeleton algorithm
# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Current Developer: Chansol Hong (cshong@rit.kaist.ac.kr)

from __future__ import print_function

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random

import base64
import numpy as np

#reset_reason
NONE = 0
GAME_START = 1
SCORE_MYTEAM = 2
SCORE_OPPONENT = 3
GAME_END = 4
DEADLOCK = 5 # when the ball is stuck for 5 seconds

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2

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
        self.subimages = None
        self.coordinates = None

class Component(ApplicationSession):
    """
    AI Base + Commentator Skeleton
    """ 

    def __init__(self, config):
        ApplicationSession.__init__(self, config)

    def onConnect(self):
        print("Transport connected")
        self.join(self.config.realm)

    @inlineCallbacks
    def onJoin(self, details):
        print("session attached")

##############################################################################
        def init_variables(self, info):
            # Here you have the information of the game (virtual init())
            # List: game_time, goal, number_of_robots, penalty_area, codewords,
            #       robot_height, robot_radius, max_linear_velocity, field, team_info,
            #       {rating, name}, axle_length, resolution, ball_radius
            # self.game_time = info['game_time']
            # self.field = info['field']
            self.field = info['field']
            self.resolution = info['resolution']
            self.colorChannels = 3
            self.end_of_frame = False
            self.image = Received_Image(self.resolution, self.colorChannels)
            print("Initializing variables for commentator...")
            return
##############################################################################
            
        try:
            info = yield self.call(u'aiwc.get_info', args.key)
        except Exception as e:
            print("Error: {}".format(e))
        else:
            print("Got the game info successfully (commentator)")
            try:
                self.sub = yield self.subscribe(self.on_event, args.key)
                print("Subscribed with subscription ID {}".format(self.sub.id))
            except Exception as e2:
                print("Error: {}".format(e2))
               
        init_variables(self, info)
        
        try:
            yield self.call(u'aiwc.ready', args.key)
        except Exception as e:
            print("Error: {}".format(e))
        else:
            print("I am the commentator for this game!")
            
            
    @inlineCallbacks
    def on_event(self, f):        
        #print("event received")

        @inlineCallbacks
        def set_comment(self, commentary):
            yield self.call(u'aiwc.commentate', args.key, commentary)
            return
        
        # initiate empty frame
        received_frame = Frame()
        received_subimages = []        

        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
            # Comment the next lines if you don't need to use the image information
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
            
        #print(received_frame.time)
        #print(received_frame.score)
        #print(received_frame.reset_reason)
        #print(self.end_of_frame)
        
        if (received_frame.reset_reason == GAME_START):
            set_comment(self, "Game has begun")
        elif (received_frame.reset_reason == DEADLOCK):
            set_comment(self, "Position is reset since no one touched the ball") 

        if (self.end_of_frame):
            #print("end of frame")

            # How to get the robot and ball coordinates: (ROBOT_ID can be 0,1,2,3,4)
            #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][X])            
            #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][Y])
            #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][TH])
            #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][X])
            #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][Y])
            #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][TH])
            #print(received_frame.coordinates[OP_TEAM][0][X])
            #print(received_frame.coordinates[OP_TEAM][0][Y])
            #print(received_frame.coordinates[OP_TEAM][0][TH])        
            #print(received_frame.coordinates[BALL][X])
            #print(received_frame.coordinates[BALL][Y])

            # To get the image at the end of each frame use the variable:
            # self.image.ImageBuffer

            if (received_frame.coordinates[BALL][X] >= (self.field[X] / 2)):
                set_comment(self, "A Team scored!!")
            elif (received_frame.coordinates[BALL][X] <= (-self.field[X] / 2)):
                set_comment(self, "B Team scored!!")

            if (received_frame.reset_reason == GAME_END):
                print("Game ended.")

                if (received_frame.score[0] > received_frame.score[1]):
                    set_comment(self, "A Team won")
                elif (received_frame.score[0] > received_frame.score[1]):
                    set_comment(self, "B Team won")
                else:
                    set_comment(self, "The game ended in a draw")

##############################################################################
                #(virtual finish())
                #save your data
                with open(args.datapath + '/result.txt', 'w') as output:
                    #output.write('yourvariables')
                    output.close()
                #unsubscribe; reset or leave  
                yield self.sub.unsubscribe()
                print("Commentator Unsubscribed...")
                try:
                    yield self.leave()
                except Exception as e:
                    print("Error: {}".format(e))
##############################################################################
            
            self.end_of_frame = False


    def onDisconnect(self):
        print("commentator disconnected")
        if reactor.running:
            reactor.stop()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument("server_ip")
    parser.add_argument("port")
    parser.add_argument("realm")
    parser.add_argument("key")
    parser.add_argument("datapath")
    
    args = parser.parse_args()
    #print ("Arguments:")
    #print (args.server_ip)
    #print (args.port)
    #print (args.realm)
    #print (args.key)
    #print (args.datapath)
    
    ai_sv = "rs://" + args.server_ip + ":" + args.port
    ai_realm = args.realm
    
    # create a Wamp session object
    session = Component(ComponentConfig(ai_realm, {}))

    # initialize the msgpack serializer
    serializer = MsgPackSerializer()
    
    # use Wamp-over-rawsocket
    runner = ApplicationRunner(ai_sv, ai_realm, serializers=[serializer])
    
    runner.run(session, auto_reconnect=True)
