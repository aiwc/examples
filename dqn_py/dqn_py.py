#!/usr/bin/python3

# Basic Deep Q Network example in Python 3
# Run the trained model

from __future__ import print_function

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random
import math

import base64
import numpy as np

#from PIL import Image
from dqn_nn import NeuralNetwork

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

#path to your checkpoint
CHECKPOINT = "/home/aiwc/Desktop/test_world/examples/dqn_py/dqn.ckpt"

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
    AI Base + Deep Q Network example
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
            # Here you have the information of the game (virtual init() in random_walk.cpp)
            # List: game_time, goal, number_of_robots, penalty_area, codewords,
            #       robot_size, max_linear_velocity, field, team_info,
            #       {rating, name}, axle_length, resolution, ball_radius
            # self.game_time = info['game_time']
            # self.field = info['field']
            self.max_linear_velocity = info['max_linear_velocity']
            self.resolution = info['resolution']
            self.colorChannels = 3 # nf in dqn_main.py
            self.end_of_frame = False
            self.image = Received_Image(self.resolution, self.colorChannels)
            self._frame = 0 
            self.Q = NeuralNetwork(None, CHECKPOINT, False) # 2nd term: False to start training from scratch, use CHECKPOINT to load a checkpoint
            print("Initializing variables...")
            return
##############################################################################
            
        try:
            info = yield self.call(u'aiwc.get_info', args.key)
        except Exception as e:
            print("Error: {}".format(e))
        else:
            print("Got the game info successfully")
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
            print("I am ready for the game!")
            
            
    @inlineCallbacks
    def on_event(self, f):        
        #print("event received")

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return

        def set_action(action_number):
            if action_number == 0:
                return [0.8, 0.8, 0, 0, 0, 0, 0, 0, 0 ,0] # Go Forward with fixed velocity
            elif action_number == 1:
                return [-0.8, 0.8, 0, 0, 0, 0, 0, 0, 0 ,0] # Turn Left with fixed velocity
            elif action_number == 2:
                return [0.8, -0.8, 0, 0, 0, 0, 0, 0, 0 ,0] # Turn right with fixed velocity
            elif action_number == 3:
                return [-0.8, -0.8, 0, 0, 0, 0, 0, 0, 0 ,0] # Go Backward
            elif action_number == 4:
                return [0, 0, 0, 0, 0, 0, 0, 0, 0 ,0] # Do not move
        
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
	    
            self._frame += 1        
            print(self._frame)

            # To get the image at the end of each frame use the variable:
            #print(self.image.ImageBuffer)

##############################################################################
            #(virtual update() in random_walk.cpp)
            wheels = []

            # State

            # If you want to use the image as the input for your network
            # You can use pillow: PIL.Image to get and resize the input frame as follows
            #img = Image.fromarray((self.image.ImageBuffer/255).astype('uint8'), 'RGB') # Get normalized image as a PIL.Image object
            #resized_img = img.resize((NEW_X,NEW_Y))
            #final_img = np.array(resized_img)

            # Example: using the normalized coordinates for robot 0 and ball
            position = [received_frame.coordinates[MY_TEAM][0][X]/1.25, received_frame.coordinates[MY_TEAM][0][Y]/0.9, received_frame.coordinates[MY_TEAM][0][TH]/(2*math.pi),
                        received_frame.coordinates[BALL][X]/1.25, received_frame.coordinates[BALL][Y]/0.9]

            # Action

            action = self.Q.BestAction(np.array(position)) # using CNNs use final_img as input

            print(action)

            # Set robot wheels
            wheels = set_action(action)
            set_wheel(self, wheels)

##############################################################################            

            if(received_frame.reset_reason == GAME_END):
                print("Game ended.")

##############################################################################
                #(virtual finish() in random_walk.cpp)
                #save your data
                with open(args.datapath + '/result.txt', 'w') as output:
                    #output.write('yourvariables')
                    output.close()
                #unsubscribe; reset or leave  
                yield self.sub.unsubscribe()
                print("Unsubscribed...")
                self.leave()
##############################################################################
            
            self.end_of_frame = False

    
    def onDisconnect(self):
        print("disconnected")
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
    
    runner.run(session, auto_reconnect=False)
