# -*- coding: utf-8 -*-

from __future__ import print_function

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random

#reset_reason
NONE = 0
GAME_START = 1
SCORE_MYTEAM = 2
SCORE_OPPONENT = 3
GAME_END = 4

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2
        
class Frame(object):
    def __init__(self):
        self.time = None
        self.score = None
        self.reset_reason = None
        self.subimages = None
        self.coordinates = None

class Component(ApplicationSession):
    """
    AI Base + Random Walk
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
            self.end_of_frame = False
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
                self.sub = yield self.subscribe(self.on_event, args.key.decode('unicode-escape'))
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
        print("event received")

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return
        
        # initiate empty frame
        received_frame = Frame()
        
        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']            
        if 'EOF' in f:
            self.end_of_frame = f['EOF']
            
        #print(received_frame.time)
        #print(received_frame.score)
        #print(received_frame.reset_reason)
        #print(self.end_of_frame)
        
        # How to get the robot and ball coordinates: (ROBOT_ID can be 0,1,2,3,4)
        # Available after the 5th event received...
        #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][X])
        #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][Y])
        #print(received_frame.coordinates[MY_TEAM][ROBOT_ID][TH])
        #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][X])
        #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][Y])
        #print(received_frame.coordinates[OP_TEAM][ROBOT_ID][TH])
        #print(received_frame.coordinates[BALL][X])
        #print(received_frame.coordinates[BALL][Y])
        
        if (self.end_of_frame):
            #print("end of frame")

##############################################################################
            #(virtual update() in random_walk.cpp)
            wheels = [random.uniform(-self.max_linear_velocity,self.max_linear_velocity) for _ in range(10)]
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
    session = Component(ComponentConfig(ai_realm.decode('unicode-escape'), {}))

    # initialize the msgpack serializer
    serializer = MsgPackSerializer()
    
    # use Wamp-over-rawsocket
    runner = ApplicationRunner(ai_sv.decode('unicode-escape'), ai_realm.decode('unicode-escape'), serializers=[serializer])
    
    runner.run(session, auto_reconnect=True)