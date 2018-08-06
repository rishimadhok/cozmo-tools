"""
  Randomness.fsm demonstrates three ways to introduce randomness in
  state machine behavior.

  1) Use the =RND=> transition to select a destination node at random.

  2) Pass a list of utterances to Say(), and it will choose one at
     random.

  3) Specialize a node class such as Forward or Turn and use Python's
     random() function to generate a random value for the node's
     parameter.

"""

import random

from cozmo_fsm import *

class RandomForward(Forward):
    """Move forward a random distance."""
    def __init__(self,mindist=10,maxdist=50,**kwargs):
        super().__init__(**kwargs)
        self.mindist = mindist if isinstance(mindist,Distance) else distance_mm(mindist)
        self.maxdist = maxdist if isinstance(maxdist,Distance) else distance_mm(maxdist)

    def start(self,event=None):
        self.distance = distance_mm(self.mindist.distance_mm +
                                    random.random() * (self.maxdist.distance_mm - self.mindist.distance_mm))
        super().start(event)

class RandomTurn(Turn):
    """Turn by a random amount."""
    def __init__(self,minangle=20,maxangle=170,**kwargs):
        super().__init__(**kwargs)
        self.minangle = minangle if isinstance(minangle,Angle) else degrees(minangle)
        self.maxangle = maxangle if isinstance(maxangle,Angle) else degrees(maxangle)

    def start(self,event=None):
        angle = self.minangle.degrees + random.random()*(self.maxangle.degrees - self.minangle.degrees)
        self.angle = degrees(angle) if random.random()>=0.5 else degrees(-angle)
        super().start(event)

class Randomness(StateMachineProgram):
    def setup(self):
        """
            startnode: StateNode() =RND=> {fwd, fwd, turn, turn, joke}
    
            fwd: Say(["Forward", "Straight", "Full steam ahead"])
                   =T(2)=> RandomForward() =T(2)=> startnode
    
            turn: Say(["Turn", "Rotate", "Yaw"])
                   =T(2)=> RandomTurn() =C=> startnode
    
            joke: Say(["Watch this", "Hold my beer", "I'm not lost",
                       "Be cool", "Wanna race?"])
                   =C=> StateNode() =T(2)=> startnode
        """
        
        # Code generated by genfsm on Thu Feb 16 23:51:48 2017:
        
        startnode = StateNode() .set_name("startnode") .set_parent(self)
        fwd = Say(["Forward", "Straight", "Full steam ahead"]) .set_name("fwd") .set_parent(self)
        randomforward1 = RandomForward() .set_name("randomforward1") .set_parent(self)
        turn = Say(["Turn", "Rotate", "Yaw"]) .set_name("turn") .set_parent(self)
        randomturn1 = RandomTurn() .set_name("randomturn1") .set_parent(self)
        joke = Say(["Watch this", "Hold my beer", "I'm not lost",
                   "Be cool", "Wanna race?"]) .set_name("joke") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        
        randomtrans1 = RandomTrans() .set_name("randomtrans1")
        randomtrans1 .add_sources(startnode) .add_destinations(fwd,fwd,turn,turn,joke)
        
        timertrans1 = TimerTrans(2) .set_name("timertrans1")
        timertrans1 .add_sources(fwd) .add_destinations(randomforward1)
        
        timertrans2 = TimerTrans(2) .set_name("timertrans2")
        timertrans2 .add_sources(randomforward1) .add_destinations(startnode)
        
        timertrans3 = TimerTrans(2) .set_name("timertrans3")
        timertrans3 .add_sources(turn) .add_destinations(randomturn1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(randomturn1) .add_destinations(startnode)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(joke) .add_destinations(statenode1)
        
        timertrans4 = TimerTrans(2) .set_name("timertrans4")
        timertrans4 .add_sources(statenode1) .add_destinations(startnode)
        
        return self

