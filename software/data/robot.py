class Position:
    def __init__(self,x,y,ori):
        self.x=x
        self.y=y
        self.ori=ori

class Velocity:
    def __init__(self,wl,wr,v):
        self.wl=wl
        self.wr=wr
        self.v=v


class Robot(Position,Velocity):
    def __init__(self,x,y,ori,wl,wr,v,name):
        Position.__init__(self,x,y,ori)
        Velocity.__init__(self,wl,wr,v)
        self.name=name
    
