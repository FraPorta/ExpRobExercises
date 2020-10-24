
## class Map
# Map structure definition and functions
class PetMap(self):
    ## The constructor
    #  @param self The object pointer
    def __init__(self):
        # initialization of dimension and positions on the Map
        self.actualX = 0
        self.actualY = 0
        self.totalX = 100
        self.totalY = 100

    ## updateMap
    # @param x X position
    # @param y Y position
    def updateMap(self,x,y):
        self.actualX = x
        self.actualY = y


