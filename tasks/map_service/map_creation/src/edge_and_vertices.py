class Vertice:

    def __init__(self, name, coord, next):
        self.name = name 
        self.coords = {coord} 
        self.next = [next]
    
    def addCoord(self, new_coord):
        self.coords.add(new_coord)
    
    def setNext(self, next):
        if next not in self.next:
            self.next.append(next)
    
    def not(other):
        return not self.name == other.name

    def notName(name):
        return not self.name == name
    
    def _compress(self):
        #compress slef.coords with hugh_transform
        pass


class Graph:

    def __init__(self):

        self.size = 0
        self.adjLists = {}

    def hasVertice(self, vertice):
        for v in self.adjLists:
            if vertice.name == v.name:
                return true
        return false

    
    def addVertice(self, vertice, previous_vertice = None):
        if not self.hasVertice(vertice) and previous_vertice is not None:
            #updates previous vertice and add new vertice into graph
            previous_vertice.setNext(vertice)
            temp = self.adjLists['previous_vertice']
            self.adjLists.update({previous_vertice.name : temp + [vertice.name] })

            vertice.setNext(previous_vertice)
            self.adjLists.update({vertice.name : [previous_vertice.name]})
        elif:
            self.adjLists.update({vertice.name : []})
            self.pre
        else:
            return -1



