
#assuming that i have a server that takes location and return the name of the room


import rospy
import Vertice, Graph



class MapCreationServer:

    def __init__(self, own_location_server, room_name_server):

        self.graph = Graph()
        self.current_vertice = None
        pass


    def updateMap(self, req):
        room_name = req.room_name
        location = req.location

        #if robot has reached a new room 
        if self.current_vertice is None or self.current_vertice.notName(room_name):
            #create a new vertice with the room name
            new_vertice = Vertice(room_name, location, self.current_vertice)    
            self.graph.addVertice(new_vertice, self.current_vertice)
            self.current_vertice = new_vertice

        #if robot still in the same room
        elif not self.current_vertice.notName(room_name):
            self.current_vertice.addCoord(location)
            self.current_vertice = self.current_vertice.next[0]
        
        else:
            return False
        
        return True
    



if __name__ == '__main__':
    rospy.init_node("map_creation_server", anonymous=True)
    try:
        while not rospy.is_shutdown():
            server = MapCreationServer()
            service = rospy.Service('map_creation', MapCreation, server)
            rospy.loginfo("Map Creation Server is ready")
            rospy.spin()
    except rospy.ROSInterruptException:
        pass