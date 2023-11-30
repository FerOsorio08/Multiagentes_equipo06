from mesa import Agent
import networkx as nx
import matplotlib.pyplot as plt

class Car(Agent):
    """
    Agent that moves randomly.
    Attributes:
        unique_id: Agent's ID 
        direction: Randomly chosen direction chosen from one of eight directions
    """
    def __init__(self, unique_id, model,graph,goal,state):
        """
        Creates a new random agent.
        Args:
            unique_id: The agent's ID
            model: Model reference for the agent
        """
        super().__init__(unique_id, model)
        self.graph=graph
        self.initialPos = (0,0)
        self.goal = goal
        self.light=0
        self.path = None  # Store the path
        self.count=0
        self.path=[]
        self.patience=2
        self.state=state
        self.car=0

    def move(self):
        """ 
        Determines if the agent can move in the direction that was chosen
        """
        possible_steps = self.model.grid.get_neighborhood(
            self.pos,
            moore=True, # Boolean for whether to use Moore neighborhood (including diagonals) or Von Neumann (only up/down/left/right).
            include_center=True)       
        # Checks which grid has traffic light "s"
        trafficLighspace_s= list(map(self.trafficLight_s, possible_steps))
         # Checks which grid has traffic light "S"
        trafficLighspace_S= list(map(self.trafficLight_S, possible_steps))
        #Checks which grid cells have not been visited.
        RoadSpacesUp = list(map(self.road_up, possible_steps))
        RoadSpacesDown = list(map(self.road_down, possible_steps))
        RoadSpacesRight = list(map(self.road_right, possible_steps))
        RoadSpacesLeft = list(map(self.road_left, possible_steps))
        ObstacleSpaces = list(map(self.building, possible_steps))
        CarSpaces = list(map(self.checkCar, possible_steps))
        
        #Saves list of available positions depending on agent type  
        next_move_s = [p for p,f in zip(possible_steps, trafficLighspace_s) if f == True]
        next_move_S = [p for p,f in zip(possible_steps, trafficLighspace_S) if f == True]
        next_move_road_up = [p for p,f in zip(possible_steps, RoadSpacesUp) if f == True]
        next_move_road_down = [p for p,f in zip(possible_steps, RoadSpacesDown) if f == True]
        next_move_road_right = [p for p,f in zip(possible_steps, RoadSpacesRight) if f == True]
        next_move_road_left = [p for p,f in zip(possible_steps, RoadSpacesLeft) if f == True]
        next_move_obstacle = [p for p,f in zip(possible_steps, ObstacleSpaces) if f == True]
        next_move_car = [p for p,f in zip(possible_steps, CarSpaces) if f == True]
       
       #Conditions for Car Agent
        if self.starSignal()==True:
            self.checkrecalculate()
            print("would be:",self.path[0])
            next_move = self.path.pop(0)
            self.model.grid.move_agent(self, next_move)
            print("car is equal to: ", self.car)
            self.car=0
            return
            
        if len(next_move_car) > 1 and self.path[0] in next_move_car:
            #print("pos", "Nextmove",next_move)
            #decrement patience in the car. The car must wait.
            #print("not being able to move")
            x,y=self.pos
            xp,yp=self.path[0]
            print("self.pos",self.pos,"self.path[0]",self.path[0])
            if(x+1==xp and y==yp): #If going to the RIGHT
                if len(next_move_road_right) > 0:
                    print("not moving right")
                    if (xp,yp+1) in next_move_road_right and (xp,yp+1) not in next_move_car and (x,y+1) not in next_move_car:#if right up free
                        print("right up")
                        # self.path=self.a_star_search(self.graph, self.pos, self.goal)
                        # print("CHanging path patience", self.path)
                        self.path.pop(0)
                        self.model.grid.move_agent(self, (xp,yp+1))
                        return
                    elif (xp,yp-1) in next_move_road_right and (xp,yp-1) not in next_move_car  and (x,y-1) not in next_move_car:#if right down free
                        print("right dow")
                        #self.path.pop(0)
                        self.model.grid.move_agent(self, (xp,yp-1))
                        return
                    # else:
                    #     self.patience-=1
            elif(y+1==yp): #If going UP
                print("up")
                if len(next_move_road_up) > 0:
                    print("up not empty")
                    if (xp+1,yp) in next_move_road_up and (xp+1,yp) not in next_move_car:
                        print("op right")
                        self.path.pop(0)
                        self.model.grid.move_agent(self, (xp+1,yp))
                        return
                    elif (xp-1,yp) in next_move_road_up and (xp-1,yp) not in next_move_car:
                        print("left op")
                        self.path.pop(0)
                        self.model.grid.move_agent(self, (xp-1,yp))
                        return
            elif y - 1 == yp:  # If going DOWN
                print("down")
                if len(next_move_road_down) > 0:
                    # print("down not empty")
                    # print("y:", y, "yp:", yp)
                    # print("next_move_road_down:", next_move_road_down)
                    # print("Conditions:", (xp + 1, yp) in next_move_road_down, (xp + 1, yp) not in next_move_car, (xp - 1, yp) in next_move_road_down, (xp - 1, yp) not in next_move_car)

                    if (xp + 1, yp) in next_move_road_down and (xp + 1, yp) not in next_move_car:  # if down right is free
                        print("down right")
                        self.path.pop(0)
                        self.model.grid.move_agent(self, (xp + 1, yp))
                        return
                    elif (xp - 1, yp) in next_move_road_down and (xp - 1, yp) not in next_move_car:  # if down left is free
                        print("down left")
                        self.path.pop(0)
                        self.model.grid.move_agent(self, (xp - 1, yp))
                        return
            elif(x-1==xp and y==yp): #If going to the left.
                if len(next_move_road_left) > 0:
                    print("not moving left")
                    if (xp,yp+1) in next_move_road_left and (xp,yp+1) not in next_move_car:#if right up free
                        print("left up")
                        self.path.pop(0)
                        self.model.grid.move_agent(self, (xp,yp+1))
                        return
                    elif (xp,yp-1) in next_move_road_left and (xp,yp-1) not in next_move_car:#if right down free
                        print("left down")
                        self.path.pop(0)
                        self.model.grid.move_agent(self, (xp,yp-1))
                        return
                #If going LEFT
                
            self.patience-=1
            print("patience decrementing")
            #self.model.grid.move_agent(self, next_move)
            return  
        
        if len(next_move_car) > 1 and self.path[0] in next_move_car:
            #print("pos", "Nextmove",next_move)
            #decrement patience in the car. The car must wait.
            self.patience-=1
            print("patience decrementing")
            #self.model.grid.move_agent(self, next_move)
            return
            
        if len(next_move_s) > 0 and self.light==0 and self.path[0] in next_move_s:
            print("REd light patience decrementing")
            #self.recalculate()
            self.patience-=1
            return 
        if len(next_move_S) > 0 and self.light==0 and self.path[0] in next_move_S:
            print("REd light patience decrementing")
            self.patience-=1
            return 
        if len(self.path) > 0:
            print("normal move")
            next_move = self.path.pop(0)
            print(next_move)
            self.model.grid.move_agent(self, next_move)
            return
        
    # def plot_graph(self, graph):
    #     pos = {node: (node[0], -node[1]) for node in graph.nodes}  # Flip y-axis for visualization
    #     #unflip y-axis
    #     pos = {node: (node[0], node[1]) for node in graph.nodes}
    #     nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=8, font_color='black')
    #     plt.show()
        
    def checkCar(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            if isinstance(agent, Car):
                return True
        return False 

    def trafficLight_s(self, pos):
        #print("Checking for green lists s")
        contents = self.model.grid.get_cell_list_contents(pos)
        #print(contents)
        for agent in contents:
            if isinstance(agent, Traffic_Light):
                #print("hello")
                if agent.traffic_type == True:
                    #print("Check2")
                    if agent.state==False: #if the traffic light is red
                        return True 
                else:
                    print(agent.traffic_type)
            return False
    
    def starSignal(self):
        content = self.model.grid.get_cell_list_contents(self.pos)
        x,y=self.pos
        for agent in content:
            if isinstance(agent, Road):
                if agent.direction=="stararriba" or agent.direction=="starabajo" or agent.direction=="starderecha" or agent.direction=="starizquierda":
                    return True
        return False
        
    def recalculate(self):
        #save path in a temporary variable
        #copy path to a new variable
        print("hola recalculate")
        tempPath=[]
        print("hola recalculate")
        for i in range(len(self.path)):
            tempPath.append(self.path[i])
        print("hola recalculate")
        print(self.pos)
        # print("tempPath", tempPath)
        copyGraph = nx.DiGraph()
        #Hasta aqui revsado
        # #go trough the self.graph and copy it to the copyGraph
        for i in self.graph.nodes:
            copyGraph.add_node(i)
        for i in self.graph.edges:
            copyGraph.add_edge(i[0],i[1])
        print("hola recalculate")

        # # self.plot_graph(copyGraph)
        #remove the elements in the path from the graph
        for i in range(len(tempPath)):
            #remove everything except in the last position and second to last position
            if i != len(tempPath)-1 and i != len(tempPath)-2 and i != 0:
                copyGraph.remove_node(tempPath[i])
        #recalculate the path
        newPath = self.a_star_search(copyGraph, self.pos, self.goal)
        self.path = newPath
        if self.path is None:
            self.path = tempPath
            print("path not found second", self.path)
            return
        # self.plot_graph(self.graph)
        print("recalculated path", self.path)
        
    def checkrecalculate(self):
        content = self.model.grid.get_cell_list_contents(self.pos)
        print("Hello trying 1")
        x,y=self.pos
        for agent in content:
            if isinstance(agent, Road):
                if agent.direction=="stararriba":
                    self.car=0
                    print("arriba")
                    content1 = self.model.grid.get_cell_list_contents((x,y+1))
                    for agent in content1:
                        if isinstance(agent, Car):
                            print("Hello trying 3")
                            print("found car 1")
                            self.car+=1
                    content2 = self.model.grid.get_cell_list_contents((x,y+2))
                    for agent in content2:
                        if isinstance(agent, Car):
                            print("found car 2")
                            self.car+=2
                            if self.car==2:
                                self.recalculate()
                            print("¿recalculating?")
                elif agent.direction=="starabajo":
                    self.car=0
                    print("arriba")
                    content1 = self.model.grid.get_cell_list_contents((x,y-1))
                    for agent in content1:
                        if isinstance(agent, Car):
                            print("Hello trying 3")
                            print("found car 1")
                            self.car+=1
                    content2 = self.model.grid.get_cell_list_contents((x,y-2))
                    for agent in content2:
                        if isinstance(agent, Car):
                            print("found car 2")
                            self.car+=2
                            if self.car==2:
                                self.recalculate()
                                print("¿recalculating?")
                elif agent.direction=="starderecha":
                    self.car=0
                    # content1 = self.model.grid.get_cell_list_contents((x+1,y))
                    # for agent in content1:
                    #     if isinstance(agent, Car):
                    #         print("found car 1")
                    #         self.car=1
                    content2 = self.model.grid.get_cell_list_contents((x+3,y))
                    for agent in content2:
                        if isinstance(agent, Car):
                            print("found car 2")
                            self.car+=1
                    print("IN I")
                    content3 = self.model.grid.get_cell_list_contents((x+4,y))
                    for agent in content3:
                        if isinstance(agent, Car):
                            print("found car RIGHT")
                            self.car+=1
                            if self.car==2:
                                print("right star calc")
                                self.recalculate()
                elif agent.direction=="starizquierda":
                    self.car=0
                    content2 = self.model.grid.get_cell_list_contents((x+3,y))
                    for agent in content2:
                        if isinstance(agent, Car):
                            print("found car izquierda")
                            self.car+=1
                    print("IN I")
                    content3 = self.model.grid.get_cell_list_contents((x+4,y))
                    for agent in content3:
                        if isinstance(agent, Car):
                            print("found car izquierda 2")
                            self.car+=1
                            if self.car==2:
                                print("izq star calc")
                                self.recalculate()
       
    def trafficLight_S(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        #print("S")
        for agent in contents:
            if isinstance(agent, Traffic_Light):
                if agent.traffic_type == False:
                    if agent.state==False: #if traffic light is red
                        return True 
            return False
    
    def road_up(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            x,y=self.pos
            x1,y1=pos
            if isinstance(agent, Road):
                if (x1==x and y1==y+1) or (x1-1==x and y1==y+1) or (x1+1==x and y1==y+1): #only top neighbors
                    if agent.direction =="Up" or agent.direction=="stararriba":
                        print("pos",pos,"self.pos", self.pos)
                        return True
            return False
    
    def road_down(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            x,y=self.pos
            x1,y1=pos
            if isinstance(agent, Road):
                if (x1==x and y1==y-1) or (x1-1==x and y1==y-1) or (x1+1==x and y1==y-1): #only down neighbors
                    if agent.direction =="Down" or agent.direction=="starabajo":
                        print("pos",pos,"self.pos", self.pos)
                        return True
            return False
    
    def road_right(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            x,y=self.pos
            x1,y1=pos
            if isinstance(agent, Road):
                if (x1==x+1 and y1==y+1) or (x1==x+1 and y1==y) or (x1==x+1 and y1==y-1): #only right neighbors
                    if agent.direction =="Right" or agent.direction=="starderecha":
                        print("pos",pos,"self.pos", self.pos)
                        return True
            return False
    
    def road_left(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            x,y=self.pos
            x1,y1=pos
            if isinstance(agent, Road):
                if (x1==x-1 and y1==y+1) or (x1==x-1 and y1==y) or (x1==x-1 and y1==y-1): #only right neighbors
                    if agent.direction =="Left" or agent.direction=="starizquierda":
                        print("pos",pos,"self.pos", self.pos)
                        return True
            return False
    
    def building(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            if isinstance(agent, Obstacle):
                return True
            return False

    #Complejidad O(E +VlogV), donde E es el numero de aristas y V el numero de vertices
    def a_star_search(self, graph, start, goal):
        """A* search to find the shortest path between a start and a goal node.
        Args:
            graph: The graph to search
            start: The start node
            goal: The goal node
        """
        # self.graph.add_node(self.initialPos)
        print("start: ", start)
        print("goal: ", goal)
        x, y = start
        j, i = goal
        try:
            path = nx.astar_path(graph, start, goal, heuristic=self.heuristic)
            #print("path: ", path)
            return path
        except nx.NetworkXNoPath:
            # If no path is found, return None
            print("start", start," goal ", goal)
            return None
    
    #Complejidad O(1)
    def heuristic(self, a, b):
        """Manhattan distance heuristic for A* pathfinding."""
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    #Function to delete agents once they reach their goal
    def deleteAgent(self):
        self.model.grid.remove_agent(self)
        self.model.schedule.remove(self)
    
    def step(self):
        """ 
        Determines the new direction it will take, and then moves
        """
        print(self.pos)
        if self.state==1:
            self.deleteAgent()
        elif self.count==0:
            self.path = self.a_star_search(self.graph, self.pos, self.goal)
            self.count=1
            if self.path is not None:
                #self.checkrecalculate()
                self.path.pop(0)
                self.move()
            else:
                print("path not found first", self.path)
        elif self.path is not None and len(self.path) > 0:
            #self.checkrecalculate()
            self.move()
            if self.pos == self.goal:
                self.state=1
                #self.deleteAgent()
            print("found path", self.path)
        else:
            # If the path is empty, find a new path
            self.path = self.a_star_search(self.graph, self.pos, self.goal)
            print("path not found", self.path)


class Traffic_Light(Agent):
    """
    Traffic light. Where the traffic lights are in the grid.
    """
    def __init__(self, unique_id, model, traffic_type, state = False, timeToChange = 10):
        super().__init__(unique_id, model)
        """
        Creates a new Traffic light.
        Args:
            unique_id: The agent's ID
            model: Model reference for the agent
            state: Whether the traffic light is green or red
            timeToChange: After how many step should the traffic light change color 
        """
        #print("Traffic light created")
        self.traffic_type = traffic_type
        #print("Traffic type: ", self.traffic_type)
        self.state = state
        #print("State: ", self.state)
        self.timeToChange = timeToChange
        #print("Time to change: ", self.timeToChange)

    def step(self):
        """ 
        To change the state (green or red) of the traffic light in case you consider the time to change of each traffic light.
        """
        # print("Traffic light created")
        # print("Traffic type: ", self.traffic_type)
        # print("State: ", self.state)
        # print("Time to change: ", self.timeToChange)
        if self.model.schedule.steps % self.timeToChange == 0:
            self.state = not self.state

class Destination(Agent):
    """
    Destination agent. Where each car should go.
    """
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def step(self):
        pass

class Obstacle(Agent):
    """
    Obstacle agent. Just to add obstacles to the grid.
    """
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def step(self):
        pass

class Road(Agent):
    """
    Road agent. Determines where the cars can move, and in which direction.
    """
    def __init__(self, unique_id, model, direction= "Left"):
        """
        Creates a new road.
        Args:
            unique_id: The agent's ID
            model: Model reference for the agent
            direction: Direction where the cars can move
        """
        super().__init__(unique_id, model)
        self.direction = direction

    def step(self):
        pass
