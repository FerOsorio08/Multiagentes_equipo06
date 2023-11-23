from mesa import Agent
import networkx as nx

class Car(Agent):
    """
    Agent that moves randomly.
    Attributes:
        unique_id: Agent's ID 
        direction: Randomly chosen direction chosen from one of eight directions
    """
    def __init__(self, unique_id, model,graph,goal):
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
        # #Checks which grid cells have no agent.
        # agentSpaces = list(map(self.agentcheck, possible_steps))
        # #Checks which grid cells have not been visited.
        # notVisited = list(map(self.notVisited, possible_steps))
        
        next_move_s = [p for p,f in zip(possible_steps, trafficLighspace_s) if f == True]
        next_move_S = [p for p,f in zip(possible_steps, trafficLighspace_S) if f == True]
        next_move_road_up = [p for p,f in zip(possible_steps, RoadSpacesUp) if f == True]
        next_move_road_down = [p for p,f in zip(possible_steps, RoadSpacesDown) if f == True]
        next_move_road_right = [p for p,f in zip(possible_steps, RoadSpacesRight) if f == True]
        next_move_road_left = [p for p,f in zip(possible_steps, RoadSpacesLeft) if f == True]
        next_move_obstacle = [p for p,f in zip(possible_steps, ObstacleSpaces) if f == True]
        next_move_car = [p for p,f in zip(possible_steps, CarSpaces) if f == True]
       
        # Cuando el agente esta sobre un semaforo todavía no funciona. 
        
        
        #Se encarga de revisar si existe un coche en ese lugar
        # if len(next_move_car) > 0:
        #     next_move = self.random.choice(next_move_car) 
        #     print("pos", "Nextmove",next_move)
        #     #decrement patience in the car. The car must wait.
        #     self.model.grid.move_agent(self, next_move)
        #     return
        
        #Se encarga de mover hacia la derecha
        if len(next_move_road_right) > 0:
            next_move = self.random.choice(next_move_road_right) 
            print("pos", "Nextmove",next_move)
            x,y=next_move
            self.lastDirection="Right"
            self.model.grid.move_agent(self, (x,y))
            return
        
        #Se encarga de mover hacia la izquierda
        if len(next_move_road_left) > 0:
            next_move = self.random.choice(next_move_road_left) 
            print("pos", "Nextmove",next_move)
            x,y=next_move
            self.model.grid.move_agent(self, (x,y))
            return
        #Se encarga de mover hacia abajo
        if len(next_move_road_down) > 0:
            next_move = self.random.choice(next_move_road_down) 
            print("pos", "Nextmove",next_move)
            x,y=next_move
            self.model.grid.move_agent(self, (x,y))
            return
        #Se encarga de mover hacia arriba
        if len(next_move_road_up) > 0:
            next_move = self.random.choice(next_move_road_up) 
            print("pos", "Nextmove",next_move)
            x,y=next_move
            self.model.grid.move_agent(self, (x,y))
            return
        if len(next_move_s) > 0 and self.light==0:
            print("green Light")
            next_move = self.random.choice(next_move_s)
            x,y=next_move
            self.model.grid.move_agent(self, (x,y))
            #self.light=1
            return True
        if len(next_move_S) > 0 and self.light==0:
            print("green Light")
            next_move = self.random.choice(next_move_S)
            x,y=next_move
            self.model.grid.move_agent(self, (x,y))
            #self.light=1
            return True
    
           
    def checkCar(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            if isinstance(agent, Car):
                return True
        return False 

    def trafficLight_s(self, pos):
        print("Checking for green lists")
        contents = self.model.grid.get_cell_list_contents(pos)
        #print(contents)
        for agent in contents:
            if isinstance(agent, Traffic_Light):
                print("hello")
                if agent.traffic_type == True:
                    print("Check2")
                    if agent.state==True: #if the traffic light is green
                        return True 
                else:
                    print(agent.traffic_type)
            return False
    
    def trafficLight_S(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        #print(contents)
        for agent in contents:
            if isinstance(agent, Traffic_Light):
                if agent.traffic_type == False:
                    if agent.state==True: #if traffic light is green
                        return True 
            return False
    
    def road_up(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        for agent in contents:
            x,y=self.pos
            x1,y1=pos
            if isinstance(agent, Road):
                if (x1==x and y1==y+1) or (x1-1==x and y1==y+1) or (x1+1==x and y1==y+1): #only top neighbors
                    if agent.direction =="Up":
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
                    if agent.direction =="Down":
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
                    if agent.direction =="Right":
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
                    if agent.direction =="Left":
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
            print("path: ", path)
            return path
        except nx.NetworkXNoPath:
            # If no path is found, return None
            return None
    
    #Complejidad O(1)
    def heuristic(self, a, b):
        """Manhattan distance heuristic for A* pathfinding."""
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)
    
    def step(self):
        """ 
        Determines the new direction it will take, and then moves
        """
        if self.path is not None and len(self.path) > 1:
            next_pos = self.path.pop(0)
            self.model.grid.move_agent(self, next_pos)
        else:
            # If the path is empty, find a new path
            self.path = self.a_star_search(self.graph, self.pos, self.goal)


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
        self.traffic_type = traffic_type
        self.state = state
        self.timeToChange = timeToChange

    def step(self):
        """ 
        To change the state (green or red) of the traffic light in case you consider the time to change of each traffic light.
        """
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
