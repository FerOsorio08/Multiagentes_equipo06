from mesa import Agent

class Car(Agent):
    """
    Agent that moves randomly.
    Attributes:
        unique_id: Agent's ID 
        direction: Randomly chosen direction chosen from one of eight directions
    """
    def __init__(self, unique_id, model,graph):
        """
        Creates a new random agent.
        Args:
            unique_id: The agent's ID
            model: Model reference for the agent
        """
        super().__init__(unique_id, model)
        self.graph=graph

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
        
        # Cuando el agente esta sobre un semaforo todavía no funciona. 
        # if len(next_move_s) > 0:
        #     next_move = self.random.choice(next_move_s)
        #     self.model.grid.move_agent(self, next_move)
        #     return
        
        #Se encarga de mover hacia la derecha
        if len(next_move_road_right) > 0:
            next_move = self.random.choice(next_move_road_right) 
            print("pos", "Nextmove",next_move)
            x,y=next_move
            self.model.grid.move_agent(self, (x,y))
            return
        
        #Se encarga de mover hacia la derecha
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
           
        

    def trafficLight_s(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        #print(contents)
        for agent in contents:
            if isinstance(agent, Traffic_Light):
                if agent.traffic_type == "s":
                    return True 
            return False
    
    def trafficLight_S(self, pos):
        contents = self.model.grid.get_cell_list_contents(pos)
        #print(contents)
        for agent in contents:
            if isinstance(agent, Traffic_Light):
                if agent.traffic_type == "S":
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
    
    def step(self):
        """ 
        Determines the new direction it will take, and then moves
        """
        self.move()

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
