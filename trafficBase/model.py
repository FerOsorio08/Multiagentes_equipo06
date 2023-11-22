from mesa import Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid
from agent import *
import json
import networkx as nx

class CityModel(Model):
    """ 
        Creates a model based on a city map.

        Args:
            N: Number of agents in the simulation
    """
    def __init__(self, N):

        # Load the map dictionary. The dictionary maps the characters in the map file to the corresponding agent.
        dataDictionary = json.load(open("city_files/mapDictionary.json"))

        self.traffic_lights = []
        graph = nx.Graph()

        # Load the map file. The map file is a text file where each character represents an agent.
        with open('city_files/2022_base.txt') as baseFile:
            lines = baseFile.readlines()
            self.width = len(lines[0])-1
            self.height = len(lines)

            self.grid = MultiGrid(self.width, self.height, torus = False) 
            self.schedule = RandomActivation(self)

            # Goes through each character in the map file and creates the corresponding agent.
            for r, row in enumerate(lines):
                for c, col in enumerate(row):
                    if col in ["v", "^", ">", "<"]:
                        agent = Road(f"r_{r*self.width+c}", self, dataDictionary[col])
                        self.grid.place_agent(agent, (c, self.height - r - 1))
                        # add edges to the graph
                        if col == "^":
                            graph.add_edge((c, self.height - r - 1), (c, self.height - r))
                        elif col == "v":
                            graph.add_edge((c, self.height - r - 1), (c, self.height - r - 2))
                        elif col == ">":
                            graph.add_edge((c, self.height - r - 1), (c + 1, self.height - r - 1))
                        elif col == "<":
                            graph.add_edge((c, self.height - r - 1), (c - 1, self.height - r - 1))

                    elif col in ["S", "s"]:
                        traffic_type = "S" if col == "S" else "s"
                        agent = Traffic_Light(f"tl_{r*self.width+c}", self, traffic_type, False if col == "S" else True, int(dataDictionary[col]))
                        self.grid.place_agent(agent, (c, self.height - r - 1))
                        self.schedule.add(agent)
                        self.traffic_lights.append(agent)
                        graph.add_node((c, self.height - r - 1))

                    elif col == "#":
                        agent = Obstacle(f"ob_{r*self.width+c}", self)
                        self.grid.place_agent(agent, (c, self.height - r - 1))

                    elif col == "D":
                        agent = Destination(f"d_{r*self.width+c}", self)
                        self.grid.place_agent(agent, (c, self.height - r - 1))
                        graph.add_node((c, self.height - r - 1))

         # Add edges for neighboring intersections
        # Add edges for neighboring intersections
        for node in graph.nodes:
            x, y = node
            neighbors = [
                (x + 1, y),
                (x - 1, y),
                (x, y + 1),
                (x, y - 1)
            ]
            for neighbor in neighbors:
                if neighbor in graph.nodes:
                    neighbor_x, neighbor_y = neighbor
                    # Check if the neighboring node is within the grid bounds
                    if 0 <= neighbor_x < self.width and 0 <= neighbor_y < self.height:
                        graph.add_edge(node, neighbor)
        print("GRAPH", graph.edges)

        self.num_agents = N

        # Creates the cars
        for i in range(1):
            agent = Car(i, self, graph)
            self.grid.place_agent(agent, (0, 0))
            self.schedule.add(agent)

        self.running = True

    def step(self):
        '''Advance the model by one step.'''
        self.schedule.step()