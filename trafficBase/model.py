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
            graph = nx.DiGraph()  # Change to directed graph

            # Load the map file. The map file is a text file where each character represents an agent.
            with open('city_files/2022_base.txt') as baseFile:
                lines = baseFile.readlines()
                self.width = len(lines[0]) - 1
                self.height = len(lines)

                self.grid = MultiGrid(self.width, self.height, torus=False)
                self.schedule = RandomActivation(self)

                # Goes through each character in the map file and creates the corresponding agent.
                for r, row in enumerate(lines):
                    for c, col in enumerate(row):
                        if col in ["v", "^", ">", "<"]:
                            agent = Road(f"r_{r * self.width + c}", self, dataDictionary[col])
                            self.grid.place_agent(agent, (c, self.height - r - 1))
                            graph.add_node((c, self.height - r - 1), direction=col)  # Add direction as an attribute

                        elif col in ["S", "s"]:
                            agent = Traffic_Light(f"tl_{r * self.width + c}", self, False if col == "S" else True,
                                                int(dataDictionary[col]))
                            self.grid.place_agent(agent, (c, self.height - r - 1))
                            self.schedule.add(agent)
                            self.traffic_lights.append(agent)
                            graph.add_node((c, self.height - r - 1), direction=None)  # No direction for traffic lights

                        elif col == "#":
                            agent = Obstacle(f"ob_{r * self.width + c}", self)
                            self.grid.place_agent(agent, (c, self.height - r - 1))

                        elif col == "D":
                            agent = Destination(f"d_{r * self.width + c}", self)
                            self.grid.place_agent(agent, (c, self.height - r - 1))
                            graph.add_node((c, self.height - r - 1), direction=None)  # No direction for destination

            # Add directed edges for neighboring intersections with direction as weight
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
                        direction = graph.nodes[node]['direction']
                        print("direction: ", direction)
                        graph.add_edge(node, neighbor, weight=direction)

            self.num_agents = N

            # Creates the cars
            for i in range(1):
                agent = Car(i, self, graph)
                print("graph: ", graph.nodes)
                self.grid.place_agent(agent, (0, 0))
                self.schedule.add(agent)

            self.running = True

    def step(self):
        '''Advance the model by one step.'''
        self.schedule.step()