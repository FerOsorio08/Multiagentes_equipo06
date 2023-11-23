from mesa import Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid
from agent import *
import json
import networkx as nx
import matplotlib.pyplot as plt

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
            goal = (0, 0)  # Change to destination

            # Load the map file. The map file is a text file where each character represents an agent.
            with open('city_files/2022modified.txt') as baseFile:
                lines = baseFile.readlines()
                self.width = len(lines[0]) - 1
                self.height = len(lines)

                self.grid = MultiGrid(self.width, self.height, torus=False)
                self.schedule = RandomActivation(self)

                # Goes through each character in the map file and creates the corresponding agent.
                for r, row in enumerate(lines):
                    for c, col in enumerate(row):
                        if col in ["v", "^", ">", "<", "*","I","X","@"]:
                            agent = Road(f"r_{r * self.width + c}", self, dataDictionary[col])
                            self.grid.place_agent(agent, (c, self.height - r - 1))
                            graph.add_node((c, self.height - r - 1), direction=col)  # Add direction as an attribute

                        elif col in ["S", "s"]:
                            agent = Traffic_Light(f"tl_{r * self.width + c}", self, False if col == "S" else True, int(dataDictionary[col]))
                            self.grid.place_agent(agent, (c, self.height - r - 1))
                            self.schedule.add(agent)
                            self.traffic_lights.append(agent)
                            # Add an attribute to mark "S" as "long" and "s" as "short"
                            graph.add_node((c, self.height - r - 1), direction=None, signal_type="long" if col == "S" else "short")
                            # print("signal_type: ", graph.nodes[(c, self.height - r - 1)]['signal_type'])


                        elif col == "#":
                            agent = Obstacle(f"ob_{r * self.width + c}", self)
                            self.grid.place_agent(agent, (c, self.height - r - 1))

                        elif col == "D":
                            agent = Destination(f"d_{r * self.width + c}", self)
                            self.grid.place_agent(agent, (c, self.height - r - 1))
                            graph.add_node((c, self.height - r - 1), direction=None)  # No direction for destination
                            goal = (c, self.height - r - 1)
                            print("goal: ", goal)


            # Add directed edges for neighboring intersections with direction as weight
            # for node in graph.nodes:
            #     x, y = node
            #     neighbors = [
            #         (x + 1, y+1),
            #         (x - 1, y+1),
            #         (x, y + 1)
            #         (x, y - 1)
            #     ]
            #     for neighbor in neighbors:
            #         if neighbor in graph.nodes:
            #             direction = graph.nodes[node]['direction']
            #             # print("direction: ", direction)
            #             graph.add_edge(node, neighbor, weight=direction)
            direction_weights = {
                "v": 1,
                "^": 2,
                ">": 3,
                "<": 4,
                "*": 1,
                "I": 4,
                "X": 3,
                "@": 2
            }
            
            for node in graph.nodes:
                x, y = node
                # x1,y1 = neighbor_node

                # UP
                if graph.nodes[node]['direction'] == "^" or graph.nodes[node]['direction'] == "@":
                    if (x, y + 1) in graph.nodes and 'signal_type' in graph.nodes[(x, y + 1)]:
                        if graph.nodes[(x, y + 1)]['signal_type'] == "long" or graph.nodes[(x, y + 1)]['signal_type'] == "short":
                            neighbors = [
                                (x + 1, y + 1),
                                (x - 1, y + 1),
                                (x, y + 1)
                            ]
                            for neighbor in neighbors:
                                if neighbor in graph.nodes:
                                    direction = graph.nodes[node]['direction']
                                    weight = direction_weights.get(direction, 0) 
                                    graph.add_edge(node, neighbor, weight=weight)
                                    # graph.add_edge(neighbor, (x, y - 2), weight=direction)
                            weight = direction_weights.get(direction, 0) 
                            graph.add_edge((x, y + 1), (x, y + 2), weight=weight)
                    else:
                        neighbors = [
                            (x + 1, y + 1),
                            (x - 1, y + 1),
                            (x, y + 1)
                        ]
                        for neighbor in neighbors:
                            if neighbor in graph.nodes:
                                direction = graph.nodes[node]['direction']
                                weight = direction_weights.get(direction, 0) 
                                graph.add_edge(node, neighbor, weight=weight)
                
                #DOWN
                if graph.nodes[node]['direction'] == "v" or graph.nodes[node]['direction'] == "*":
                    if (x, y - 1) in graph.nodes and 'signal_type' in graph.nodes[(x, y -1)]:
                        if graph.nodes[(x, y - 1)]['signal_type'] == "long" or graph.nodes[(x, y - 1)]['signal_type'] == "short":
                            neighbors = [
                                (x + 1, y - 1),
                                (x - 1, y - 1),
                                (x, y - 1)
                            ]
                            for neighbor in neighbors:
                                if neighbor in graph.nodes:
                                    direction = graph.nodes[node]['direction']
                                    weight = direction_weights.get(direction, 0) 
                                    graph.add_edge(node, neighbor, weight=weight)
                                    #add next edge to S node
                                    # graph.add_edge(neighbor, (x, y + 2), weight=direction)
                            weight = direction_weights.get(direction, 0) 
                            graph.add_edge((x, y - 1), (x, y - 2), weight=weight)
                                
                    else:
                        neighbors = [
                            (x + 1, y - 1),
                            (x - 1, y - 1),
                            (x, y - 1)
                        ]
                        for neighbor in neighbors:
                            if neighbor in graph.nodes:
                                direction = graph.nodes[node]['direction']
                                weight = direction_weights.get(direction, 0) 
                                graph.add_edge(node, neighbor, weight=weight)
                
                #LEFT
                if graph.nodes[node]['direction'] == "<" or graph.nodes[node]['direction'] == "I":
                    if (x - 1, y) in graph.nodes and 'signal_type' in graph.nodes[(x - 1, y)]:
                        if graph.nodes[(x - 1, y)]['signal_type'] == "long" or graph.nodes[(x - 1, y)]['signal_type'] == "short":
                            neighbors = [
                                (x - 1, y + 1),
                                (x - 1, y - 1),
                                (x - 1, y)
                            ]
                            for neighbor in neighbors:
                                if neighbor in graph.nodes:
                                    direction = graph.nodes[node]['direction']
                                    weight = direction_weights.get(direction, 0) 
                                    graph.add_edge(node, neighbor, weight=weight)
                                    # graph.add_edge(neighbor, (x + 2, y), weight=direction)
                            weight = direction_weights.get(direction, 0) 
                            graph.add_edge((x - 1, y), (x - 2, y), weight=weight)
                    else:
                        neighbors = [
                            (x - 1, y + 1),
                            (x - 1, y - 1),
                            (x - 1, y)
                        ]
                        for neighbor in neighbors:
                            if neighbor in graph.nodes:
                                direction = graph.nodes[node]['direction']
                                weight = direction_weights.get(direction, 0) 
                                graph.add_edge(node, neighbor, weight=weight)
                #Right
                if graph.nodes[node]['direction'] == ">" or graph.nodes[node]['direction'] == "X":
                    if (x + 1, y) in graph.nodes and 'signal_type' in graph.nodes[(x + 1, y)]:
                        if graph.nodes[(x + 1, y)]['signal_type'] == "long" or graph.nodes[(x + 1, y)]['signal_type'] == "short":
                            neighbors = [
                                (x + 1, y + 1),
                                (x + 1, y - 1),
                                (x + 1, y)
                            ]
                            for neighbor in neighbors:
                                if neighbor in graph.nodes:
                                    direction = graph.nodes[node]['direction']
                                    weight = direction_weights.get(direction, 0) 
                                    graph.add_edge(node, neighbor, weight=weight)
                                    # graph.add_edge(neighbor, (x - 2, y), weight=direction)
                            weight = direction_weights.get(direction, 0) 
                            graph.add_edge((x + 1, y), (x + 2, y), weight=weight)
                    else:
                        if (x==12 and y==9) or(x==11 and y==9):
                            neighbors = [
                            (x + 1, y - 1),
                            (x + 1, y)
                            ]
                            for neighbor in neighbors:
                                if neighbor in graph.nodes:
                                    direction = graph.nodes[node]['direction']
                                    weight = direction_weights.get(direction, 0) 
                                    graph.add_edge(node, neighbor, weight=weight)
                
                        else:
                            neighbors = [
                                (x + 1, y + 1),
                                (x + 1, y - 1),
                                (x + 1, y)
                            ]
                            for neighbor in neighbors:
                                if neighbor in graph.nodes:
                                    direction = graph.nodes[node]['direction']
                                    weight = direction_weights.get(direction, 0) 
                                    graph.add_edge(node, neighbor, weight=weight)
                
                # if graph.nodes[node]['direction'] == "<":
                #     neighbors = [
                #         (x - 1, y + 1),
                #         (x - 1, y),
                #         (x - 1, y - 1)
                #     ]
                    
                #     for neighbor in neighbors:
                #         if neighbor in graph.nodes:
                #             direction = graph.nodes[node]['direction']
                #             graph.add_edge(node, neighbor, weight=direction)

            self.num_agents = N

            # Creates the cars
            for i in range(1):
                agent = Car(i, self, graph,goal)
                # print("graph: ", graph.nodes)
                self.grid.place_agent(agent, (0, 0))
                self.schedule.add(agent)

            self.running = True
            self.plot_graph(graph)

    def plot_graph(self, graph):
        pos = {node: (node[0], -node[1]) for node in graph.nodes}  # Flip y-axis for visualization
        #unflip y-axis
        pos = {node: (node[0], node[1]) for node in graph.nodes}
        nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=8, font_color='black')
        plt.show()

    def step(self):
        '''Advance the model by one step.'''
        self.schedule.step()