# TC2008B. Sistemas Multiagentes y Gráficas Computacionales
# Python flask server to interact with Unity. Based on the code provided by Sergio Ruiz.
# Octavio Navarro. October 2023git

from flask import Flask, request, jsonify
from model import CityModel
from agent import *
# Size of the board:
number_agents = 10
width = 28
height = 28
randomModel = None
currentStep = 0

app = Flask("Traffic example")

@app.route('/init', methods=['GET', 'POST'])
def initModel():
    global currentStep, randomModel, number_agents, width, height

    if request.method == 'POST':
        number_agents = int(request.form.get('NAgents'))
        width = int(request.form.get('width'))
        height = int(request.form.get('height'))
        currentStep = 0
        print(request.form)
        print(number_agents, width, height)
        randomModel = CityModel(number_agents)

        return jsonify({"message":"Parameters recieved, model initiated."})
    elif request.method == 'GET':
        number_agents = 10
        width = 30
        height = 30
        currentStep = 0
        randomModel = CityModel(number_agents)

        return jsonify({"message":"Default parameters recieved, model initiated."})


@app.route('/getAgents', methods=['GET'])
def getAgents():
    global randomModel

    if request.method == 'GET':
        # agentPositions = [{"id": str(a.unique_id), "x": x, "y":1, "z":z}
        #                   for a, (x, z) in randomModel.grid.coord_iter()
        #                   if isinstance(a, Car)
        agentPositions = []
        for a,(x,z) in randomModel.grid.coord_iter():
            # if len(a)>1:
            #     print("A",a)
            for agent in a:
                if isinstance(agent, Car):
                    print("Agent", agent)
                    agentPositions += [{"id": str(agent.unique_id), "x": x, "y":1, "z":z}]
        

        return jsonify({'positions':agentPositions})


@app.route('/getObstacles', methods=['GET'])
def getObstacles():
    global randomModel

    if request.method == 'GET':
        carPositions = [{"id": str(a.unique_id), "x": x, "y":1, "z":z}
                        for a, (x, z) in randomModel.grid.coord_iter()
                        if isinstance(a, ObstacleAgent)]

        return jsonify({'positions':carPositions})

@app.route('/getTrafficLight', methods=['GET'])
def getTrafficLight():
    global randomModel

    if request.method == 'GET':
        trafficPositions = [{"id": str(a.unique_id), "x": x, "y":1, "z":z, "state":a.state}
                        for a, (x, z) in randomModel.grid.coord_iter()
                        if isinstance(a, Traffic_Light)]

        return jsonify({'positions':trafficPositions})


@app.route('/update', methods=['GET'])
def updateModel():
    global currentStep, randomModel
    if request.method == 'GET':
        randomModel.step()
        currentStep += 1
        return jsonify({'message':f'Model updated to step {currentStep}.', 'currentStep':currentStep})


if __name__=='__main__':
    app.run(host="localhost", port=8585, debug=True)