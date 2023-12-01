#Lucía Barrenechea y Fernanda Osorio
# 30 de noviembre del 2023
#Proyecto de Mobilidad Urbana
#Modelo de tráfico basado en agentes
#Api para visualizar el modelo de tráfico basado en agentes y mandar los datos a la interfaz gráfica

from flask import Flask, request, jsonify
from model import CityModel
from agent import *
# Size of the board:
number_agents = 10
width = 24
height = 24
randomModel = None
currentStep = 0

# Create the app:
app = Flask("Traffic example")

# Define the routes:
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
        width = 24
        height = 24
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
                    agentPositions += [{"id": str(agent.unique_id), "x": x, "y":0, "z":z, "goal":agent.goal, "state":agent.state}]
        

        return jsonify({'positions':agentPositions})


@app.route('/getObstacles', methods=['GET'])
def getObstacles():
    global randomModel

    if request.method == 'GET':
        carPositions = [{"id": str(a.unique_id), "x": x, "y":0, "z":z}
                        for a, (x, z) in randomModel.grid.coord_iter()
                        if isinstance(a, ObstacleAgent)]

        return jsonify({'positions':carPositions})

@app.route('/getTrafficLight', methods=['GET'])
def getTrafficLight():
    global randomModel

    if request.method == 'GET':
        trafficPositions = []
        for a,(x,z) in randomModel.grid.coord_iter():
            for agent in a:
                if isinstance(agent, Traffic_Light):
                    trafficPositions += [{"id": str(agent.unique_id), "x": x, "y":0, "z":z, "state":agent.state}]
        # trafficPositions = [{"id": str(a.unique_id), "x": x, "y":0, "z":z, "state":a.state}
        #                 for a, (x, z) in randomModel.grid.coord_iter()
        #                 if isinstance(a, Traffic_Light)]

        return jsonify({'positions':trafficPositions})


@app.route('/update', methods=['GET'])
def updateModel():
    global currentStep, randomModel
    if request.method == 'GET':
        randomModel.step()
        currentStep += 1
        if currentStep == 1002:
            raise KeyboardInterrupt
        return jsonify({'message':f'Model updated to step {currentStep}.', 'currentStep':currentStep})


# Run the app:
if __name__=='__main__':
    app.run(host="localhost", port=8585, debug=True)