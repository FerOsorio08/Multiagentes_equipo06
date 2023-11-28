// TC2008B. Sistemas Multiagentes y Gráficas Computacionales
// C# client to interact with Python. Based on the code provided by Sergio Ruiz.
// Octavio Navarro. October 2023

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Networking;

[Serializable]


public class AgentData
{
    /*
    The AgentData class is used to store the data of each agent.
    
    Attributes:
        id (string): The id of the agent.
        x (float): The x coordinate of the agent.
        y (float): The y coordinate of the agent.
        z (float): The z coordinate of the agent.
    */
    public string id;
    public float x, y, z;
    public float[] goal;

    public AgentData(string id, float x, float y, float z, float[] goal)
    {
        this.id = id;
        this.x = x;
        this.y = y;
        this.z = z;
        this.goal = goal;
    }
}
[Serializable]
public class TrafficLight
{
    public string id;
    public bool state;
    public float x, y, z;

    public TrafficLight(string id, float x, float y, float z,bool state)
    {
        this.id = id;
        this.x = x;
        this.y = y;
        this.z = z;
        this.state = state;
    }
}

[Serializable]

public class AgentsData
{
    /*
    The AgentsData class is used to store the data of all the agents.

    Attributes:
        positions (list): A list of AgentData objects.
    */
    public List<AgentData> positions;

    public AgentsData() => this.positions = new List<AgentData>();
}

public class TrafficLights
{
    public List<TrafficLight> positions;

    public TrafficLights() => this.positions = new List<TrafficLight>();
}

public class AgentController : MonoBehaviour
{
    /*
    The AgentController class is used to control the agents in the simulation.

    Attributes:
        serverUrl (string): The url of the server.
        getAgentsEndpoint (string): The endpoint to get the agents data.
        getObstaclesEndpoint (string): The endpoint to get the obstacles data.
        sendConfigEndpoint (string): The endpoint to send the configuration.
        updateEndpoint (string): The endpoint to update the simulation.
        agentsData (AgentsData): The data of the agents.
        obstacleData (AgentsData): The data of the obstacles.
        agents (Dictionary<string, GameObject>): A dictionary of the agents.
        prevPositions (Dictionary<string, Vector3>): A dictionary of the previous positions of the agents.
        currPositions (Dictionary<string, Vector3>): A dictionary of the current positions of the agents.
        updated (bool): A boolean to know if the simulation has been updated.
        started (bool): A boolean to know if the simulation has started.
        agentPrefab (GameObject): The prefab of the agents.
        obstaclePrefab (GameObject): The prefab of the obstacles.
        floor (GameObject): The floor of the simulation.
        NAgents (int): The number of agents.
        width (int): The width of the simulation.
        height (int): The height of the simulation.
        timeToUpdate (float): The time to update the simulation.
        timer (float): The timer to update the simulation.
        dt (float): The delta time.
    */
    string serverUrl = "http://localhost:8585";
    string getAgentsEndpoint = "/getAgents";
    string getObstaclesEndpoint = "/getObstacles";
    string getTrafficLightEndpoint = "/getTrafficLight";
    string sendConfigEndpoint = "/init";
    string updateEndpoint = "/update";
    AgentsData agentsData, obstacleData;

    TrafficLights trafficData;
    Dictionary<string, GameObject> agents;
    Dictionary<string, Vector3> prevPositions, currPositions;

    Dictionary<string, GameObject> trafficLightColors;

    bool updated = false, started = false;

    public GameObject agentPrefab, obstaclePrefab, floor;
    public int NAgents, width, height;
    public float timeToUpdate = 5.0f;
    private float timer, dt;
    float positionTolerance = 2f; 

    [SerializeField] GameObject carPrefab;
    [SerializeField] GameObject trafficLightPrefab;

    void Start()
    {
        agentsData = new AgentsData();
        trafficData = new TrafficLights();

        prevPositions = new Dictionary<string, Vector3>();
        currPositions = new Dictionary<string, Vector3>();

        //traffic light color dictionary
        trafficLightColors = new Dictionary<string, GameObject>();

        // trafficLightsD = new Dictionary<string, GameObject>();

        agents = new Dictionary<string, GameObject>();
        
        timer = timeToUpdate;

        // Launches a couroutine to send the configuration to the server.
        StartCoroutine(SendConfiguration());
    }

    private void Update() 
    {
        if(timer < 0)
        {
            timer = timeToUpdate;
            updated = false;
            StartCoroutine(UpdateSimulation());
            
        }

        if (updated)
        {
            timer -= Time.deltaTime;

            // Iterates over the agents to update their positions.
            // The positions are interpolated between the previous and current positions.
            

            // float t = (timer / timeToUpdate);
            // dt = t * t * ( 3f - 2f*t);
        }
    }
 
    IEnumerator UpdateSimulation()
    {
        UnityWebRequest www = UnityWebRequest.Get(serverUrl + updateEndpoint);
        yield return www.SendWebRequest();
 
        if (www.result != UnityWebRequest.Result.Success)
            Debug.Log(www.error);
        else 
        {
            StartCoroutine(GetAgentsData());
            StartCoroutine(GetTrafficData());
        }
    }

    IEnumerator SendConfiguration()
    {
        /*
        The SendConfiguration method is used to send the configuration to the server.

        It uses a WWWForm to send the data to the server, and then it uses a UnityWebRequest to send the form.
        */
        WWWForm form = new WWWForm();

        form.AddField("NAgents", NAgents.ToString());
        form.AddField("width", width.ToString());
        form.AddField("height", height.ToString());

        UnityWebRequest www = UnityWebRequest.Post(serverUrl + sendConfigEndpoint, form);
        www.SetRequestHeader("Content-Type", "application/x-www-form-urlencoded");

        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success)
        {
            Debug.Log(www.error);
        }
        else
        {
            Debug.Log("Configuration upload complete!");
            Debug.Log("Getting Agents positions");

            // Once the configuration has been sent, it launches a coroutine to get the agents data.
            StartCoroutine(GetAgentsData());
            StartCoroutine(GetTrafficData());
        }
    }

    IEnumerator GetAgentsData() 
    {
        // The GetAgentsData method is used to get the agents data from the server.
        UnityWebRequest www = UnityWebRequest.Get(serverUrl + getAgentsEndpoint);
        yield return www.SendWebRequest();
 
        if (www.result != UnityWebRequest.Result.Success)
            Debug.Log(www.error);
        else 
        {
            // Once the data has been received, it is stored in the agentsData variable.
            // Then, it iterates over the agentsData.positions list to update the agents positions.
            agentsData = JsonUtility.FromJson<AgentsData>(www.downloadHandler.text);
            Debug.Log("Agents Data");
            Debug.Log(www.downloadHandler.text);

            foreach(AgentData agent in agentsData.positions)
            {
                Vector3 newAgentPosition = new Vector3(agent.x, agent.y, agent.z);
                    if(!agents.ContainsKey(agent.id))
                    {
                        // prevPositions[agent.id] = newAgentPosition;
                        agents[agent.id] = Instantiate(carPrefab, Vector3.zero, Quaternion.identity);   
                        ApplyTransforms applyTransforms = agents[agent.id].GetComponentInChildren<ApplyTransforms>();
                        applyTransforms.getPosition(newAgentPosition, true);
                    }
                     else
                    {
                        ApplyTransforms applyTransforms = agents[agent.id].GetComponentInChildren<ApplyTransforms>();
                        applyTransforms.getPosition(newAgentPosition, false);

                        // Check if the agent has reached its goal within the tolerance
                        if (Mathf.Abs(agent.goal[0] - newAgentPosition.x) < positionTolerance &&
                            Mathf.Abs(agent.goal[1] - newAgentPosition.y) < positionTolerance)
                        {
                            Debug.Log("Agent " + agent.id + " has reached its goal");
                            Destroy(agents[agent.id]);
                            agents.Remove(agent.id);
                        }
                    }
            }
            updated = true;
        }
    }

    // traffic light si solo al inicio
    //hacer script para cambiar la luz del semadforo
    // desde aqui se accese al estado del semaforo y se llaman los metodos para cambiarlo

    IEnumerator GetTrafficData() 
    {
        UnityWebRequest www = UnityWebRequest.Get(serverUrl + getTrafficLightEndpoint);
        yield return www.SendWebRequest();
 
        if (www.result != UnityWebRequest.Result.Success)
            Debug.Log(www.error);
        else 
        {
            trafficData = JsonUtility.FromJson<TrafficLights>(www.downloadHandler.text);

            Debug.Log("Traffic Data");
            Debug.Log(www.downloadHandler.text);

            // Debug.Log("Getting Traffic positions");
            // Debug.Log(trafficData.positions);

            Debug.Log("Traffic Data Count");
            Debug.Log(trafficData.positions.Count);

              foreach (TrafficLight traffic in trafficData.positions)
            {
                if (!started)
                {
                    trafficLightColors[traffic.id] = Instantiate(trafficLightPrefab, new Vector3(traffic.x, traffic.y, traffic.z), Quaternion.identity);
                }
                Light greenLight = trafficLightColors[traffic.id].transform.Find("green").GetComponent<Light>();
                Light redLight = trafficLightColors[traffic.id].transform.Find("red").GetComponent<Light>();
                greenLight.enabled = traffic.state;  // Access the state property from the TrafficLight object
                redLight.enabled = !traffic.state;
                
                Debug.Log("Traffic state: " + traffic.state);
            }

            if (!started) started = true;

        }
    }
}


