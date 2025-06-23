using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;


/// <summary>
/// hummin bird machine learning agent
/// </summary>
public class HumminBirdAgent : Agent
{
    [Tooltip("force to apply when moving")]
    public float moveForce = 2f;

    [Tooltip("speed of pitch ")]
    public float pitchSpeed = 100f;

    [Tooltip("speed of yaw")]
    public float yawSpeed = 100f;

    [Tooltip("Transform at the tip of the beak ")]
    public Transform beaktip;

    [Tooltip("the agents's camera")]
    public Camera agentCamera;

    [Tooltip("whether this is training mode or gameplay mode")]
    public bool trainingMode;

    //rigidbody of the agent
    new private Rigidbody rigidbody;

    //the flower area that the agent is in
    private FlowerArea flowerArea;

    //the nearest flower to the agent
    private Flower nearestFlower;

    //allows for smooth pitch changes
    private float smoothPitchChange = 0f;

    //allows for smooth yaw changes
    private float smoothYawChange = 0f;

    //maximum angle that the bird can pitch
    private const float MaxPitchAngle = 80f;

    //Maximum distance from the beak tip to accept nectar collision
    private const float BeakTipRadius = 0.008f;

    //wether the agent is frozen
    private bool frozen = false;

    /// <summary>
    /// amount of nectar obtained by agent this episode
    /// </summary>
    public float NectarObtained { get; private set; }


    public override void Initialize()
    {
        rigidbody = GetComponent<Rigidbody>();
        flowerArea = GetComponentInParent<FlowerArea>();

        if (!trainingMode) MaxStep = 0;
    }


    public override void OnEpisodeBegin()
    {
        if (trainingMode)
        {
            //Only reset flowers in training when there is one agent per area
            flowerArea.ResetFlowers();
        }

        //Reset nectar obtained 
        NectarObtained = 0f;

        //zero out velocities so that movement stops before a new episode 
        rigidbody.linearVelocity = Vector3.zero;
        rigidbody.angularVelocity = Vector3.zero;

        //defult to spawning in front of flower 
        bool inFrontOfFlower = true;

        if (trainingMode)
        {
            //Spawn in front of flower 50% of the during training
            inFrontOfFlower = UnityEngine.Random.value > .5f;

        }

        //move agent to safe random position 
        MoveToSafeRandomPosition(inFrontOfFlower);


        //Recalculate nearest flower now that the agent has moved
        UpdateNearestFlower();



    }

    /// <summary>
    /// called when and action is recieved from either the player input or the neural network
    ///
    /// vector action[i]
    /// index 0: move vector x (+1 =right , -1= left)
    /// index 1: move vector y (+1 =up , -1= down)
    /// index 2: move vector z (+1 =forward , -1= backward)
    /// index 3: pitch angle (+1 =pitch up, -1= pitch down)
    /// index 4: yaw angle (+1 =turn right , -1=turn left)
    /// </summary>
    /// <param name="vectorAction">the action to take</param>
    public override void OnActionReceived(ActionBuffers actions)
{
    // If frozen, don't take any actions
    if (frozen) return;

    // Get continuous actions from buffer
    var continuousActions = actions.ContinuousActions;
    
    // Calculate movement vector (using the new API)
    Vector3 move = new Vector3(
        continuousActions[0],
        continuousActions[1],
        continuousActions[2]
    );

    // Add force in the direction of the move vector
    rigidbody.AddForce(move * moveForce);

    // Get the current rotation
    Vector3 rotationVector = transform.rotation.eulerAngles;

    // Get pitch and yaw rotation from actions
    float pitchChange = continuousActions[3];
    float yawChange = continuousActions[4];

    // Calculate smooth rotation changes
    smoothPitchChange = Mathf.MoveTowards(smoothPitchChange, pitchChange, 2f * Time.fixedDeltaTime);
    smoothYawChange = Mathf.MoveTowards(smoothYawChange, yawChange, 2f * Time.fixedDeltaTime);

    // Calculate new pitch and yaw based on smoothed values
    // Clamp pitch to avoid flipping upside down
    float pitch = rotationVector.x + smoothPitchChange * Time.fixedDeltaTime * pitchSpeed;
    if (pitch > 180f) pitch -= 360f;
    pitch = Mathf.Clamp(pitch, -MaxPitchAngle, MaxPitchAngle);

    float yaw = rotationVector.y + smoothYawChange * Time.fixedDeltaTime * yawSpeed;

    // Apply the new rotation
    transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
}

    /// <summary>
    /// collect vector observation from the enviroment
    /// </summary>
    /// <param name="sensor">the vector sensor</param>
    public override void CollectObservations(VectorSensor sensor)
    {

        //if nearest is null , observe an empty array and return early
        if (nearestFlower == null)
        {
            sensor.AddObservation(new float[10]);
            return;
        }
        //observe agent local rotation (4 observation)
        sensor.AddObservation(transform.localRotation.normalized);

        //get vector from from the beak tip to nearest flower
        Vector3 toFlower = nearestFlower.FlowerCenterPosition - beaktip.position;

        //observe a normalized vector pointing to the nearest flower (3 observations)
        sensor.AddObservation(toFlower.normalized);


        //observe a dot product that indicates whether the beak tip is in the front flower
        //+1 --> beak tip infront of the flower ,  -1 beak tip behind flower
        sensor.AddObservation(Vector3.Dot(toFlower.normalized, -nearestFlower.FlowerUpVector.normalized));

        //observe a dot porduct that indicates whether the beak is pointing toward the flower
        // +1 --> beak pointing towards it , -1 --> beak is not pointing at it
        sensor.AddObservation(Vector3.Dot(beaktip.forward.normalized, -nearestFlower.FlowerUpVector.normalized));

        //observe the relative distance from the beak tip to the flower (1 observatio )
        sensor.AddObservation(toFlower.magnitude / FlowerArea.AreaDiameter);



    }

    /// <summary>
    /// when the behaviour of the agent is heuristic only , the function is called
    ///its return value will be fed into <see cref="OnActionRecevied(float[])"> instead of using the neural network
    /// </summary>
    /// <param name="actionsOut"></param>
    public override void Heuristic(in ActionBuffers actionsOut)
{
    // Create placeholders
    Vector3 forward = Vector3.zero;
    Vector3 left = Vector3.zero;
    Vector3 up = Vector3.zero;
    float pitch = 0f;
    float yaw = 0f;

    // Get keyboard reference
    var keyboard = UnityEngine.InputSystem.Keyboard.current;

    // Convert keyboard inputs to movement/turning
    if (keyboard.wKey.isPressed) forward = transform.forward;          // Forward
    if (keyboard.sKey.isPressed) forward = -transform.forward;        // Backward
    
    if (keyboard.dKey.isPressed) left = transform.right;              // Right
    if (keyboard.aKey.isPressed) left = -transform.right;             // Left
    
    if (keyboard.eKey.isPressed) up = transform.up;                   // Up
    if (keyboard.cKey.isPressed) up = -transform.up;                  // Down

    if (keyboard.upArrowKey.isPressed) pitch = 1f;                    // Pitch up
    if (keyboard.downArrowKey.isPressed) pitch = -1f;                 // Pitch down
    
    if (keyboard.rightArrowKey.isPressed) yaw = 1f;                   // Turn right
    if (keyboard.leftArrowKey.isPressed) yaw = -1f;                   // Turn left

    // Combine vectors
    Vector3 combined = (forward + left + up).normalized;

    // Write to continuous actions
    var continuousActions = actionsOut.ContinuousActions;
    continuousActions[0] = combined.x;
    continuousActions[1] = combined.y;
    continuousActions[2] = combined.z;
    continuousActions[3] = pitch;
    continuousActions[4] = yaw;
}

    /// <summary>
    /// prevent agent from moving
    /// </summary>
    public void FreezeAgent()
    {
        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supposed in training ");
        frozen = true;
        rigidbody.Sleep();
    }

    /// <summary>
    /// allow agent from moving
    /// </summary>
    public void UnFreezeAgent()
    {
        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supposed in training ");
        frozen = false;
        rigidbody.WakeUp();
    }

    /// <summary>
    /// move agent to a random safe position (not to colide with other objects)
    /// in front of the flower , also point the beak at the flower 
    /// </summary>
    /// <param name="inFrontOfFlower"> wether to choose a spot in front of a flower </param>
    /// <exception cref="NotImplementedException"></exception>
    private void MoveToSafeRandomPosition(bool inFrontOfFlower)
    {
        bool safePositionFound = false;
        int attemptsRemaining = 100; //prevent infintie loop
        Vector3 potentialPosition = Vector3.zero;
        Quaternion potentialRotation = new Quaternion();

        //Loop until a safe position is found or run out of attempts

        while (!safePositionFound && attemptsRemaining > 0)
        {
            attemptsRemaining--;
            if (inFrontOfFlower)
            {
                //pick a random flower
                Flower randomFlower = flowerArea.Flowers[UnityEngine.Random.Range(0, flowerArea.Flowers.Count)];

                //Position 10 to 20 cm in front of the flower 
                float distanceFromFlower = UnityEngine.Random.Range(0.1f, 0.2f);
                potentialPosition = randomFlower.transform.position + randomFlower.FlowerUpVector * distanceFromFlower;

                //point beak at flower (birds head is center of transform )
                Vector3 toFlower = randomFlower.FlowerCenterPosition - potentialPosition;
                potentialRotation = Quaternion.LookRotation(toFlower, Vector3.up);

            }
            else
            {
                //pick a random height from ground
                float height = UnityEngine.Random.Range(1.2f, 2.5f);

                //pick a random radius from the center of the area
                float radius = UnityEngine.Random.Range(2f, 7f);

                //pick random direction of the rotated around the y axis
                Quaternion direction = Quaternion.Euler(0f, UnityEngine.Random.Range(-180f, 180f), 0f);

                //Combine height, radius and direction to pick a potential position
                potentialPosition = flowerArea.transform.position + Vector3.up * height + direction * Vector3.forward * radius;


                //choose and set random starting pitch and yaw
                float pitch = UnityEngine.Random.Range(-60f, 60f);

                float yaw = UnityEngine.Random.Range(-180f, 180f);

                potentialRotation = Quaternion.Euler(pitch, yaw, 0f);

            }

            //check to see if the agent will collide with anything 
            Collider[] colliders = Physics.OverlapSphere(potentialPosition, 0.05f);

            //Safe position has been found if no colliders are overlapped
            safePositionFound = colliders.Length == 0;

        }

        Debug.Assert(safePositionFound, "couldnt find a safe position to respawn");

        //set position and rotation

        transform.position = potentialPosition;
        transform.rotation = potentialRotation;
    }

    private void UpdateNearestFlower()
    {
        foreach (Flower flower in flowerArea.Flowers)
        {
            if (nearestFlower == null && flower.HasNectar)
            {
                //no current nearest flower and this flower has necter the set it to this flower
                nearestFlower = flower;

            }
            else if (flower.HasNectar)
            {
                //calc distance between current and nearest fllower
                float distanceToFlower = Vector3.Distance(flower.transform.position, beaktip.position);
                float distanceToCurrentNearestFlower = Vector3.Distance(nearestFlower.transform.position, beaktip.position);


                //if current flower is empty or this flower is closer then update nearest flower
                if (!nearestFlower.HasNectar || distanceToFlower < distanceToCurrentNearestFlower)
                {
                    nearestFlower = flower;
                }

            }
        }

    }


    /// <summary>
    /// called when the agent collider enters a trigger collider
    /// </summary>
    /// <param name="other"></param>
    private void OnTriggerEnter(Collider other)
    {
        TriggerEnterOrStay(other);
    }

    /// <summary>
    /// called when the agent collider stays in a trigger collider
    /// </summary>
    /// <param name="other"></param>
    private void OnTriggerStay(Collider other)
    {
        TriggerEnterOrStay(other);
    }

    /// <summary>
    /// handles when agents collider enters or stays in a trigger collider
    /// </summary>
    /// <param name="collider"></param>
    private void TriggerEnterOrStay(Collider collider)
    {
        //check if agent is colliding with nectar
        if (collider.CompareTag("nectar"))
        {
            Vector3 closesPointToBeakTip = collider.ClosestPoint(beaktip.position);

            // check if the closest collision point is close to the beak tip 
            // a collision with anything other than the beak should not count

            if (Vector3.Distance(beaktip.position, closesPointToBeakTip) < BeakTipRadius)
            {
                //look up the flower for the nectar collider 
                Flower flower = flowerArea.GetFlowerFromNectar(collider);

                //attempts to take .01 nectar
                // this is per fixed timestep, meaning it happens every .02 seconds
                float nectarReceived = flower.Feed(.1f);

                //keep track of nectar
                NectarObtained += nectarReceived;

                if (trainingMode)
                {
                    //calc reward for getting nectar
                    float bonus = 0.2f * Mathf.Clamp01(Vector3.Dot(transform.forward.normalized, -nearestFlower.FlowerUpVector.normalized));
                    AddReward(.01f + bonus);


                }

                //if flower empty update nearest  flower
                if (!flower.HasNectar)
                {
                    UpdateNearestFlower();
                }
            }
        }

    }

    /// <summary>
    /// called when the agent collide with something solid 
    /// </summary>
    /// <param name="collision"></param>
    private void OnCollisionEnter(Collision collision)
    {
        if (trainingMode && collision.collider.CompareTag("boundary"))
        {
            // called with the area boundry , give a negative reward
            AddReward(-.5f);
        }
    }


    private void Update()
    {
        //draw a line from the  the beak to the nearest flower
        if (nearestFlower != null)
        {
            Debug.DrawLine(beaktip.position, nearestFlower.FlowerCenterPosition, Color.green);

        }
    }


    private void FixedUpdate()
    {   //aviods scenario where nearest flower nectar is stolen by opponent and not updated
        if (nearestFlower != null && !nearestFlower.HasNectar)
        {
            UpdateNearestFlower();
        }
    }
}
