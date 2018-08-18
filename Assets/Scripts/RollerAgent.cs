using System.Collections.Generic;
using UnityEngine;
using MLAgents;

public class RollerAgent : Agent {

    ///public
    
    /// target object (the green block in this case)
    public Transform Target;

    /// wall object (the grey block in this case)
    public Transform Wall;

    /// speed multiplier of ball
    public float speed = 10;

    ///private
    
    /// rigidbody of self (the ball in this case)
    private Rigidbody rBody;

    /// distance to block from ball from the last update
    private float prevDistance = float.MaxValue;

	// Use this to get self's rigidbody component
	void Start () {
        rBody = GetComponent<Rigidbody>();
	}

    // override Agent functions:

    // called when agent resets its self
    public override void AgentReset() {

        if (this.transform.position.y < -1) {

            // ball fell off the floor
            this.transform.position =    Vector3.zero;
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity =        Vector3.zero;
        } else {

            // move target block to new spot
            Target.position = new Vector3(
                Random.value * 8 - 4,
                0.5f,
                Random.value * 8 - 4
                );

            // move wall to new spot away from block
            // and start position of ball
            float distBetweenWallAndTarget = 0;
            float distBetweenWallAndBallStart = 0;
            while (distBetweenWallAndTarget < 1.45f ||
                distBetweenWallAndBallStart < 1.45f) {

                Wall.position = new Vector3(
                    Random.value * 8 - 4,
                    0.5f,
                    Random.value * 8 - 4
                    );
                distBetweenWallAndTarget = Vector3.Distance(Target.position, Wall.position);
                distBetweenWallAndBallStart = Vector3.Distance(Vector3.zero, Wall.position);
            }
        }
    }

    // called for collecting observations to feed into a feature
    // vector, which is then fed into a nueral network, resulting
    // in some calculated action
    // note: (divides observations by 5 to normalize on 10x10 plane)
    public override void CollectObservations() {

        // calculate relative positions
        Vector3 relativePositionTarget = Target.position - this.transform.position;
        Vector3 relativePositionWall = Wall.position - this.transform.position;

        // relative position (to target)
        AddVectorObs(relativePositionTarget.x / 5);
        AddVectorObs(relativePositionTarget.z / 5);

        // relative position (to wall)
        AddVectorObs(relativePositionWall.x / 5);
        AddVectorObs(relativePositionWall.z / 5);

        // distance to edges of platform
        AddVectorObs((this.transform.position.x + 5) / 5);
        AddVectorObs((this.transform.position.x - 5) / 5);
        AddVectorObs((this.transform.position.z + 5) / 5);
        AddVectorObs((this.transform.position.z - 5) / 5);

        // agent velocity
        AddVectorObs(rBody.velocity.x / 5);
        AddVectorObs(rBody.velocity.z / 5);
    }

    // called for providing rewards to the agent, acting
    // based on brain numeric results, and determining
    // if the agent is done with its task yet or not
    public override void AgentAction(float[] vectorAction, string textAction) {

        // relative distance to target from agent
        float distanceToTarget = Vector3.Distance(
            this.transform.position,
            Target.position
            );

        // reached target reward
        if (distanceToTarget < 1.42f) {
            AddReward(1.0f);
            Done();
        }

        // getting closer reward
        if (distanceToTarget < prevDistance) {
            AddReward(0.1f);
        }

        // time penalty
        AddReward(-0.05f);

        // fell off platform penalty & action
        if (this.transform.position.y < -1.0) {
            AddReward(-1.0f);
            Done();
        }
        prevDistance = distanceToTarget;

        // x and z movement actions
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = vectorAction[0];
        controlSignal.z = vectorAction[1];
        rBody.AddForce(controlSignal * speed);
    }
}
