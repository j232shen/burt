using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Statistics;
using BurtSharp.Util.ClassExtensions;
using BurtSharp.CoAP.MsgTypes;
using UnityEngine;
using UnityEngine.UI;
using BurtSharp.Control;
using System.IO;
using CsvHelper;
using System.Threading;
using System.Diagnostics;

public class robotController : MonoBehaviour {

	// Scales the robot position to more closely appoximate the Unity workspace for this game.
	public float positionScale = 40.0f;

	// Robot state
	private Vector3 tool_position;

	public static readonly float g = 9.81f;
	public static readonly int kDof = 3; //num degrees of freedom
	public static readonly int kNumDim = 3; //num cartesian dimensions

	private float kpTool = 800.0f;  // N/m
	private float kiTool = 0.0f;   // N/m-s
	private float kdTool = 100.0f;   // N-s/m

	private float kpMove = 700.0f;  // N/m
	private float kiMove = 30.0f;   // N/m-s
	private float kdMove = 30.0f;   // N-s/m

	private float alpha = 1.5f;
	private float beta = 0.5f;

	private Vector<float> toolPosL;       // current tool position
	private Vector<float> toolVelL;       // current tool velocity
	private Vector<float> toolHoldPosL;  
	private Vector<float> toolForceL;
	private Vector<float> toolPosL_M;     
	private Vector<float> toolVelL_M;    
	private Vector<float> toolForceL_M;

	private Vector<float> toolPosR;      
	private Vector<float> toolVelR; 
	private Vector<float> toolHoldPosR; 
	private Vector<float> toolForceR;
	private Vector<float> toolPosR_M;      
	private Vector<float> toolVelR_M; 
	private Vector<float> toolForceR_M;

	private Vector<float> toolZero;

	private bool forceFieldEnable;

	private BurtSharp.Control.PidScalar toolPidL_S;
	private BurtSharp.Control.PidScalar toolPidR_S;

	private BurtSharp.Control.PidVector toolPidL_Move;
	private BurtSharp.Control.PidVector toolPidR_Move;

	// Position commands should be saved in vectors to be used as inputs to
	// the PID controllers.
	private Vector<float> toolCommandL;
	private Vector<float> toolCommandR;

	// Trajectory generators create a linear trajector with a trapezoidal velocity profile.
	// See the BURT C# library API documentation for more details about the characteristics
	// of this type of trajectory and how it is generated.
	private BurtSharp.Control.LinearTrajectoryVector toolTrajL;
	private BurtSharp.Control.LinearTrajectoryVector toolTrajR;

	private Transform leftHand;
	private Transform rightHand;

	private float x0_L = 0.38f;
	private float y0_L = 0.2f;
	private float x0_R = 0.38f;
	private float y0_R = -0.2f;

	float xOffset = -0.00f; //robot x axis offset right hand centre
	float yOffset = -0.15f; //robot y axis offset right hand centre;

	private float rX = 0.1f;
	private float rY = 0.1f;

	private int logCount = 0;

	private bool logging = false;
	private bool emgRecording = false;
	private List<RobotState> records = new List<RobotState> ();
	Stopwatch timestamper;
	private float elapsedTime;

	private RFT.RFT_Sensor FTSensorL; 
	private RFT.RFT_Sensor FTSensorR; 

	private TriggerEMG.CreateArdunioConnection EMGTrigger; 

	RobotControllerSwitchable robotL;
	RobotControllerSwitchable robotR;

	public int exState = 1;
	public int exStateReal = 1;
	public float experimentTime;

	private Vector<float> rPos;
	private Vector<float> lPos;

	private Text experimentText;
	private Text leftRobotSpeedMonitor;
	private Text rightRobotSpeedMonitor;

	private RobotCommand RunControlCycleLeft ()
	{
		toolPosL.FromVector3 (robotL.ToolPosition);
		toolVelL.FromVector3 (robotL.ToolVelocity);

		toolForceL[0] = 0f;
		toolForceL[1] = 0f;
		toolForceL[2] = toolPidL_S.Update (toolHoldPosL[2], toolPosL[2], toolZero[2], toolVelL[2], robotL.TimeSinceLastCommand);
			
		return new RobotCommand (ControlMode.Force, toolForceL.ToVector3());
	}

	private RobotCommand RunControlCycleRight ()
	{
		toolPosR.FromVector3 (robotR.ToolPosition);
		toolVelR.FromVector3 (robotR.ToolVelocity);

		toolForceR[0] = 0f;
		toolForceR[1] = 0f;
		toolForceR[2] = toolPidR_S.Update (toolHoldPosR[2], toolPosR[2], toolZero[2], toolVelR[2], robotR.TimeSinceLastCommand);

		if (logging) {
			records.Add (new RobotState (
				robotL.TimeSinceLastCommand,
				robotL.ControlCycleDuration,
				robotL.FirmwareTimestamp,
				robotR.TimeSinceLastCommand,
				robotR.ControlCycleDuration,
				robotR.FirmwareTimestamp,
				robotL.ToolPosition,
				robotL.ToolVelocity,
				FTSensorL.latestFTSample,
				robotR.ToolPosition,
				robotR.ToolVelocity,
				FTSensorR.latestFTSample,
				exState,
				exStateReal,
				timestamper.ElapsedMilliseconds / 1000f
			));
		}

		return new RobotCommand (ControlMode.Force, toolForceR.ToVector3());
	}

	private RobotCommand zeroForceController() {
		return new RobotCommand (ControlMode.Zero, toolZero.ToVector3 ());
	}
		
	private RobotCommand RunToolMoveLeft() {
		toolPosL_M.FromVector3 (robotL.ToolPosition);
		toolVelL_M.FromVector3 (robotL.ToolVelocity);
		toolTrajL.Update ();    
		toolForceL_M = toolPidL_Move.Update (toolTrajL.Position, toolPosL_M, toolTrajL.Velocity, toolVelL_M, robotL.TimeSinceLastCommand);
		return new RobotCommand (ControlMode.Force, toolForceL_M.ToVector3 ());
	}

	private RobotCommand RunToolMoveRight() {
		toolPosR_M.FromVector3 (robotR.ToolPosition);
		toolVelR_M.FromVector3 (robotR.ToolVelocity);
		toolTrajR.Update ();    
		toolForceR_M = toolPidR_Move.Update (toolTrajR.Position, toolPosR_M, toolTrajR.Velocity, toolVelR_M, robotR.TimeSinceLastCommand);
		return new RobotCommand (ControlMode.Force, toolForceR_M.ToVector3 ());
	}

	void Awake () {
		robotL = new RobotControllerSwitchable (zeroForceController,false, "192.168.100.201");
		robotR = new RobotControllerSwitchable (zeroForceController, false, "192.168.100.200");

		toolPosL = Vector<float>.Build.Dense (kNumDim);
		toolVelL = Vector<float>.Build.Dense (kNumDim);
		toolHoldPosL = Vector<float>.Build.Dense (kNumDim);
		toolForceL = Vector<float>.Build.Dense (kNumDim);
		toolPosL_M = Vector<float>.Build.Dense (kNumDim);
		toolVelL_M = Vector<float>.Build.Dense (kNumDim);
		toolForceL_M = Vector<float>.Build.Dense (kNumDim);
		toolCommandL = Vector<float>.Build.Dense (kNumDim);

		toolPosR = Vector<float>.Build.Dense (kNumDim);
		toolVelR = Vector<float>.Build.Dense (kNumDim);
		toolHoldPosR = Vector<float>.Build.Dense (kNumDim);
		toolForceR = Vector<float>.Build.Dense (kNumDim);
		toolPosR_M = Vector<float>.Build.Dense (kNumDim);
		toolVelR_M = Vector<float>.Build.Dense (kNumDim);
		toolForceR_M = Vector<float>.Build.Dense (kNumDim);
		toolCommandR = Vector<float>.Build.Dense (kNumDim);

		toolZero = Vector<float>.Build.Dense (kNumDim);

		FTSensorL = new RFT.RFT_Sensor ("ttyUSB0");
		Thread.Sleep (2000);
		FTSensorL.sendStartFTData ();
		Thread.Sleep (2000);
		FTSensorL.sendSetBias (1);
		Thread.Sleep (2000);

		FTSensorR = new RFT.RFT_Sensor ("ttyUSB1");
		Thread.Sleep (2000);
		FTSensorR.sendStartFTData ();
		Thread.Sleep (2000);
		FTSensorR.sendSetBias (1);
		Thread.Sleep (1000);

		EMGTrigger = new TriggerEMG.CreateArdunioConnection("/dev/ttyACM0");
		Thread.Sleep (2000);

		// Set up PID controllers
		toolPidL_S = new BurtSharp.Control.PidScalar (kpTool, kiTool, kdTool);
		toolPidR_S = new BurtSharp.Control.PidScalar (kpTool, kiTool, kdTool);

		toolPidL_Move = new BurtSharp.Control.PidVector (kpMove, kiMove, kdMove, kNumDim);
		toolPidR_Move = new BurtSharp.Control.PidVector (kpMove, kiMove, kdMove, kNumDim);

		// Set up trajectory generators
		toolTrajL = new BurtSharp.Control.LinearTrajectoryVector (kNumDim);
		toolTrajR = new BurtSharp.Control.LinearTrajectoryVector (kNumDim);

		leftHand = GameObject.Find ("leftHand").GetComponent<Transform> ();
		rightHand = GameObject.Find ("rightHand").GetComponent<Transform> ();
		experimentText = GameObject.Find ("experimentCondition").GetComponent<Text> ();
		leftRobotSpeedMonitor = GameObject.Find ("leftRobotSpeedMonitor").GetComponent<Text> ();
		rightRobotSpeedMonitor = GameObject.Find ("rightRobotSpeedMonitor").GetComponent<Text> ();

		timestamper = new Stopwatch ();

		rPos = Vector<float>.Build.Dense (kNumDim);
		lPos = Vector<float>.Build.Dense (kNumDim);
	}
		
	void OnEnable () {}

	void Start () {
		robotL.RegisterControlFunction ("zero", zeroForceController);
		robotL.RegisterControlFunction ("force", RunControlCycleLeft);
		robotL.RegisterControlFunction ("tool", RunToolMoveLeft);

		robotR.RegisterControlFunction ("zero", zeroForceController);
		robotR.RegisterControlFunction ("force", RunControlCycleRight);
		robotR.RegisterControlFunction ("tool", RunToolMoveRight);
	}
		
	void FixedUpdate ()	{

		Vector3 tempL = new Vector3 ();
		Vector3 tempR = new Vector3 ();
		Vector3 tempE = new Vector3 ();
		Vector3 tempZ = new Vector3 ();
		Vector3 tempNZ = new Vector3 ();
		tempZ.x = 0f; tempZ.y = 0f; tempZ.z = 0f;

		leftRobotSpeedMonitor.text = "";
		rightRobotSpeedMonitor.text = "";

		// Condition 1: show feedback for right hand only
		if (exState == 1) {
			exStateReal = 1;

			elapsedTime = timestamper.ElapsedMilliseconds / 1000f;
			if (elapsedTime <= 10 || elapsedTime >= 160) {
				experimentText.text = "STOP";
				exStateReal = 0;
			} else if (elapsedTime > 10 && elapsedTime <= 12) {
				experimentText.text = "GO!";
			} else {
				experimentText.text = "";

				//checking robot speed to make sure it is in a predetermined range
//				float lRSM = Mathf.Sqrt ((robotL.ToolVelocity.x) * (robotL.ToolVelocity.x) + (robotL.ToolVelocity.y) * (robotL.ToolVelocity.y));
//				float rRSM = Mathf.Sqrt ((robotR.ToolVelocity.x) * (robotR.ToolVelocity.x) + (robotR.ToolVelocity.y) * (robotR.ToolVelocity.y));
//
//				if (lRSM <= 0.1f) {
//					leftRobotSpeedMonitor.text = "Faster";
//				} else if ((lRSM >= 0.5f)) {
//					leftRobotSpeedMonitor.text = "Slower";
//				} else {
//					leftRobotSpeedMonitor.text = "";
//				}
//
//				if (rRSM <= 0.1f) {
//					rightRobotSpeedMonitor.text = "Faster";
//				} else if ((rRSM >= 0.5f)) {
//					rightRobotSpeedMonitor.text = "Slower";
//				} else {
//					rightRobotSpeedMonitor.text = "";
//				}
			}

			tempL.x = -robotL.ToolPosition.y;
			tempL.y = robotL.ToolPosition.x; 
			tempL.z = 0.0f;
			leftHand.position = positionScale * tempL;

			tempR.x = -robotR.ToolPosition.y + yOffset;
			tempR.y = robotR.ToolPosition.x + xOffset;
			tempR.z = 0.0f;
			rightHand.position = positionScale * tempR;
		
			// Investigate the effect of feedback for one hand only (actual error vs amplified error)
		} else if (exState == 2) {
			leftHand.position = tempZ; // no feedback for left hand

			elapsedTime = timestamper.ElapsedMilliseconds / 1000f;
			// Switch between error (C2) and amplified error (C3)
			if (elapsedTime <= 40) {
				exStateReal = 1;
				if (elapsedTime <= 10) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 10 && elapsedTime <= 12) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}
					
				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x; 
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;

				// Condition 2: actual error of right hand
			} else if ((elapsedTime > 40 & elapsedTime <= 142)) { 
				exStateReal = 2;
				if (elapsedTime > 40 && elapsedTime <= 42) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 42 && elapsedTime <= 52) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 52 && elapsedTime <= 54) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;

				// Condition 3: amplified error of right hand 
			} else if (elapsedTime > 142 & elapsedTime <= 214) { 
				exStateReal = 3;
				if (elapsedTime > 142 && elapsedTime <= 144) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 144 && elapsedTime <= 154) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 154 && elapsedTime <= 156) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				rPos.FromVector3 (robotR.ToolPosition);
				float thetaR = Mathf.Atan2 (rPos [1] - y0_R, rPos [0] - x0_R);
				float eR = Mathf.Sqrt ((rPos [0] - x0_R) * (rPos [0] - x0_R) + (rPos [1] - y0_R) * (rPos [1] - y0_R)) - rX;

				tempE.x = -(alpha * eR + rX) * Mathf.Sin (thetaR) - y0_R + yOffset; // alpha = 1.5f;
				tempE.y = (alpha * eR + rX) * Mathf.Cos (thetaR) + x0_R; // alpha = 1.5f;
				rightHand.position = positionScale * tempE;

			} else if ((elapsedTime > 214 & elapsedTime <= 256)) { 
				exStateReal = 2;
				if (elapsedTime > 214 && elapsedTime <= 216) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 216 && elapsedTime <= 226) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 226 && elapsedTime <= 228) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;
			} else if (elapsedTime > 226) { 
				experimentText.text = "STOP";
				exStateReal = 0;
			}
		
			// Investigate the effect of composite feedback for one hand only (composite error vs actual error)
		} else if (exState == 3) {
			leftHand.position = tempZ; // no feedback for left hand

			elapsedTime = timestamper.ElapsedMilliseconds / 1000f;
			// Switch between error (C2) and amplified error (C3)
			if (elapsedTime <= 40) {
				exStateReal = 1;
				if (elapsedTime <= 10) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 10 && elapsedTime <= 12) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x; 
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;

				// Condition 2: actual error of right hand
			} else if ((elapsedTime > 40 & elapsedTime <= 142)) { 
				exStateReal = 2;
				if (elapsedTime > 40 && elapsedTime <= 42) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 42 && elapsedTime <= 52) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 52 && elapsedTime <= 54) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;

				// Condition 4: composite error of right hand (right + left shown for right)
			} else if (elapsedTime > 142 & elapsedTime <= 214) { 
				exStateReal = 4;
				if (elapsedTime > 142 && elapsedTime <= 144) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 144 && elapsedTime <= 154) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 154 && elapsedTime <= 156) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				rPos.FromVector3 (robotR.ToolPosition);
				float thetaR = Mathf.Atan2 (rPos [1] - y0_R, rPos [0] - x0_R);
				float eR = Mathf.Sqrt ((rPos [0] - x0_R) * (rPos [0] - x0_R) + (rPos [1] - y0_R) * (rPos [1] - y0_R)) - rX;

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;

				tempR.x = -(eR + rX) * Mathf.Sin (thetaR);
				tempR.y = (eR + rX) * Mathf.Cos (thetaR);
				rightHand.position = positionScale * tempR;

				lPos.FromVector3 (robotL.ToolPosition);
				float thetaL = Mathf.Atan2 (lPos [1] - y0_L, lPos [0] - x0_L);
				float eL = Mathf.Sqrt ((lPos [0] - x0_L) * (lPos [0] - x0_L) + (lPos [1] - y0_L) * (lPos [1] - y0_L)) - rX;

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x; 
				tempL.z = 0.0f;

				tempL.x = -(eL + rX) * Mathf.Sin (thetaL);
				tempL.y = (eL + rX) * Mathf.Cos (thetaL);

				float thetaE = Mathf.Atan2 ((tempL.y + tempR.y) / 2f, (tempL.x + tempR.x) / 2f);
				tempE.x = (1f * eR + beta * eL + rX) * Mathf.Cos (thetaE) - y0_R + yOffset; // alpha = 1.5f &  beta = 0.5f;
				tempE.y = (1f * eR + beta * eL + rX) * Mathf.Sin (thetaE) + x0_R;
				rightHand.position = positionScale * tempE;

			} else if ((elapsedTime > 214 & elapsedTime <= 256)) { 
				exStateReal = 2;
				if (elapsedTime > 214 && elapsedTime <= 216) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 216 && elapsedTime <= 226) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 226 && elapsedTime <= 228) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;
			} else if (elapsedTime > 226) { 
				experimentText.text = "STOP";
				exStateReal = 0;
			}
				
		} else if (exState == 4) {
			rightHand.position = tempZ; // no feedback for right hand

			elapsedTime = timestamper.ElapsedMilliseconds / 1000f;
			// Switch between error (C2) and amplified error (C3)
			if (elapsedTime <= 40) {
				exStateReal = 1;
				if (elapsedTime <= 10) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 10 && elapsedTime <= 12) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x; 
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;

				// Condition 2: actual error of right hand
			} else if ((elapsedTime > 40 & elapsedTime <= 142)) { 
				exStateReal = 5;
				if (elapsedTime > 40 && elapsedTime <= 42) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 42 && elapsedTime <= 52) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 52 && elapsedTime <= 54) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x;
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

				// Condition 3: amplified error of right hand 
			} else if (elapsedTime > 142 & elapsedTime <= 214) { 
				exStateReal = 6;
				if (elapsedTime > 142 && elapsedTime <= 144) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 144 && elapsedTime <= 154) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 154 && elapsedTime <= 156) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				lPos.FromVector3 (robotL.ToolPosition);
				float thetaL = Mathf.Atan2 (lPos [1] - y0_L, lPos [0] - x0_L);
				float eL = Mathf.Sqrt ((lPos [0] - x0_L) * (lPos [0] - x0_L) + (lPos [1] - y0_L) * (lPos [1] - y0_L)) - rX;

				tempE.x = -(alpha * eL + rX) * Mathf.Sin (thetaL) - y0_L; // alpha = 1.5f
				tempE.y = (alpha * eL + rX) * Mathf.Cos (thetaL) + x0_L;
				leftHand.position = positionScale * tempE;

			} else if ((elapsedTime > 214 & elapsedTime <= 256)) { 
				exStateReal = 5;
				if (elapsedTime > 214 && elapsedTime <= 216) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 216 && elapsedTime <= 226) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 226 && elapsedTime <= 228) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x;
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

			} else if (elapsedTime > 226) { 
				experimentText.text = "STOP";
				exStateReal = 0;
			}

		} else if (exState == 5) {
			rightHand.position = tempZ; // no feedback for right hand

			elapsedTime = timestamper.ElapsedMilliseconds / 1000f;
			// Switch between error (C2) and amplified error (C3)
			if (elapsedTime <= 40) {
				exStateReal = 1;
				if (elapsedTime <= 10) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 10 && elapsedTime <= 12) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x; 
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;
				rightHand.position = positionScale * tempR;

				// Condition 2: actual error of right hand
			} else if ((elapsedTime > 40 & elapsedTime <= 142)) { 
				exStateReal = 5;
				if (elapsedTime > 40 && elapsedTime <= 42) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 42 && elapsedTime <= 52) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 52 && elapsedTime <= 54) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x;
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

				// Condition 3: amplified error of right hand 
			} else if (elapsedTime > 142 & elapsedTime <= 214) { 
				exStateReal = 7;
				if (elapsedTime > 142 && elapsedTime <= 144) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 144 && elapsedTime <= 154) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 154 && elapsedTime <= 156) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				rPos.FromVector3 (robotR.ToolPosition);
				float thetaR = Mathf.Atan2 (rPos [1] - y0_R, rPos [0] - x0_R);
				float eR = Mathf.Sqrt ((rPos [0] - x0_R) * (rPos [0] - x0_R) + (rPos [1] - y0_R) * (rPos [1] - y0_R)) - rX;

				tempR.x = -robotR.ToolPosition.y + yOffset;
				tempR.y = robotR.ToolPosition.x + xOffset;
				tempR.z = 0.0f;

				tempR.x = -(eR + rX) * Mathf.Sin (thetaR);
				tempR.y = (eR + rX) * Mathf.Cos (thetaR);
				leftHand.position = positionScale * tempR;

				lPos.FromVector3 (robotL.ToolPosition);
				float thetaL = Mathf.Atan2 (lPos [1] - y0_L, lPos [0] - x0_L);
				float eL = Mathf.Sqrt ((lPos [0] - x0_L) * (lPos [0] - x0_L) + (lPos [1] - y0_L) * (lPos [1] - y0_L)) - rX;

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x; 
				tempL.z = 0.0f;

				tempL.x = -(eL + rX) * Mathf.Sin (thetaL);
				tempL.y = (eL + rX) * Mathf.Cos (thetaL);

				float thetaE = Mathf.Atan2 ((tempL.y + tempR.y) / 2f, (tempL.x + tempR.x) / 2f);
				tempE.x = (beta * eR + 1f * eL + rX) * Mathf.Cos (thetaE) - y0_L; // alpha = 1.5f &  beta = 0.5f;
				tempE.y = (beta * eR + 1f * eL + rX) * Mathf.Sin (thetaE) + x0_L;
				leftHand.position = positionScale * tempE;

			} else if ((elapsedTime > 214 & elapsedTime <= 256)) { 
				exStateReal = 5;
				if (elapsedTime > 214 && elapsedTime <= 216) {
					experimentText.text = "STOP";
					exStateReal = 0;
				} else if (elapsedTime > 216 && elapsedTime <= 226) {
					experimentText.text = "GOING TO HOME POSITION - BE READY ...";
					exStateReal = 0;
				} else if (elapsedTime > 226 && elapsedTime <= 228) {
					experimentText.text = "GO!";
				} else {
					experimentText.text = "";
				}

				tempL.x = -robotL.ToolPosition.y;
				tempL.y = robotL.ToolPosition.x;
				tempL.z = 0.0f;
				leftHand.position = positionScale * tempL;

			} else if (elapsedTime > 226) { 
				experimentText.text = "STOP";
				exStateReal = 0;
			}
		}

	}
		

	void OnDisable () {}

	void OnGUI () {
		Event e = Event.current;
		string keyPressed = e.keyCode.ToString();
		if (e.type == EventType.keyUp) {
			if (keyPressed.ToLower () == "p") {

				toolPosL.FromVector3 (robotL.ToolPosition);
				UnityEngine.Debug.Log("left tool (robot)  " + toolPosL[0] + " , " + toolPosL[1] + " , " + toolPosL[2]);
//				Vector3 lhp = new Vector3 ();
//				lhp = leftHand.position;// / positionScale;
//				UnityEngine.Debug.Log("left tool (unity)  " + lhp[0] + " , " + lhp[1] + " , " + lhp[2]);
//
				toolPosR.FromVector3 (robotR.ToolPosition);
				UnityEngine.Debug.Log("right tool (robot)  " + toolPosR[0] + " , " + toolPosR[1] + " , " + toolPosR[2]);
//				Vector3 rhp = new Vector3 ();
//				rhp = rightHand.position;// / positionScale;
//				UnityEngine.Debug.Log("right tool (unity)  " + rhp[0] + " , " + rhp[1] + " , " + rhp[2]);


				UnityEngine.Debug.Log("L force sensor  " + FTSensorL.latestFTSample.serialize());
				UnityEngine.Debug.Log("R force sensor  " + FTSensorR.latestFTSample.serialize());

			} else if (keyPressed.ToLower () == "e") { // enable robot
				disableForce ();
				robotL.Enable ();
				robotR.Enable ();
			} else if (keyPressed.ToLower () == "d") { // disable robot
				disableForce ();
				robotL.Disable ();
				robotR.Disable ();

				FTSensorL.closePort ();
				FTSensorR.closePort ();

				if(logging) disableLogging();
				if(emgRecording) sendEMGStop();

				EMGTrigger.disconnectFromArduino ();
			} else if (keyPressed.ToLower () == "z") { // disable 2D force field
				disableForce ();
			} else if (keyPressed.ToLower () == "c") { // enable 2D force field
				enableForce ();
			} else if (keyPressed.ToLower () == "f") {
				toggleForceField ();
			} else if (keyPressed.ToLower () == "l") { // start/stop logging data
				if (logging) {
					disableLogging ();
				} else {
					enableLogging ();
				}
				if (emgRecording) {
					sendEMGStop ();
				} else {
					sendEMGStart ();
				}
			} else if (keyPressed.ToLower () == "h") {
				//if(logging) disableLogging();
				//if(emgRecording) sendEMGStop();

				MoveLeftTool ();
				MoveRightTool ();
			} else if (keyPressed.ToLower () == "alpha1") { // 1-5 for different experiment phases
				timestamper.Reset ();
				timestamper.Start ();

				if(logging) disableLogging();
				enableLogging();

				if(emgRecording) sendEMGStop();
				sendEMGStart ();

				exState = 1;
				UnityEngine.Debug.Log ("State 1\n");
			} else if (keyPressed.ToLower () == "alpha2") {
				timestamper.Reset ();
				timestamper.Start ();

				if(logging) disableLogging();
				enableLogging();

				if(emgRecording) sendEMGStop();
				sendEMGStart ();

				exState = 2;
				UnityEngine.Debug.Log ("State 2\n");
			} else if (keyPressed.ToLower () == "alpha3") {
				timestamper.Reset ();
				timestamper.Start ();

				if(logging) disableLogging();
				enableLogging();

				if(emgRecording) sendEMGStop();
				sendEMGStart ();

				exState = 3;
				UnityEngine.Debug.Log ("State 3\n");
			} else if (keyPressed.ToLower () == "alpha4") {
				timestamper.Reset ();
				timestamper.Start ();

				if(logging) disableLogging();
				enableLogging();

				if(emgRecording) sendEMGStop();
				sendEMGStart ();

				exState = 4;
				UnityEngine.	Debug.Log ("State 4\n");
			} else if (keyPressed.ToLower () == "alpha5") {
				timestamper.Reset ();
				timestamper.Start ();

				if(logging) disableLogging();
				enableLogging();

				if(emgRecording) sendEMGStop();
				sendEMGStart ();

				exState = 5;
				UnityEngine.Debug.Log ("State 5\n");
			}  
		}
	}

	/// <summary>
	/// Raises the application quit event. This is called when you quit the game.
	/// </summary>
	void OnApplicationQuit() {
		robotL.Disable ();
		robotR.Disable ();
	}

	void ffON () {
		forceFieldEnable = true;
		UnityEngine.Debug.Log ("FF ON");
	}

	void ffOFF () {
		forceFieldEnable = false;
		UnityEngine.Debug.Log ("FF OFF");
	}

	void toggleForceField () {
		if (forceFieldEnable) {
			ffOFF ();
		} else {
			ffON ();
		}
	}

	void disableForce() {
		robotL.SetActiveControlFunction ("zero", 1f);	
		robotR.SetActiveControlFunction ("zero", 1f);
		ffOFF ();
		UnityEngine.Debug.Log ("ZERO \n");
	}

	void enableForce() {
		toolHoldPosL.FromVector3 (robotL.ToolPosition);
		toolHoldPosR.FromVector3 (robotR.ToolPosition);

		// ensuring that the robot height doesnt decrease when switching between force filed and home position
		if (toolHoldPosL [2] <= 0.32f & toolHoldPosL [2] >= 0.19f &
			toolHoldPosR [2] <= 0.32f & toolHoldPosR [2] >= 0.19f) // these limits are found by try-and-error
		{
			toolHoldPosL [2] = 0.275f; // this is some fixed value which makes robot arm parallel to the ground
			toolHoldPosR [2] = 0.275f;
		}


		// Left tool home position
		toolCommandL [0] = x0_L + rX;
		toolCommandL [1] = y0_L;
		toolCommandL [2] = (toolHoldPosL [2] + toolHoldPosR[2])/2f;

		// Right tool home position
		toolCommandR [0] = x0_R + rX;
		toolCommandR [1] = y0_R;
		toolCommandR [2] = (toolHoldPosL [2] + toolHoldPosR[2])/2f;


		robotL.SetActiveControlFunction ("force", 1f);	
		robotR.SetActiveControlFunction ("force", 1f);	
	}

	void disableLogging() {
		logging = false;
		logCount++;
		TextWriter writer = File.CreateText(@"./log_" + logCount.ToString() + ".csv");
		CsvWriter csv = new CsvWriter (writer);
		csv.WriteRecords (records);
		records = new List<RobotState> ();
		UnityEngine.Debug.Log ("Logging OFF, writing file\n");
	}

	void enableLogging() {
		logging = true;
		UnityEngine.Debug.Log ("Logging ON \n");
	}

	void sendEMGStart() {
		emgRecording = true;
		EMGTrigger.writeToArduino("S");
		UnityEngine.Debug.Log ("EMG Started \n");
	}

	void sendEMGStop() {
		emgRecording = true;
		EMGTrigger.writeToArduino("E");
		UnityEngine.Debug.Log ("EMG Stopped \n");
	}

	void displayPositions() {
		Vector<float> lPos = Vector<float>.Build.Dense (kDof);
		lPos.FromVector3 (robotL.ToolPosition);
		UnityEngine.Debug.Log ("Left Pos: " + lPos [0] + ", " + lPos [1] + ", " + lPos [2] + "\n");
	}

	public class RobotState
	{
		public float timestampSW1_L { get; set; }
		public float timestampSW2_L { get; set; }
		public float timestampFW_L { get; set; }
		public float timestampSW1_R { get; set; }
		public float timestampSW2_R { get; set; }
		public float timestampFW_R { get; set; }
		public float toolPosX_L { get; set; }
		public float toolPosY_L { get; set; }
		public float toolPosZ_L { get; set; }
		public float toolVelX_L { get; set; }
		public float toolVelY_L { get; set; }
		public float toolVelZ_L { get; set; }
		public float RFT_fx_L { get; set; }
		public float RFT_fy_L { get; set; }
		public float RFT_fz_L { get; set; }
		public float RFT_tx_L { get; set; }
		public float RFT_ty_L { get; set; }
		public float RFT_tz_L { get; set; }
		public float toolPosX_R { get; set; }
		public float toolPosY_R { get; set; }
		public float toolPosZ_R { get; set; }
		public float toolVelX_R { get; set; }
		public float toolVelY_R { get; set; }
		public float toolVelZ_R { get; set; }
		public float RFT_fx_R { get; set; }
		public float RFT_fy_R { get; set; }
		public float RFT_fz_R { get; set; }
		public float RFT_tx_R { get; set; }
		public float RFT_ty_R { get; set; }
		public float RFT_tz_R { get; set; }
		public int experimentState { get; set; }
		public int experimentState2 { get; set; }
		public float experimentTime { get; set; }

		public RobotState (
			float timestampSW1_L_,
			float timestampSW2_L_,
			float timestampFW_L_,
			float timestampSW1_R_,
			float timestampSW2_R_,
			float timestampFW_R_,
			Vector3 toolPos_L,
			Vector3 toolVel_L,
			RFT.ForceTorqueSample ftSamp_L,
			Vector3 toolPos_R,
			Vector3 toolVel_R,
			RFT.ForceTorqueSample ftSamp_R,
			int state_,
			int stateReal_,
			float expTime_
		)
		{
			timestampSW1_L = timestampSW1_L_;
			timestampSW2_L = timestampSW2_L_;
			timestampFW_L = timestampFW_L_;
			timestampSW1_R = timestampSW1_R_;
			timestampSW2_R = timestampSW2_R_;
			timestampFW_R = timestampFW_R_;
			toolPosX_L = toolPos_L.x;
			toolPosY_L = toolPos_L.y;
			toolPosZ_L = toolPos_L.z;
			toolVelX_L = toolVel_L.x;
			toolVelY_L = toolVel_L.y;
			toolVelZ_L = toolVel_L.z;
			RFT_fx_L = ftSamp_L.Fx;
			RFT_fy_L = ftSamp_L.Fy;
			RFT_fz_L = ftSamp_L.Fz;
			RFT_tx_L = ftSamp_L.Tx;
			RFT_ty_L = ftSamp_L.Ty;
			RFT_tz_L = ftSamp_L.Tz;
			toolPosX_R = toolPos_R.x;
			toolPosY_R = toolPos_R.y;
			toolPosZ_R = toolPos_R.z;
			toolVelX_R = toolVel_R.x;
			toolVelY_R = toolVel_R.y;
			toolVelZ_R = toolVel_R.z;
			RFT_fx_R = ftSamp_R.Fx;
			RFT_fy_R = ftSamp_R.Fy;
			RFT_fz_R = ftSamp_R.Fz;
			RFT_tx_R = ftSamp_R.Tx;
			RFT_ty_R = ftSamp_R.Ty;
			RFT_tz_R = ftSamp_R.Tz;
			experimentState = state_;
			experimentState2 = stateReal_;
			experimentTime = expTime_;
		}
	}

	public void MoveLeftTool ()
	{
		if (robotL.GetActiveControlFunctionName ().Equals ("force")) {
			//UnityEngine.Debug.Log("Moving left tool to (" + toolCommandL.ToVector3 ().ToString ("f3") + ")");
				toolPosL_M.FromVector3 (robotL.ToolPosition);
				toolVelL_M.FromVector3 (robotL.ToolVelocity);
				toolTrajL.BeginMove (toolPosL_M, toolCommandL, 0.3f, 0.3f);
				toolForceL_M.Clear ();
				toolPidL_Move.ResetAll ();
				robotL.SetActiveControlFunction ("tool", 1f);
		} else {
			UnityEngine.Debug.Log("Left tool: active the force field to enable home positioning.\n");
		}
	}

	public void MoveRightTool ()
	{
		if (robotR.GetActiveControlFunctionName ().Equals ("force")) {
			//UnityEngine.Debug.Log("Moving right tool to (" + toolCommandR.ToVector3 ().ToString ("f3") + ")");
			toolPosR_M.FromVector3 (robotR.ToolPosition);
			toolVelR_M.FromVector3 (robotR.ToolVelocity);
			toolTrajR.BeginMove (toolPosR_M, toolCommandR, 0.3f, 0.3f);
			toolForceR_M.Clear ();
			toolPidR_Move.ResetAll ();
			robotR.SetActiveControlFunction ("tool", 1f);
		} else {
			UnityEngine.Debug.Log("Right tool: active the force field to enable home positioning.\n");
		}
	}


}
