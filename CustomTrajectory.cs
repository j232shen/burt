using BurtSharp.CoAP;
using BurtSharp.CoAP.MsgTypes;
using BurtSharp.Control;
using BurtSharp.Util.ClassExtensions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Statistics;
using System;
using System.Diagnostics;
using System.Threading;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using CsvHelper;
using CsvHelper.Configuration;

namespace BurtSharp.Examples
{
    /// <summary>
    /// Uses a PID controller to follow a custom trajectory in joint space space. In this example, the
    /// custom trajectory is a circle. In joint space, the circle is in joints 2 and 3.
    /// </summary>
    public class CustomTrajectory
    {
        private RobotControllerSwitchable robot;
        private BurtSharp.KeyboardManager keyboardManager;

        const int kDof = 3;                  // number of degrees of freedom
        const int kNumDim = 3;               // number of Cartesian dimensions


        private Vector<float> jointPos;      // current joint positions
        private Vector<float> jointVel;      // current joint velocities
        private Vector<float> jointCommand;  // commanded joint positions
        private Vector<float> jointTorque;   // commanded joint torques
		//private Vector<float> jointHoldPos;  // joint hold position command
        private Vector<float> zeroJoint;
		private Vector<float> jointVelCommand;

        // Characteristics of the joint space circle. Separate start positions are needed for
        // right- and left-handed workspaces.
        readonly float[] startPosJointLeft = new float[] { 0.0f, 0.0f, -1.57f };
        readonly float[] startPosJointRight = new float[] { -0.15f, 0.45f, 2.00f };
        private Vector<float> startPosJoint; 
		private float[] TargetRotation;// = new float[100]; // degrees --> radians
		private Vector<float> TargetAngle;
		const float TimetoWait = 2.0f;
		const float TimetoMove = 3.0f;
		private int State = 0;
		//		private int BeepFreq = 800; // used for console.beep()
		//		private int BeepDuration = 200;
		private float a0;
		private float a2;
		private float a3;
		private float velMax1 = 0.6f;
		private float velMax2 = 0.5f;
		private float a02;
		private float a22;
		private float a32;
		const float AngleOffsetC = 0.004f;
		const float AngleOffsetT = 0.009f;

		// Safety Constants
		const float TorqueMax0 = 30f;
		const float TorqueMax1 = 30f;
		const float TorqueMax2 = 30f;
		const float VelMax0 = 2.5f;
		const float VelMax1 = 2.5f;
		const float VelMax2 = 2.5f;

        private BurtSharp.Control.PidVector jointPid;
        public static readonly float[] kpJointDefault = { 300, 100, 50 };  // N-m/rad 
		public static readonly float[] kiJointDefault = {  15, 50,  10 };  // N-m/rad-s
        public static readonly float[] kdJointDefault = {  15, 30,  10 };  // N-m-s/rad
        private Vector<float> kpJoint;
        private Vector<float> kiJoint;
        private Vector<float> kdJoint;

        // A linear trajectory for moving to the start point of the circle
        private BurtSharp.Control.LinearTrajectoryVector startTrajJoint;

        // See Example02-HoldPosition for an explanation of the timers used in this example.
        // The circleTimer is added for generating the circular movement. This timer keeps
        // track of how long the circular trajectory has been running. The displayTimer
        // is added to update the data display at a rate slower than the control loop.
        private Stopwatch circleTimer = new Stopwatch ();
		private Stopwatch IdleTimer = new Stopwatch ();
		private Stopwatch Timer = new Stopwatch ();

		private List<RobotState> records = new List<RobotState> ();
		private bool logging = false;
		private bool emgRecording = false;
		private bool DoneExperiment = false;
		private int MoveBack1 = 1; // 1: forward motion as expected, 2: forward motion in oposite direction
		private int MoveBack2 = 1;

		private TriggerEMG.CreateArdunioConnection EMGTrigger; 

		int readingCounter = 0;
		int iterationCounter = 0;
		string FileName = "1N";

        public CustomTrajectory ()
        {
            // Initialize vectors
            jointPos = Vector<float>.Build.Dense (kDof);
            jointVel = Vector<float>.Build.Dense (kDof);
            jointTorque = Vector<float>.Build.Dense (kDof);
            jointCommand = Vector<float>.Build.Dense (kDof);
            zeroJoint = Vector<float>.Build.Dense (kNumDim);
			jointVelCommand = Vector<float>.Build.Dense (kNumDim);
            kpJoint = Vector<float>.Build.DenseOfArray (kpJointDefault);
            kiJoint = Vector<float>.Build.DenseOfArray (kiJointDefault);
            kdJoint = Vector<float>.Build.DenseOfArray (kdJointDefault);

			TargetAngle = Vector<float>.Build.Dense (3);

            // Set up communication with the robot
            robot = new RobotControllerSwitchable ();
			robot.RegisterControlFunction ("joint", RunJointPid);

            // Set up keyboard callbacks
            keyboardManager = new BurtSharp.KeyboardManager ();
            keyboardManager.SetDebug (true);  // print key pressed
            keyboardManager.SetQuitKey ("q");
            keyboardManager.AddKeyPressCallback ("e", OnEnable);
			keyboardManager.AddKeyPressCallback ("d", OnDisable);
			keyboardManager.AddKeyPressCallback ("j", StartExperiment);
            keyboardManager.AddKeyPressCallback ("i", Idle);
            keyboardManager.AddKeyPressCallback ("?", PrintUsage);
			keyboardManager.AddKeyPressCallback ("t", PrintCurrentLocation);
			keyboardManager.AddKeyPressCallback ("r", ReadData);
			keyboardManager.AddKeyPressCallback ("o", EMGStartStop);
            PrintUsage ();

			EMGTrigger = new TriggerEMG.CreateArdunioConnection("/dev/ttyACM0");
			Thread.Sleep (2000);

            // Set up PID controllers
            jointPid = new BurtSharp.Control.PidVector (kpJoint, kiJoint, kdJoint, kDof);

            // Set up trajectory generators
            startTrajJoint = new BurtSharp.Control.LinearTrajectoryVector (kDof);

            bool isRunning = true;
            while (isRunning) {
                isRunning = keyboardManager.ReadKeyPress ();
                Thread.Sleep (50);
            }

            Console.Write ("Quitting.");
            robot.Dispose ();
            Environment.Exit (0);
        }

        public static void Main (string[] args)
        {
            new CustomTrajectory ();
        }
			
        /// Updates the trajectory and runs the joint PID controller.
		public RobotCommand RunJointPid ()
        {
            jointPos.FromVector3 (robot.JointPositions);
            jointVel.FromVector3 (robot.JointVelocities);

			// Target angle for active forward motion
			 if (TargetRotation[0] == 0 && !DoneExperiment) {
				startTrajJoint.BeginMove (jointPos, startPosJoint);
				State = 10; // All movements performed, go back to home and stop
			}

			if (State == 0) { // Move robot to home position before begining of the experiment
				startTrajJoint.Update ();
				startTrajJoint.Position.CopyTo (jointCommand);
				zeroJoint.CopyTo (jointVelCommand);
				if (startTrajJoint.DoneMoving) {
					State = 1;
				}
					
			} else if (State == 1) { // Wait at home position for 5 seconds before the experiment starts
				if (!IdleTimer.IsRunning) {
					IdleTimer.Start ();
					System.Media.SystemSounds.Exclamation.Play();
				} 
				float time1 = (float)(IdleTimer.ElapsedMilliseconds) / 1000f;
				if (time1 <= 5.0f) {
					startPosJoint.CopyTo (jointCommand);
					zeroJoint.CopyTo (jointVelCommand);
				} else {
					IdleTimer.Reset ();
					circleTimer.Reset ();
					jointPos.CopyTo (startPosJoint);

					// Determine the first state: forward active or passive
					if (TargetRotation [0] > 0.0f) {
						State = 2;
						TargetAngle[1] = startPosJoint [1] - TargetRotation [0] * (Mathf.PI / 180.0f); 
					} else {
						State = 4;
					}
				}
					
			} else if (State == 2) { // Moving Forward Active (move toward target)
				if (!circleTimer.IsRunning) {
					a0 = startPosJoint [1];
					a2 = (4f*(float)Math.Pow(-velMax1,2))/(3f*(TargetAngle [1] - startPosJoint [1]));
					a3 = -(16f*(float)Math.Pow(-velMax1,3))/(27f*(float)Math.Pow(TargetAngle [1] - startPosJoint [1],2));
//					Console.Beep (BeepFreq, BeepDuration);

					circleTimer.Start (); // Start the movement timer

					Console.WriteLine ("Moving Forward Active, Rotation = [{0}]", TargetRotation [0].ToString ());
					// Play a beep sound: start of the motion
					System.Media.SystemSounds.Exclamation.Play();
				}

				if (jointCommand [1] > TargetAngle [1] + AngleOffsetC) { 
					float time2 = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
					// position planning: 3rd order polynomial
					jointCommand [0] = startPosJoint [0];
					jointCommand [1] =  a0 + a2 * (float)Math.Pow(time2,2) + a3 * (float)Math.Pow(time2,3);
					jointCommand [2] = startPosJoint [2];
					// velocity planning
					jointVelCommand [0] = 0;
					jointVelCommand [1] =  2f * a2 * time2 + 3f* a3 * (float)Math.Pow(time2,2);
					jointVelCommand [2] = 0;

				}	else if(jointPos [1] < TargetAngle [1] + AngleOffsetT) {
					circleTimer.Reset ();
					IdleTimer.Reset ();

					jointPos.CopyTo (TargetAngle);
					jointCommand [0] = startPosJoint [0];
					jointCommand [1] = TargetAngle [1];
					jointCommand [2] = TargetAngle [2]; 
					zeroJoint.CopyTo (jointVelCommand);

					State = 5;
				}
					
			} else if (State == 3) { // Moving Backward Active
				if (!circleTimer.IsRunning) {
					circleTimer.Start (); // Start the movement timer

					if (jointPos [1] <= startPosJoint [1]) {
						MoveBack1 = 1;
						a0 = jointPos [1];
						a2 = (4f*(float)Math.Pow(velMax1,2))/(3f*(startPosJoint [1] - jointPos [1]));
						a3 = -(16f*(float)Math.Pow(velMax1,3))/(27f*(float)Math.Pow(startPosJoint [1] - jointPos [1],2));
					} else {
						MoveBack1 = 2;
						a0 = jointPos [1];
						a2 = (4f*(float)Math.Pow(-velMax1,2))/(3f*(startPosJoint [1] - jointPos [1]));
						a3 = -(16f*(float)Math.Pow(-velMax1,3))/(27f*(float)Math.Pow(startPosJoint [1] - jointPos [1],2));
					}

					if (jointPos [2] >= startPosJoint [2]) {
						MoveBack2 = 1;
						a02 = jointPos [2];
						a22 = (4f*(float)Math.Pow(-velMax2,2))/(3f*(startPosJoint [2] - jointPos [2]));
						a32 = -(16f*(float)Math.Pow(-velMax2,3))/(27f*(float)Math.Pow(startPosJoint [2] - jointPos [2],2));
					} else {
						MoveBack2 = 2;
						a02 = jointPos [2];
						a22 = (4f*(float)Math.Pow(velMax2,2))/(3f*(startPosJoint [2] - jointPos [2]));
						a32 = -(16f*(float)Math.Pow(velMax2,3))/(27f*(float)Math.Pow(startPosJoint [2] - jointPos [2],2));
					}
				} 

				jointCommand [0] = startPosJoint [0];
				jointVelCommand [0] = 0;

				if (MoveBack1 == 1) {
					if (jointCommand [1] <= startPosJoint [1] - AngleOffsetC){ 
						float time2 = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
						jointCommand [1] =  a0 + a2 * (float)Math.Pow(time2,2) + a3 * (float)Math.Pow(time2,3);
						jointVelCommand [1] =  2f * a2 * time2 + 3f * a3 * (float)Math.Pow(time2,2);
					} 
				} else if (MoveBack1 == 2) {
					if (jointCommand [1] >= startPosJoint [1] + AngleOffsetC){  
						float time2 = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
						jointCommand [1] =  a0 + a2 * (float)Math.Pow(time2,2) + a3 * (float)Math.Pow(time2,3);
						jointVelCommand [1] =  2f * a2 * time2 + 3f * a3 * (float)Math.Pow(time2,2);
					} 
				}

				if (MoveBack2 == 1) {
					if (jointCommand [2] >= startPosJoint [2] + AngleOffsetC){  
						float time2 = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
						jointCommand [2] =  a02 + a22 * (float)Math.Pow(time2,2) + a32 * (float)Math.Pow(time2,3);
						jointVelCommand [2] =  2f * a22 * time2 + 3f * a32 * (float)Math.Pow(time2,2);
					} 
				} else if (MoveBack2 == 2) {
					if (jointCommand [2] <= startPosJoint [2] - AngleOffsetC){  
						float time2 = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
						jointCommand [2] =  a02 + a22 * (float)Math.Pow(time2,2) + a32 * (float)Math.Pow(time2,3);
						jointVelCommand [2] =  2f * a22 * time2 + 3f * a32 * (float)Math.Pow(time2,2);
					} 
				}

				if ((jointPos [1] > startPosJoint [1] - AngleOffsetT && jointPos [2] < startPosJoint [2] + AngleOffsetT && MoveBack1 == 1 && MoveBack2 == 1) 
					|| 
					(jointPos [1] < startPosJoint [1] + AngleOffsetT && jointPos [2] > startPosJoint [2] - AngleOffsetT && MoveBack1 == 2 && MoveBack2 == 2)
					|| 
					(jointPos [1] > startPosJoint [1] - AngleOffsetT && jointPos [2] > startPosJoint [2] - AngleOffsetT && MoveBack1 == 1 && MoveBack2 == 2)
					|| 
					(jointPos [1] < startPosJoint [1] + AngleOffsetT && jointPos [2] < startPosJoint [2] + AngleOffsetT && MoveBack1 == 2 && MoveBack2 == 1)
					) {
					circleTimer.Reset ();
					IdleTimer.Reset ();
					State = 6;
				}

			} else if (State == 4) { // Moving Forward Passive
				if (!circleTimer.IsRunning) {
					circleTimer.Start (); // Start the movement timer
					Console.WriteLine ("**Moving Forward Passive, Rotation = [{0}]", TargetRotation [0].ToString ());
					System.Media.SystemSounds.Exclamation.Play();
				}
				float time2 = (float)(circleTimer.ElapsedMilliseconds) / 1000f;
				if (time2 <= TimetoMove) { 
					jointPos.CopyTo (TargetAngle);
					jointCommand [0] = startPosJoint [0];
					jointCommand [1] = TargetAngle [1];
					jointCommand [2] = TargetAngle [2]; 
					zeroJoint.CopyTo (jointVelCommand);
				} else {
					circleTimer.Reset ();
					IdleTimer.Reset ();
					jointPos.CopyTo (TargetAngle);
					jointCommand [0] = startPosJoint [0];
					jointCommand [1] = TargetAngle [1];
					jointCommand [2] = TargetAngle [2]; 
					zeroJoint.CopyTo (jointVelCommand);
					State = 5;
				}
			
			} else if (State == 5) { // Stay at Target for "TimetoWait" seconds
				if (!IdleTimer.IsRunning) {
					IdleTimer.Start ();
					Console.WriteLine ("Stay at Target for [{0}] seconds.", TimetoWait.ToString ());
				} 
				float time1 = (float)(IdleTimer.ElapsedMilliseconds) / 1000f;
				if (time1 > TimetoWait) { // go back to home position active (Moving Backward Active) otherwise stay at target
					State = 3;
					circleTimer.Reset ();
					IdleTimer.Reset ();
				}

			} else if (State == 6) {// Stay at Home for "TimetoWait" seconds
				if (!IdleTimer.IsRunning) {
					IdleTimer.Start ();
					Console.WriteLine ("Stay at Home for [{0}] seconds.", TimetoWait.ToString ());
				} 
				float time1 = (float)(IdleTimer.ElapsedMilliseconds) / 1000f;
				if (time1 <= TimetoWait) {
					startPosJoint.CopyTo (jointCommand);
					zeroJoint.CopyTo (jointVelCommand);
				} else { 
					// Remove the previously executed motion from the list of Target Positions
					TargetRotation = TargetRotation.Skip (1).ToArray ();

					iterationCounter++;
					Console.WriteLine ("Target number [{0}] out of [{1}].", iterationCounter.ToString (), readingCounter.ToString ());

					// Determine the next state based on forward active or passive
					if (TargetRotation [0] > 0.0f) {
						State = 2;
						TargetAngle[1] = startPosJoint [1] - TargetRotation [0] * (Mathf.PI / 180.0f); 
					} else {
						State = 4;
					}
					circleTimer.Reset ();
					IdleTimer.Reset ();
				}


			} else if (State == 10) {
				startTrajJoint.Position.CopyTo (jointCommand);
				startTrajJoint.Update ();
				DoneExperiment = true;

				if (startTrajJoint.DoneMoving) {
					startPosJoint.CopyTo (jointCommand);

					// Save Data
					if (logging){
						if(emgRecording) sendEMGStop();

						Console.WriteLine ("Experiment Done! Saving Data.");
						Timer.Reset ();
						logging = false;
						TextWriter writer = File.CreateText(@"../../log_" + FileName + ".csv");
						CsvWriter csv = new CsvWriter (writer);
						csv.WriteRecords (records);	
					}
				}

			} else {
				robot.SetActiveControlFunction ("zero", 1f);
			}


			// Calculate Robot Torque for the Current State
			if (State == 2) { // moving forward active
				jointTorque = jointPid.Update (jointCommand, jointPos, jointVelCommand, jointVel, robot.TimeSinceLastCommand);
				jointTorque [2] = 0.0f;
			} else if (State == 4) { // moving forward passive
				jointTorque = jointPid.Update (jointCommand, jointPos, jointVelCommand, jointVel, robot.TimeSinceLastCommand);
				jointTorque [1] = 0.0f;
				jointTorque [2] = 0.0f;
			} else {
				jointTorque = jointPid.Update (jointCommand, jointPos, jointVelCommand, jointVel, robot.TimeSinceLastCommand);
			}
				
			// Safeguard: torques/velocities must be all smaller thatn TorqueMax/VelocityMax otherwise stop
			if (Math.Abs(jointTorque [0]) > TorqueMax0  || Math.Abs(jointTorque [1]) > TorqueMax1  ||  Math.Abs(jointTorque [2]) > TorqueMax2 ||
				Math.Abs(jointVel [0]) > VelMax0 || Math.Abs(jointVel [1]) > VelMax1 || Math.Abs(jointVel [2]) > VelMax2){
				robot.Stop ();

				if (logging) {
					records.Add (new RobotState (
						robot.TimeSinceLastCommand,
						robot.ControlCycleDuration,
						robot.FirmwareTimestamp,
						robot.ToolPosition,
						robot.ToolVelocity,
						robot.JointPositions,
						robot.JointVelocities,
						State,
						TargetRotation [0],
						TargetAngle[1],
						TargetAngle[2],
						startPosJoint [1],
						startPosJoint [2],
						jointCommand [1],
						jointCommand [2],
						jointTorque[0],
						jointTorque[1],
						jointTorque[2],
						Timer.ElapsedMilliseconds / 1000f
					));
				}
			}

			if (logging) {
				records.Add (new RobotState (
					robot.TimeSinceLastCommand,
					robot.ControlCycleDuration,
					robot.FirmwareTimestamp,
					robot.ToolPosition,
					robot.ToolVelocity,
					robot.JointPositions,
					robot.JointVelocities,
					State,
					TargetRotation [0],
					TargetAngle[1],
					TargetAngle[2],
					startPosJoint [1],
					startPosJoint [2],
					jointCommand [1],
					jointCommand [2],
					jointTorque[0],
					jointTorque[1],
					jointTorque[2],
					Timer.ElapsedMilliseconds / 1000f
				));
			}

            return new RobotCommand (ControlMode.Torque, jointTorque.ToVector3 ());
        }

//		Console.WriteLine ("[{0}] TA", TargetAngle.ToString ());
//		Console.WriteLine ("[{0}] SP", string.Join (", ", startPosJoint));

        /// Prints the usage instructions.
        public void PrintUsage ()
        {
            Console.WriteLine ("  [E]nable");
            Console.WriteLine ("  [D]isable");
            Console.WriteLine ("  Start [J]oint motion");
            Console.WriteLine ("  [I]dle the robot (stop moving)");
            Console.WriteLine ("  [Q]uit");
        }

        /// Sends an enable request to the robot.
        public void OnEnable ()
        {
            if (!robot.GetActiveControlFunctionName ().Equals ("zero")) {
                robot.Stop ();
            }
            robot.Enable ();
        }

        /// Smoothly stops the robot if a movement is in progress.
        public void Idle ()
        {
            startTrajJoint.EndMove ();
            if (!robot.GetActiveControlFunctionName ().Equals ("zero")) {
                robot.Stop (0.5f);
            }

			// Save Data
			if (logging){
				if(emgRecording) sendEMGStop();

				Console.WriteLine ("Experiment Done! Saving Data.");
				Timer.Reset ();
				logging = false;
				TextWriter writer = File.CreateText(@"../../log_" + FileName + ".csv");
				CsvWriter csv = new CsvWriter (writer);
				csv.WriteRecords (records);	
			}

        }
			
		/// Sends an enable request to the robot.
		public void OnDisable ()
		{
			if(emgRecording) sendEMGStop();
			EMGTrigger.disconnectFromArduino ();

			robot.Disable ();
		}

		public void PrintCurrentLocation ()
		{
//			System.Media.SystemSounds.Beep.Play();
//			System.Media.SystemSounds.Asterisk.Play();
//			System.Media.SystemSounds.Exclamation.Play();
//			System.Media.SystemSounds.Question.Play();
//			Console.Beep();
			jointPos.FromVector3 (robot.JointPositions);
			jointVel.FromVector3 (robot.JointVelocities);

			Console.WriteLine ("[{0}] JP current", string.Join(", ", jointPos));
			Console.WriteLine ("[{0}] JV current", string.Join(", ", jointVel));
		}

        /// Begins the experiment. 
		public void StartExperiment ()
        {
            if (robot.GetActiveControlFunctionName ().Equals ("zero")) {
                switch (robot.Status.handedness) {
                case RobotHandedness.Left:
                    startPosJoint = Vector<float>.Build.DenseOfArray (startPosJointLeft);
                    break;
				case RobotHandedness.Right:
					startPosJoint = Vector<float>.Build.DenseOfArray (startPosJointRight);
                    break;
                default:
                    return;
                }

				// Read Experiment Condition Name
				Console.WriteLine ("Enter Experiment Name (1P ... 5P (for Active) or 1N (For Passive)): ");
				FileName = Console.ReadLine();

				// Read Target Rotations from the external file
				ReadData ();

				// Reset All timers
                circleTimer.Reset ();
				IdleTimer.Reset ();
				Timer.Reset ();
				Timer.Start ();

				// Reset controller
				jointPid.ResetAll ();
				jointTorque.Clear ();

				// Move robot to home position before start
                jointPos.FromVector3 (robot.JointPositions);
                jointVel.FromVector3 (robot.JointVelocities);
                startTrajJoint.BeginMove (jointPos, startPosJoint);
				State = 0;

				// Start Data Logging
				logging = true;
				DoneExperiment = false;

				if(emgRecording) sendEMGStop();
				sendEMGStart ();

				robot.SetActiveControlFunction ("joint");

            } else {
                Console.WriteLine ("Press i to idle the robot before executing another movement.\n");
            }
        }
    
		// Record experiment information
		public class RobotState
		{
			public float timestampSW1 { get; set; }
			public float timestampSW2 { get; set; }
			public float timestampFW { get; set; }
			public float toolPosX { get; set; }
			public float toolPosY { get; set; }
			public float toolPosZ { get; set; }
			public float toolVelX { get; set; }
			public float toolVelY { get; set; }
			public float toolVelZ { get; set; }
			public float jointPos1 { get; set; }
			public float jointPos2 { get; set; }
			public float jointPos3 { get; set; }
			public float jointVel1 { get; set; }
			public float jointVel2 { get; set; }
			public float jointVel3 { get; set; }
			public int experimentState { get; set; }
			public float targetrotation { get; set; }
			public float targetangle1 { get; set; }
			public float targetangle2 { get; set; }
			public float startPosJoint1 { get; set; }
			public float startPosJoint2 { get; set; }
			public float jointcommand1 { get; set; }
			public float jointcommand2 { get; set; }
			public float jointTorq0 { get; set; }
			public float jointTorq1 { get; set; }
			public float jointTorq2 { get; set; }
			public float experimentTime { get; set; }

			public RobotState (
				float timestampSW1_,
				float timestampSW2_,
				float timestampFW_,
				Vector3 toolPos_,
				Vector3 toolVel_,
				Vector3 jointPos_,
				Vector3 jointVel_,
				int state_,
				float targetrotation_,
				float targetangle1_,
				float targetangle2_,
				float startPosJoint1_,
				float startPosJoint2_,
				float jointcommand1_,
				float jointcommand2_,
				float jointTorq0_,
				float jointTorq1_,
				float jointTorq2_, 
				float expTime_
			)
			{
				timestampSW1 = timestampSW1_;
				timestampSW2 = timestampSW2_;
				timestampFW = timestampFW_;
				toolPosX = toolPos_.x;
				toolPosY = toolPos_.y;
				toolPosZ = toolPos_.z;
				toolVelX = toolVel_.x;
				toolVelY = toolVel_.y;
				toolVelZ = toolVel_.z;
				jointPos1 = jointPos_.x;
				jointPos2 = jointPos_.y;
				jointPos3 = jointPos_.z;
				jointVel1 = jointVel_.x;
				jointVel2 = jointVel_.y;
				jointVel3 = jointVel_.z;
				experimentState = state_;
				targetrotation = targetrotation_;
				targetangle1 = targetangle1_;
				targetangle2 = targetangle2_;
				startPosJoint1 = startPosJoint1_;
				startPosJoint2 = startPosJoint2_;
				jointcommand1 = jointcommand1_;
				jointcommand2 = jointcommand2_;
				jointTorq0 = jointTorq0_;
				jointTorq1 = jointTorq1_;
				jointTorq2 = jointTorq2_;
				experimentTime = expTime_;
			}
		}

		/// Read Desired Angles from a textfile
		public void ReadData ()
		{
			TargetRotation = new float[100];
			records = new List<RobotState> ();
			readingCounter = 0;
			iterationCounter = 0;


			foreach (string line in File.ReadLines(@"../../" + FileName + ".txt")) //TargetRotationsTest2 //TargetRotationsAP
			{
				TargetRotation [readingCounter] = Convert.ToSingle (line);
				readingCounter++;
			}
//			Console.WriteLine ("List of Target Rotations: [{0}]", string.Join(", ", TargetRotation));
			readingCounter--;
		}

		public void sendEMGStart() {
			emgRecording = true;
			EMGTrigger.writeToArduino("S");
			Console.WriteLine ("EMG Started.");
		}

		public void sendEMGStop() {
			emgRecording = false;
			EMGTrigger.writeToArduino("E");
			Console.WriteLine ("EMG Stopped.");
		}
	
		public void EMGStartStop ()
		{

			if (emgRecording) {
				sendEMGStop ();
			}else{
				sendEMGStart ();
			}
		}


	}
}
