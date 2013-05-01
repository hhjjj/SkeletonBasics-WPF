//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;


    using System;
    using System.Windows.Media.Media3D;
    using System.Windows.Media.Imaging;

    using Microsoft.Kinect;
    using Bespoke.Common;
    using Bespoke.Common.Osc;

    using System.Threading;
    using System.Net;
    using System.Net.Sockets;
    //using System.Drawing;

    using Transmitter;
    using System.Collections.Generic;
    using System.Windows.Threading;
    using ExceptionEventArgs = Bespoke.Common.ExceptionEventArgs;


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        Matrix3D transformMatrix;

        // OSC Related

        public static readonly int kinectServerPort = 5103;
        public static readonly int cmdServerPort = 8000;
        //OscBundle bundle = CreateTestBundle();
        //OscMessage OSCMsg = CreateTestMsg();
        //ITransmitter transmitter;

         
         

        private static IPEndPoint kinectServerIP = new IPEndPoint(IPAddress.Loopback, kinectServerPort);
        private static IPEndPoint cmdServerIP = new IPEndPoint(IPAddress.Loopback, cmdServerPort);


        OscServer oscCmdReceiver;
         
        //IPEndPoint kinectServerIP = new IPEndPoint(IPAddress.Parse("192.168.0.109"), Port);   
        private OscMessage kinectMsg;
        private OscMessage cmdMsg;

        private DateTime lastTime = DateTime.MinValue;
        private int FrameRate;
        private int kinectAngle;

        private double kinectHeight;
        private double startPosition;
        private double endPosition;
        private double userPosition;

        private double currentPosition;
        private double pastPosition;

        private const double moveStep = 0.005;
        private int frameCount;

        private double kinectOffset;
        

        private int kinectID;

        private string kinectMsgAddr = "/kinect";
        private string cmdMsgAddr = "/kinect/";
 
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            ipInputTextBox.Text = kinectServerIP.Address.ToString();
            portInputBox.Text = kinectServerIP.Port.ToString();
            cmdIpInput.Text = cmdServerIP.Address.ToString();
            cmdPortInput.Text = cmdServerIP.Port.ToString();

            kinectHeight = 2.15;
            startPosition = -0.2;
            endPosition = 1.2;
            frameCount = 0;
            userPosition = -100;

            currentPosition = 0;
            pastPosition = 0;

            //moveStep = 0.005;

            kinectOffset = 3.0;
            

            kinectID = 1;
            kinectIDText.Text = "Front";
            kinectIDInput.Text = "1";
            cmdMsgAddr = "/kinect/" + kinectID.ToString();
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }


        



        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // smoothing parameter
                /*
                // Some smoothing with little latency (defaults).
                // Only filters out small jitters.
                // Good for gesture recognition in games.
                TransformSmoothParameters smoothingParam = new TransformSmoothParameters();
                {
                    smoothingParam.Smoothing = 0.5f;
                    smoothingParam.Correction = 0.5f;
                    smoothingParam.Prediction = 0.5f;
                    smoothingParam.JitterRadius = 0.05f;
                    smoothingParam.MaxDeviationRadius = 0.04f;
                };
                

                // Smoothed with some latency.
                // Filters out medium jitters.
                // Good for a menu system that needs to be smooth but
                // doesn't need the reduced latency as much as gesture recognition does.
                TransformSmoothParameters smoothingParam = new TransformSmoothParameters();
                {
                    smoothingParam.Smoothing = 0.5f;
                    smoothingParam.Correction = 0.1f;
                    smoothingParam.Prediction = 0.5f;
                    smoothingParam.JitterRadius = 0.1f;
                    smoothingParam.MaxDeviationRadius = 0.1f;
                };
                */

                // Very smooth, but with a lot of latency.
                // Filters out large jitters.
                // Good for situations where smooth data is absolutely required
                // and latency is not an issue.
                
                TransformSmoothParameters smoothingParam = new TransformSmoothParameters();
                {
                    smoothingParam.Smoothing = 0.7f;
                    smoothingParam.Correction = 0.3f;
                    smoothingParam.Prediction = 1.0f;
                    smoothingParam.JitterRadius = 1.0f;
                    smoothingParam.MaxDeviationRadius = 1.0f;
                };
                
                // Turn on the skeleton stream to receive skeleton frames
                //this.sensor.SkeletonStream.Enable();
                this.sensor.SkeletonStream.Enable(smoothingParam);

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

               
               

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                    kinectAngle = this.sensor.ElevationAngle;
                    angleText.Text = kinectAngle.ToString();
                    //this.sensor.ElevationAngle = -23;
                    ResetFrameRateCounters();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
                
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }

            //transmitter = new UdpTransmitter();
            
            //transmitter.Start(bundle);
            //transmitter.Start(OSCMsg);

            // show my ip address
            ipShowTextBlock.Text = LocalIPAddress();

            kinectHeightInput.Text = kinectHeight.ToString();
            startPositionInput.Text = startPosition.ToString();
            endPositionInput.Text = endPosition.ToString();
            kinectOffsetInput.Text = kinectOffset.ToString();


            //oscCmdReceiver = new OscServer(TransportType.Udp, IPAddress.Loopback, cmdServerPort);
            oscCmdReceiver = new OscServer(TransportType.Udp, IPAddress.Parse(LocalIPAddress()), cmdServerPort);
            //oscCmdReceiver = new OscServer(IPAddress.Parse("224.25.26.27"), cmdServerPort);
            oscCmdReceiver.FilterRegisteredMethods = false;
            //oscCmdServer.RegisterMethod(oscCmd);
            oscCmdReceiver.RegisterMethod(cmdMsgAddr);
            oscCmdReceiver.BundleReceived += new EventHandler<OscBundleReceivedEventArgs>(oscCmdReceiver_BundleReceived);
            oscCmdReceiver.MessageReceived += new EventHandler<OscMessageReceivedEventArgs>(oscCmdReceiver_MessageReceived);
            oscCmdReceiver.ReceiveErrored += new EventHandler<ExceptionEventArgs>(oscCmdReceiver_ReceiveErrored);
            oscCmdReceiver.ConsumeParsingExceptions = false;
            oscCmdReceiver.Start();
            
            
        }

        

        

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
            //transmitter.Stop();
        
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];
            

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);   
                }
            }

            if (this.sensor != null && this.sensor.SkeletonStream != null)
            {
                float closestDistance = 10000f; // Start with a far enough distance
                int closestID = 0;

                if (!this.sensor.SkeletonStream.AppChoosesSkeletons)
                {
                    this.sensor.SkeletonStream.AppChoosesSkeletons = true; // Ensure AppChoosesSkeletons is set
                }

                foreach (Skeleton skel in skeletons)
                {

                    // find the closest skeleton
                    if (skel.TrackingState != SkeletonTrackingState.NotTracked)
                    {
                        if (skel.Position.Z < closestDistance)
                        {
                            closestDistance = skel.Position.Z;
                            closestID = skel.TrackingId;
                        }
                    }

                }
                if (closestID > 0)
                {
                    this.sensor.SkeletonStream.ChooseSkeletons(closestID);
                }
                          
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {


                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {

                            
                            if (isAllJointsTracked(skel))
                            {
                                allTrackedText.Text = "All Tracked";
                            }
                            else
                            {
                                allTrackedText.Text = "Not All";
                            }

                            this.DrawBonesAndJoints(skel, dc);



                            // Calculate angles and send OSC msg
                            CalculateAndSendOSC(skel);

                        }
                        //else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        //{
                        //    dc.DrawEllipse(
                        //    this.centerPointBrush,
                        //    null,
                        //    this.SkeletonPointToScreen(skel.Position),
                        //    BodyCenterThickness,
                        //    BodyCenterThickness);
                        //}
                    }
                    
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
            UpdateFrameRate();
            frameRateText.Text = FrameRate.ToString();
            if (InfraredEmitterCheckbox.IsChecked == true)
            {
                kinectRunStatus.Background = new SolidColorBrush(Colors.Black);
                kinectRunStatus.Foreground = new SolidColorBrush(Colors.Blue);
                kinectRunStatus.Content = "Not Running";
            }
            else 
            {
                kinectRunStatus.Background = new SolidColorBrush(Colors.Red);
                kinectRunStatus.Foreground = new SolidColorBrush(Colors.Yellow);
                kinectRunStatus.Content = "Running";
            }

            frameCount++;
        }
       

        void CalculateAndSendOSC(Skeleton skel)
        {
            Joint hipCenter = skel.Joints[JointType.HipCenter];
            Joint kneeLeft = skel.Joints[JointType.KneeLeft];
            Joint kneeRight = skel.Joints[JointType.KneeRight];

            transformMatrix = new Matrix3D(1, 0, 0, 0, 0, Math.Cos(DegreeToRadian(kinectAngle)), -1 * Math.Sin(DegreeToRadian(kinectAngle)), 0, 0, Math.Sin(DegreeToRadian(kinectAngle)), Math.Cos(DegreeToRadian(kinectAngle)), 0, 0, 0, 0, 1);


            //if (hipCenter.TrackingState == JointTrackingState.NotTracked)
            //{ 
                
            //}
            float LRMove = hipCenter.Position.X *-1;
            LRMoveTextBox.Text = LRMove.ToString();

            //float FBMove = (hipCenter.Position.Z - (float)2.9) *(float) Math.Cos(23 *Math.PI / 180.0) *(float)(-1.0);
            //FBMoveTextBox.Text = FBMove.ToString();
            Vector3D skelVec = new Vector3D(skel.Position.X, skel.Position.Y, skel.Position.Z);
            Vector3D transSkel = Vector3D.Multiply(skelVec, transformMatrix);
            userPosition = kinectOffset - skelVec.Z;

            float FBMove = (float)userPosition;

            currentPosition = userPosition;
            if ((currentPosition - pastPosition) > moveStep)
            {
                FBMove = (float)1.0;
            }
            else if ((currentPosition - pastPosition) < moveStep*(-1.0) )
            {
                FBMove = (float)(-1.0);
            }
            else if ((currentPosition - pastPosition) > moveStep * (-1.0) && (currentPosition - pastPosition) < moveStep)
            {
                FBMove = (float)0;
            }
            //FBMove = (float)(currentPosition - pastPosition);
            if (frameCount > 5)
            {
                frameCount = 0;
                pastPosition = currentPosition;
            }

            //float FBMove = (float)userPosition;
            //float FBMove = (float)(3.0)-(float)Math.Sqrt(Math.Pow(skel.Position.X,2)+Math.Pow(skel.Position.Y,2)+Math.Pow(skel.Position.Z,2));
            FBMoveTextBox.Text = FBMove.ToString();

            Vector3D hipCenterVec = new Vector3D(hipCenter.Position.X, hipCenter.Position.Y, hipCenter.Position.Z);
            Vector3D transHipCenter = Vector3D.Multiply(hipCenterVec, transformMatrix);
            transHipCenter.Y = kinectHeight + transHipCenter.Y;
            UDMoveTextBox.Text = transHipCenter.Y.ToString();
            //float UDMove = hipCenter.Position.Y;
            //UDMoveTextBox.Text = UDMove.ToString();
            

            //float kneeHeight = (float)(1.0)+(kneeLeft.Position.Y + kneeRight.Position.Y) / 2;
            Vector3D kneeRightVec = new Vector3D(kneeRight.Position.X, kneeRight.Position.Y, kneeRight.Position.Z);
            Vector3D transkneeRight = Vector3D.Multiply(kneeRightVec, transformMatrix);

            //float kneeHeight = (kneeLeft.Position.Y + kneeRight.Position.Y) / 2;
            //kneeHeight = kneeHeight - FBMove * (float)Math.Sin(23 * Math.PI / 180.0);
            //kneeHeightTextBox.Text = kneeHeight.ToString();
            transkneeRight.Y = kinectHeight + transkneeRight.Y;
            kneeHeightTextBox.Text = transkneeRight.Y.ToString();
            //kneeHeightTextBox.Text = kneeRight.Position.Y.ToString();

            double FBAngle = GetFBAngle(skel);
            backBendingTextBox.Text = FBAngle.ToString();

            double LRAngle = GetLRAngle(skel);
            rightBendingTextBox.Text = LRAngle.ToString();
                            
            double shoulderRotation = GetShoulderRotation(skel);
            upperTwistAngleTextBox.Text = shoulderRotation.ToString();

            double bodyRotation = GetBodyRotation(skel);
            bodyTwistAngleTextBox.Text = bodyRotation.ToString();


            if (OSCCheckBox.IsChecked == true)
            {
                if (userPosition <= startPosition)
                {
                    cmdMsg = new OscMessage(cmdServerIP, cmdMsgAddr);
                    cmdMsg.Append("IN");
                    cmdMsg.Send(cmdServerIP);
                }
                if (userPosition > startPosition && userPosition < endPosition)
                {
                    if (FBAngle == Double.NaN) { FBAngle = 15; }
                    if (LRAngle == Double.NaN) { LRAngle = 0; }
                    if (shoulderRotation == Double.NaN) { shoulderRotation = 0; }
                    if (bodyRotation == Double.NaN) { bodyRotation = 0; }


                    kinectMsg = new OscMessage(kinectServerIP, kinectMsgAddr);
                    kinectMsg.Append((float)LRMove);
                    kinectMsg.Append((float)FBMove);
                    kinectMsg.Append((float)transHipCenter.Y);
                    kinectMsg.Append((float)FBAngle);
                    kinectMsg.Append((float)LRAngle);
                    kinectMsg.Append((float)shoulderRotation);
                    kinectMsg.Append((float)bodyRotation);
                    kinectMsg.Append((float)transkneeRight.Y);
                    kinectMsg.Send(kinectServerIP);
                }

                if (userPosition >= endPosition)
                { 

                    // let main controller know
                    cmdMsg = new OscMessage(cmdServerIP, cmdMsgAddr);
                    cmdMsg.Append("OUT");
                    cmdMsg.Send(cmdServerIP);
                }
            }
        }

        double GetShoulderRotation(Skeleton skel)
        {
            Joint leftShoulder = skel.Joints[JointType.ShoulderLeft];
            Joint rightShoulder = skel.Joints[JointType.ShoulderRight];
            if (leftShoulder.TrackingState == JointTrackingState.NotTracked ||
                rightShoulder.TrackingState == JointTrackingState.NotTracked)
            {
                return Double.NaN;
            }
            else
            {
                return Math.Atan2(
                    leftShoulder.Position.Z -rightShoulder.Position.Z,
                    rightShoulder.Position.X -leftShoulder.Position.X) * 180.0 / Math.PI;
            }
        }

        double GetBodyRotation(Skeleton skel)
        {
            Joint leftAnkle = skel.Joints[JointType.AnkleLeft];
            Joint rightAnkle = skel.Joints[JointType.AnkleRight];
            if (leftAnkle.TrackingState == JointTrackingState.NotTracked ||
                rightAnkle.TrackingState == JointTrackingState.NotTracked)
            {
                return Double.NaN;
            }
            else
            {
                return Math.Atan2(
                    leftAnkle.Position.Z - rightAnkle.Position.Z,
                    rightAnkle.Position.X - leftAnkle.Position.X) * 180.0 / Math.PI;
            }
        }

        double GetLRAngle(Skeleton skel)
        {
            Joint shoulderCenter = skel.Joints[JointType.ShoulderCenter];
            Joint hipCenter = skel.Joints[JointType.HipCenter];
            
            if (shoulderCenter.TrackingState == JointTrackingState.NotTracked ||hipCenter.TrackingState == JointTrackingState.NotTracked)
            {
                return Double.NaN;
            }
            else 
            {
                double angle;
                Vector3D shoulderCenVec = new Vector3D(shoulderCenter.Position.X, shoulderCenter.Position.Y, 0);
                Vector3D hipCenVec = new Vector3D(hipCenter.Position.X, hipCenter.Position.Y, 0);
                Vector3D hipToSholderCenterVec = shoulderCenVec - hipCenVec;
                Vector3D yAxisVec = new Vector3D(0, 1, 0);

                angle = Vector3D.AngleBetween(hipToSholderCenterVec, yAxisVec);
                if (shoulderCenter.Position.X - hipCenter.Position.X >= 0)
                {
                    return angle;
                }
                else
                { 
                    return angle* -1.0;
                }
            }  
        }

        double GetFBAngle(Skeleton skel)
        {

            Joint shoulderCenter = skel.Joints[JointType.ShoulderCenter];
            Joint hipCenter = skel.Joints[JointType.HipCenter];
            Joint ankleLeft = skel.Joints[JointType.AnkleLeft];
            Joint ankleRight = skel.Joints[JointType.AnkleRight];

            if (shoulderCenter.TrackingState == JointTrackingState.NotTracked || hipCenter.TrackingState == JointTrackingState.NotTracked)
            {
                return Double.NaN;
            }
            else
            {

                Vector3D shoulderCenVec = new Vector3D(shoulderCenter.Position.X, shoulderCenter.Position.Y, shoulderCenter.Position.Z);
                Vector3D hipCenVec = new Vector3D(hipCenter.Position.X, hipCenter.Position.Y, hipCenter.Position.Z);

                double angle;
                Vector3D hipToShoulderVec = shoulderCenVec - hipCenVec;
                Vector3D nZAxis = new Vector3D(0, 0, -1);
                angle = Vector3D.AngleBetween(hipToShoulderVec, nZAxis);

                return angle - 75; // consider bone init value

                //if (shoulderCenter.Position.Z - hipCenter.Position.Z > 0)
                //{
                //    return angle;
                //}
                //else 
                //{
                //    return angle * -1;
                //}

            }

        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);

           // msg = new OscMessage(sourceEndPoint, TestMethod);
           //// msg.Append((int)(skeleton.Joints[JointType.WristRight].Position.X *200));
           // //msg.Append((int)(skeleton.Joints[JointType.WristRight].Position.Y*200));
           // msg.Append(skeleton.Joints[JointType.WristRight].Position.X);
           // msg.Append(skeleton.Joints[JointType.WristRight].Position.Y);
           // msg.Append(skeleton.Joints[JointType.WristRight].Position.Z);
           // msg.Send(sourceEndPoint);
            

            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
          
            
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        private string LocalIPAddress()
        {
            if (!System.Net.NetworkInformation.NetworkInterface.GetIsNetworkAvailable())
            {
                return null;
            }

            IPHostEntry host;
            string localIP = "";
            host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (IPAddress ip in host.AddressList)
            {
                if (ip.AddressFamily == AddressFamily.InterNetwork)
                {
                    localIP = ip.ToString();
                }
            }
            return localIP;
        }

        //private IPEndPoint updateIPAddress(IPEndPoint ip, String addr, String port)
        //{
        //    ip.Address = IPAddress.Parse(addr);
        //    ip.Port = Convert.ToInt32(port);
        //    return ip;
        //}

        private void ipUpdateButton_Click(object sender, RoutedEventArgs e)
        {
            
            if (kinectServerIP != null)
            {
                kinectServerIP.Address = IPAddress.Parse(ipInputTextBox.Text);
                kinectServerIP.Port = Convert.ToInt32(portInputBox.Text);
                ipInputTextBox.Text = kinectServerIP.Address.ToString();
                portInputBox.Text = kinectServerIP.Port.ToString();
            }
        }

        private void cmdIpUpdateButton_Click(object sender, RoutedEventArgs e)
        {
            if (cmdServerIP != null)
            {
                cmdServerIP.Address = IPAddress.Parse(cmdIpInput.Text);
                cmdServerIP.Port = Convert.ToInt32(cmdPortInput.Text);
                cmdIpInput.Text = cmdServerIP.Address.ToString();
                cmdPortInput.Text = cmdServerIP.Port.ToString();
            }
        }


        protected int TotalFrames { get; set; }

        protected int LastFrames { get; set; }

        protected void ResetFrameRateCounters()
        {
           
            this.lastTime = DateTime.MinValue;
            this.TotalFrames = 0;
            this.LastFrames = 0;
            
        }

        protected void UpdateFrameRate()
        {
            ++this.TotalFrames;

            DateTime cur = DateTime.Now;
            var span = cur.Subtract(this.lastTime);

            if (span >= TimeSpan.FromSeconds(1))
            {
                // A straight cast will truncate the value, leading to chronic under-reporting of framerate.
                // rounding yields a more balanced result
                this.FrameRate = (int)Math.Round((this.TotalFrames - this.LastFrames) / span.TotalSeconds);
                this.LastFrames = this.TotalFrames;
                this.lastTime = cur;
            }
        }

        private double DegreeToRadian(double angle)
        {
            return Math.PI * angle / 180.0;
        }

        private bool isAllJointsTracked(Skeleton skel)
        {
            foreach (Joint joint in skel.Joints)
            {
                if (joint.TrackingState != JointTrackingState.Tracked)
                {
                    return false;
                }
            }

            return true;
        }

        private void kinectHeightSetButton_Click(object sender, RoutedEventArgs e)
        {
            kinectHeight = Convert.ToDouble(kinectHeightInput.Text);
        }

        private void startPositionSetButton_Click(object sender, RoutedEventArgs e)
        {
            startPosition = Convert.ToDouble(startPositionInput.Text);
        }

        private void endPositionSetButton_Click(object sender, RoutedEventArgs e)
        {
            endPosition = Convert.ToDouble(endPositionInput.Text);
        }

        private void kinectIDButton_Click(object sender, RoutedEventArgs e)
        {
            kinectID = Convert.ToInt32(kinectIDInput.Text);
            cmdMsgAddr = "/kinect/" + kinectID.ToString();
            if (kinectID == 1)
            {
                kinectIDText.Text = "Front";
            }
            if (kinectID == 2)
            {
                kinectIDText.Text = "Back";
            }
        }

        private void oscCmdReceiver_BundleReceived(object sender, OscBundleReceivedEventArgs e)
        {
            sBundlesReceivedCount++;

            OscBundle bundle = e.Bundle;

            this.Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
            {
                //statusIndicator.Fill = new SolidColorBrush(Colors.Green);
            }));
            Console.WriteLine(string.Format("\nBundle Received [{0}:{1}]: Nested Bundles: {2} Nested Messages: {3}", bundle.SourceEndPoint.Address, bundle.TimeStamp, bundle.Bundles.Count, bundle.Messages.Count));
            Console.WriteLine("Total Bundles Received: {0}", sBundlesReceivedCount);

        }

        private void oscCmdReceiver_MessageReceived(object sender, OscMessageReceivedEventArgs e)
        {
            sMessagesReceivedCount++;

            OscMessage message = e.Message;

            Console.WriteLine(string.Format("\nMessage Received [{0}]: {1}", message.SourceEndPoint.Address, message.Address));
            Console.WriteLine(string.Format("Message contains {0} objects.", message.Data.Count));

            if (kinectID == 1)
            {
                if (message.Address == "/kinect/1")
                {
                    if (message.Data[0].ToString() == "START")
                    {
                        this.Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                        {
                            InfraredEmitterCheckbox.IsChecked = false;
                            this.sensor.ForceInfraredEmitterOff = false;
                        }));

                    }
                }

                if (message.Address == "/kinect/1")
                {
                    if (message.Data[0].ToString() == "STOP")
                    {
                        this.Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                        {
                            InfraredEmitterCheckbox.IsChecked = true;
                            this.sensor.ForceInfraredEmitterOff = true;
                        }));

                    }
                }
            }

            if (kinectID == 2)
            {
                if (message.Address == "/kinect/2")
                {
                    if (message.Data[0].ToString() == "START")
                    {
                        this.Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                        {
                            InfraredEmitterCheckbox.IsChecked = false;
                            this.sensor.ForceInfraredEmitterOff = false;
                        }));

                    }
                }

                if (message.Address == "/kinect/2")
                {
                    if (message.Data[0].ToString() == "STOP")
                    {
                        this.Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                        {
                            InfraredEmitterCheckbox.IsChecked = true;
                            this.sensor.ForceInfraredEmitterOff = true;
                        }));

                    }
                }
            }

            for (int i = 0; i < message.Data.Count; i++)
            {
                string dataString;

                if (message.Data[i] == null)
                {
                    dataString = "Nil";
                }
                else
                {
                    dataString = (message.Data[i] is byte[] ? BitConverter.ToString((byte[])message.Data[i]) : message.Data[i].ToString());
                }
                Console.WriteLine(string.Format("[{0}]: {1}", i, dataString));
            }

            Console.WriteLine("Total Messages Received: {0}", sMessagesReceivedCount);
        }

        private static void oscCmdReceiver_ReceiveErrored(object sender, ExceptionEventArgs e)
        {
            Console.WriteLine("Error during reception of packet: {0}", e.Exception.Message);
        }


        private static int sBundlesReceivedCount;
        private static int sMessagesReceivedCount;

        private void kinectOffsetButton_Click(object sender, RoutedEventArgs e)
        {
            kinectOffset = Convert.ToDouble(kinectOffsetInput.Text);
            kinectOffsetInput.Text = kinectOffset.ToString();
        }

        private void InfraredEmitterCheckbox_Click(object sender, RoutedEventArgs e)
        {
            if (this.sensor != null)
            {
                if (InfraredEmitterCheckbox.IsChecked == true)
                {
                    this.sensor.ForceInfraredEmitterOff = true;
                }
                else
                {
                    this.sensor.ForceInfraredEmitterOff = false;
                }
            }
        }
       
        



    }
}