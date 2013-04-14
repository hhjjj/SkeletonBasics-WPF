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


    using Microsoft.Kinect;
    using Bespoke.Common;
    using Bespoke.Common.Osc;

    using System.Threading;
    using System.Net;
    using System.Net.Sockets;
    //using System.Drawing;

    using Transmitter;


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


        // OSC Related

        public static readonly int Port = 5103;
         OscBundle bundle = CreateTestBundle();
         OscMessage OSCMsg = CreateTestMsg();
         //ITransmitter transmitter;

         //private static readonly string AliveMethod = "/osctest/alive";
         private static readonly string kinectMsg = "/kinect";

         IPEndPoint sourceEndPoint = new IPEndPoint(IPAddress.Loopback, Port);
         string oscIPAddress;
         string oscPort;

        
         
         //IPEndPoint sourceEndPoint = new IPEndPoint(IPAddress.Parse("192.168.0.109"), Port);   
         private OscMessage msg;

         private DateTime lastTime = DateTime.MinValue;
         private int FrameRate;


        


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            ipInputTextBox.Text = sourceEndPoint.Address.ToString();
            portInputBox.Text = sourceEndPoint.Port.ToString();
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
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
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
            
            
        }

        private static OscBundle CreateTestBundle()
        {
            IPEndPoint sourceEndPoint = new IPEndPoint(IPAddress.Loopback, Port);
            OscBundle bundle = new OscBundle(sourceEndPoint);

            //OscBundle nestedBundle = new OscBundle(sourceEndPoint);
            OscMessage nestedMessage = new OscMessage(sourceEndPoint, kinectMsg);
            //nestedMessage.AppendNil();
            //nestedMessage.Append("Some String");
            //nestedMessage.Append(10);
            //nestedMessage.Append(50);
            //nestedMessage.Append(100000L);
            //nestedMessage.Append(1234.567f);
            //nestedMessage.Append(10.0012345);
            //nestedMessage.Append(new byte[] { 1, 2, 3, 4 });
            //nestedMessage.Append(new OscTimeTag());
            //nestedMessage.Append('c');
            //nestedMessage.Append(true);
            //nestedMessage.Append(false);
            //nestedMessage.Append(float.PositiveInfinity);
            //nestedBundle.Append(nestedMessage);
            //bundle.Append(nestedBundle);

            //OscMessage message = new OscMessage(sourceEndPoint, AliveMethod);
            //message.Append(9876.543f);
            //bundle.Append(message);

            nestedMessage.Append(10);
            nestedMessage.Append(82);
            bundle.Append(nestedMessage);
            //bundle.Append(nestedBundle);


            return bundle;
        }

        private static OscMessage CreateTestMsg()
        {
            IPEndPoint sourceEndPoint = new IPEndPoint(IPAddress.Loopback, Port);

            OscMessage msg = new OscMessage(sourceEndPoint, kinectMsg);

            msg.Append(10);
            msg.Append(82);
            



            return msg;
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
                            this.DrawBonesAndJoints(skel, dc);

                            // Calculate angles and send OSC msg
                            CalculateAndSendOSC(skel);

                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                        
                    }
                    
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
            UpdateFrameRate();
            frameRateText.Text = FrameRate.ToString();
        }

        void CalculateAndSendOSC(Skeleton skel)
        {
            Joint hipCenter = skel.Joints[JointType.HipCenter];
            Joint kneeLeft = skel.Joints[JointType.KneeLeft];
            Joint kneeRight = skel.Joints[JointType.KneeRight];

          

            //if (hipCenter.TrackingState == JointTrackingState.NotTracked)
            //{ 
                
            //}
            float LRMove = hipCenter.Position.X *-1;
            LRMoveTextBox.Text = LRMove.ToString();

            float FBMove = (hipCenter.Position.Z - (float)2.9) *(float) Math.Cos(23 *Math.PI / 180.0) *(float)(-1.0);
            FBMoveTextBox.Text = FBMove.ToString();

            float UDMove = hipCenter.Position.Y;
            UDMoveTextBox.Text = UDMove.ToString();

            float kneeHeight = (float)(1.0)+(kneeLeft.Position.Y + kneeRight.Position.Y) / 2;
            kneeHeight = kneeHeight - FBMove * (float)Math.Sin(23 * Math.PI / 180.0);
            kneeHeightTextBox.Text = kneeHeight.ToString();

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
                if (FBAngle == Double.NaN) { FBAngle = 15; }
                if (LRAngle == Double.NaN) { LRAngle = 0; }
                if (shoulderRotation == Double.NaN) { shoulderRotation = 0; }
                if (bodyRotation == Double.NaN) { bodyRotation = 0; }

                
                msg = new OscMessage(sourceEndPoint, kinectMsg);
                msg.Append((float)LRMove);
                msg.Append((float)FBMove);
                msg.Append((float)UDMove);
                msg.Append((float)FBAngle);
                msg.Append((float)LRAngle);
                msg.Append((float)shoulderRotation);
                msg.Append((float)bodyRotation);
                msg.Append((float)kneeHeight);
                msg.Send(sourceEndPoint);


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
            oscIPAddress = ipInputTextBox.Text;
            oscPort = portInputBox.Text;
            if (sourceEndPoint != null)
            {
                sourceEndPoint.Address = IPAddress.Parse(oscIPAddress);
                sourceEndPoint.Port = Convert.ToInt32(oscPort);
                ipInputTextBox.Text = sourceEndPoint.Address.ToString();
                portInputBox.Text = sourceEndPoint.Port.ToString();
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




    }
}