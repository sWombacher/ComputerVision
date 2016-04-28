#region

using System;
using System.Collections.Generic;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.Structure;
using GLab.Core;
using GLab.Core.Forms;
using GLab.Core.PluginInterfaces;
using GLab.VirtualAibo;
using GLab.VirtualAibo.Forms;
using Microsoft.Xna.Framework;
using Color = System.Drawing.Color;
using System.Threading;

#endregion

namespace GLab.Example.VrAibo
{
    internal class VrAiboExample : IPluginClient
    {
        private const int ScanLineStartY = VirtualAibo.VrAibo.SurfaceHeight - 25;
        
        // Compute alpha value (horizontal pixel expansion)
        private const float Alpha = VirtualAibo.VrAibo.FovY/VirtualAibo.VrAibo.SurfaceWidth;

        private FrmImage _frmEyeCenter;
        private FrmVrAiboRemote _frmVrAiboRemote;
        private List<Parcours> _parcours;
        private VirtualAibo.VrAibo _vrAibo;


        private Server _server;
        private readonly static int PORT = 11000;
        private bool _connected = false;
        private float _moveDistance = 1f;

        private static readonly float SPEED_ADJUSTMENT = 0.1f;
        private static readonly float ROTATION_DEGREE = 5f;


        public VrAiboExample()
        {
            Name = "VrAibo Example";
        }

        public bool GoAibo { get; set; }

        public override void Setup()
        {
            // Create a new VrAibo parcours
            _parcours = new List<Parcours>
                            {
                                // Creates a new parcour element at grid position (0,0).
                                new Parcours00(0, 0),
                                // Creates a new parcour element and rotates it 90° to the left.
                                new Parcours01(1, 0, -90),
                                // Creates a new parcour element and rotates it 180°.
                                new Parcours00(1, 1, 180),
                                // Creates a new parcour element and rotates it 90° to the right.
                                new Parcours01(0, 1, 90)
                            };
            _vrAibo = new VirtualAibo.VrAibo(_parcours) {Position = new Vector2(0, 50)};

            // Show left and right camera and center camera
            _vrAibo.ShowLeftCamera(true);
            _vrAibo.ShowRightCamera(true);
            _vrAibo.ShowCenterCamera(true);

            // Create a remote control window and link to aibo instance
            _frmVrAiboRemote = new FrmVrAiboRemote(_vrAibo, _timer);

            // Hook functionality to blue, green and red button.
            _frmVrAiboRemote.HookMeBlue += delegate { GoAibo = !GoAibo; };
            _frmVrAiboRemote.HookMeGreen += ImageProcessing;

            // Create a new window for the processed center image
            // Unprocessed left/right/center/birdview windows are created and updated automatically
            _frmEyeCenter = new FrmImage("Processing Center", VirtualAibo.VrAibo.SurfaceWidth,
                                           VirtualAibo.VrAibo.SurfaceHeight, DisplayMode.Original);

            Logger.Instance.LogInfo("Use Aibo remote to walk around.");
            Logger.Instance.LogInfo("Walk to the line and press the BLUE or GREEN button to start line tracking in RUN or STEP-Mode.");

            Start();
        }

        private void waitClient()
        {
            _server = new Server(VrAiboExample.PORT);
            _server.waitConnection();
            _connected = true;

            _vrAibo.Dispose();
            _frmEyeCenter.Dispose();
            _frmVrAiboRemote.Dispose();

            _moveDistance = 1f;
            _vrAibo = new VirtualAibo.VrAibo(_parcours) { Position = new Vector2(0, 50) };
        }

        public override void Run()
        {
            _vrAibo.Update();
            if (GoAibo)
                ImageProcessing();
        }

        public override void Teardown()
        {
            Stop();
            _vrAibo.Dispose();
            _frmEyeCenter.Dispose();
            _frmVrAiboRemote.Dispose();
            if (_connected)
                _server.dissconnetct();
        }

        private void ImageProcessing()
        {
            Bitmap leftEye = (Bitmap)_vrAibo.GetBitmapLeftEye();
            Bitmap centerEye = (Bitmap)_vrAibo.GetBitmapCenterEye();
            Bitmap rightEye = (Bitmap)_vrAibo.GetBitmapRightEye();
            
            if (!_connected)
            {
                if (_server != null)
                    _server.dissconnetct();
                waitClient();
            }

            if (!_server.sendImage(leftEye))
            {
                _connected = false;
                return;
            }
            if (!_server.sendImage(centerEye))
            {
                _connected = false;
                return;
            }
            if (!_server.sendImage(rightEye))
            {
                _connected = false;
                return;
            }

            Func<float, bool> moveAngle = f =>
            {
                _vrAibo.Turn(f);
                _vrAibo.Walk(_moveDistance);
                _vrAibo.Turn(-f);
                return true;
            };

            switch (_server.receiveAction())
            {
                case Server.ClientAction.NO_ACTION:
                case Server.ClientAction.CLOSE_CONNECTION:
                case Server.ClientAction.DISCONNECTED:
                    //_server.dissconnetct();
                    //_connected = false;
                    break;
                case Server.ClientAction.INC_SPEED:
                    _moveDistance += SPEED_ADJUSTMENT;
                    Logger.Instance.LogInfo("Speed : " + _moveDistance);
                    break;
                case Server.ClientAction.DEC_SPEED:
                    _moveDistance -= SPEED_ADJUSTMENT;
                    Logger.Instance.LogInfo("Speed : " + _moveDistance);
                    break;
                case Server.ClientAction.MOVE_FORWARD:
                    _vrAibo.Walk(_moveDistance);
                    break;
                case Server.ClientAction.MOVE_LEFT:
                    moveAngle(90);
                    break;
                case Server.ClientAction.MOVE_RIGHT:
                    moveAngle(-90);
                    break;
                case Server.ClientAction.MOVE_BACKWARD:
                    moveAngle(180);
                    break;
                case Server.ClientAction.ROTATE_LEFT:
                    _vrAibo.Turn(VrAiboExample.ROTATION_DEGREE);
                    break;
                case Server.ClientAction.ROTATE_RIGHT:
                    _vrAibo.Turn(-VrAiboExample.ROTATION_DEGREE);
                    break;
                case Server.ClientAction.HEAD_LEFT:
                case Server.ClientAction.HEAD_RIGHT:
                case Server.ClientAction.HEAD_UP:
                case Server.ClientAction.HEAD_DOWN:
                    Logger.Instance.LogInfo("Head rotation currently not supported");
                    break;
            }
            return;
            /*
            #region OLD_CODE

            // Get red-channel
            Image<Rgb, byte> center = new Image<Rgb, byte>(centerEye);
            Image<Rgb, byte> channelRed = new Image<Rgb, byte>(center.Width, center.Height);
            
            // set "channel of interest" (coi) to red
            CvInvoke.cvSetImageCOI(center.Ptr, 1);
            CvInvoke.cvSetImageCOI(channelRed.Ptr, 1);
            CvInvoke.cvCopy(center.Ptr, channelRed.Ptr, IntPtr.Zero);
            
            // reset coi
            CvInvoke.cvSetImageCOI(center.Ptr, 0);
            CvInvoke.cvSetImageCOI(channelRed.Ptr, 0);

            int lineStart = -1;
            int lineEnd = -1;

            // ...and find the line
            for (int x = 0; x < center.Width; ++x)
            {
                Rgb pixel = channelRed[ScanLineStartY, x];
                if (!pixel.Equals(new Rgb(Color.Black)))
                {
                    if (lineStart == -1)
                    {
                        lineStart = x;
                    }

                    lineEnd = x;
                }
            }

            if (lineStart != -1 && lineEnd != -1)
            {
                LineSegment2D ls = new LineSegment2D(new System.Drawing.Point(lineStart, ScanLineStartY), new System.Drawing.Point(lineEnd, ScanLineStartY));
                channelRed.Draw(ls, new Rgb(0,0,255), 2);

                int diffX = (VirtualAibo.VrAibo.SurfaceWidth/2) - (lineEnd - ((lineEnd - lineStart)/2));
                float phi = Alpha*diffX;

                if (phi != 0.0f)
                    Logger.Instance.LogInfo("Turning by " + phi + " degree");

                _vrAibo.Turn(phi/2);
                _vrAibo.Walk(0.3f);
            }
            _frmEyeCenter.SetImage(channelRed);
            #endregion
            */
        }
    }
}