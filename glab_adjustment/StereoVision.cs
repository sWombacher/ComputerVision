#region

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using Emgu.CV;
using Emgu.CV.Structure;
using GLab.Core;
using GLab.Core.Forms;
using GLab.Core.PluginInterfaces;
using GLab.VirtualAibo;
using GLab.VirtualAibo.Forms;
using Microsoft.Xna.Framework;
using System.Runtime.InteropServices;

#endregion

namespace Frame.VrAibo
{
    internal class StereoVision : IPluginClient
    {
        private const float AiboSpeed = 0.2f;
        private FrmImage _frmImage;
        private FrmImage _referenceImage;
        private FrmVrAiboRemote _frmVrAiboRemote;
        private GLab.VirtualAibo.VrAibo _vrAibo;
        private GLab.StereoVision.StereoVision _stereoVision;


        private Server _server;
        private readonly static int PORT = 11000;
        private bool _connected = false;
        static private readonly Vector2 START_POS = new Vector2(50, 100);
        static private readonly float START_ANGLE = 90;

        private List<Parcours> parcours;


        public StereoVision()
        {
            Name = "Stereo Vision Lab";
        }

        public bool GoAibo { get; set; }

        public override void Setup()
        {
            
            /*
             * DEFAULT PARCOURS LAB TASK WITH TERRAIN
             */
            parcours = new List<Parcours>
                                          {
                                              new ParcoursTerrain(),
                                              new Parcours00(0, 0, 0, false),
                                              new Parcours01(1, 0, -90, false),
                                              new Parcours02(1, 1, 180, false),
                                              new Parcours03(0, 1, 90, false)
                                          };

            // Creates a new Virtual Aibo
            _vrAibo = new GLab.VirtualAibo.VrAibo(parcours) { Position = START_POS, Rotation = START_ANGLE };

            /*
             * TREASURE LAB TASK
             */
            //List<Parcours> stripes = new List<Parcours>
            //                             {
            //                                 new Stripe00(),
            //                                 new Stripe01(),
            //                                 new Stripe02(),
            //                                 new Stripe03(),
            //                                 new StripeTreasure()
            //                             };

            /*
             * TREASURE LAB TASK WITH TERRAIN
             */
            //List<Parcours> stripes = new List<Parcours>
            //                             {
            //                                 new ParcoursTerrain(),
            //                                 new Stripe00(false),
            //                                 new Stripe01(false),
            //                                 new Stripe02(false),
            //                                 new Stripe03(false),
            //                                 new StripeTreasure(false)
            //                             };

            // Creates a new Virtual Aibo
            //_vrAibo = new GLab.VirtualAibo.VrAibo(stripes) { Position = new Vector2(0, 20) };

            // Use left and right cameras, omit center camera
            _vrAibo.ShowLeftCamera(true);
            _vrAibo.ShowRightCamera(true);
            _vrAibo.ShowCenterCamera(false);
            
            // Create a remote control window an link to Aibo instance
            _frmVrAiboRemote = new FrmVrAiboRemote(_vrAibo, _timer);

            // Hook functionality to blue green and red button.
            _frmVrAiboRemote.HookMeBlue += delegate { GoAibo = !GoAibo; };
            _frmVrAiboRemote.HookMeGreen += ImageProcessing;
            // _frmVrAiboRemote.HookMeRed += delegate { };

            Logger.Instance.LogInfo("Use Aibo remote to walk around.");
            Logger.Instance.LogInfo("Walk onto the red line and press the BLUE button to start line tracking.");

            // Setup stereo vision facilities
            _stereoVision = new GLab.StereoVision.StereoVision(
                                            GLab.VirtualAibo.VrAibo.SurfaceWidth,
                                            GLab.VirtualAibo.VrAibo.SurfaceHeight,
                                            _vrAibo.Disparity*200
                                            );
            Start();
        }

        public override void Run()
        {
            if (GoAibo)
            {
                ImageProcessing();
            }
            _vrAibo.Update();
        }

        public override void Teardown()
        {
            // Stop the execution of Run()
            Stop();

            // Clean up after yourself!
            _vrAibo.Dispose();
            _frmImage.Dispose();
            _frmVrAiboRemote.Dispose();
            _frmImage.Dispose();
            _referenceImage.Dispose();
        }

        /// <summary>
        ///   Let's do some image processing and move aibo
        /// </summary>
        private void ImageProcessing()
        {
            TrackLine();
        }

        private void waitClient()
        {
            _server = new Server(StereoVision.PORT);
            _server.waitConnection();
            _connected = true;

            _vrAibo.Dispose();
            _frmVrAiboRemote.Dispose();

            _vrAibo = new GLab.VirtualAibo.VrAibo(parcours) { Position = START_POS, Rotation = START_ANGLE };
        }

        private bool TrackLine()
        {
            // Get images from vraibo
            Bitmap centerEye_rgb = (Bitmap)_vrAibo.GetBitmapCenterEye();

            Bitmap leftEye = (Bitmap)_vrAibo.GetBitmapLeftEye();
            Bitmap rightEye = (Bitmap)_vrAibo.GetBitmapRightEye();
            Image<Gray, byte> leftEye_gray = new Image<Gray, byte>(leftEye);
            Image<Gray, byte> rightEye_gray = new Image<Gray, byte>(rightEye);
            Image<Gray, byte> centerEye_gray = new Image<Gray, byte>(centerEye_rgb);

            #region OLD
            /*
            // Let the stereo vision class compute a disparity map
            _stereoVision.ComputeDisparityMap(ref leftEye_gray, ref rightEye_gray);

            // Display the resulting disparity map. Darker values mean less disparity which means the object is farer away
            //_frmImage.SetImage(_stereoVision.DisparityMapDisplayable);

            // This is a calculation to get a point just below the image center. This point is where depth perception is shown
            int probe_x = _stereoVision.RefImage.Width / 2;
            int probe_y = (int)(_stereoVision.RefImage.Height * 0.6);

            // Get and display the reference image. Every pixel here corresponds to a pixel in the disparity map
            // If you want to detect objects for which you want to have the distance, use *this* image only.
            // Also, we draw a white cross on the point in the image from where the depth is taken
            Image<Gray, byte> tmpRefImg = _stereoVision.RefImage;
            tmpRefImg.Draw(new Cross2DF(new PointF(probe_x, probe_y), 8, 8), new Gray(255), 1);
            _referenceImage.SetImage(tmpRefImg);
            */

            // This prints the current depth on the point to the log facility, if there is enough change
            // to the depth value
            #endregion

            Func<bool> disconnect = () =>
            {
                Logger.Instance.ClearLog();
                _connected = false;
                return true;
            };

            if (!_connected)
            {
                if (_server != null)
                    _server.dissconnetct();
                waitClient();
            }

            if (!_server.sendAiboPosition((Int16)(_vrAibo.Position.X * 100), (Int16)(_vrAibo.Position.Y * 100), (Int16)(_vrAibo.Rotation * 100)))
                return disconnect();
            if (!_server.sendGrayImage(centerEye_gray))
                return disconnect();
            if (!_server.sendGrayImage(leftEye_gray))
                return disconnect();
            if (!_server.sendGrayImage(rightEye_gray))
                return disconnect();

            Func<float, float, bool> moveAngle = (angle, dist) =>
            {
                _vrAibo.Turn(angle);
                _vrAibo.Walk(dist);
                _vrAibo.Turn(-angle);
                return true;
            };

            List<KeyValuePair<Server.ClientAction, Int16>> actions = _server.receiveAction();
            if (actions.Count == 0)
                return disconnect();
            
            actions.ForEach(delegate (KeyValuePair<Server.ClientAction, Int16> action)
            {
                switch (action.Key)
                {
                    case Server.ClientAction.NO_ACTION:
                        break;
                    case Server.ClientAction.MOVE_FORWARD:
                        _vrAibo.Walk((float)action.Value / 100f);
                        break;
                    case Server.ClientAction.MOVE_LEFT:
                        moveAngle(90, (float)action.Value / 100f);
                        break;
                    case Server.ClientAction.MOVE_RIGHT:
                        moveAngle(-90, (float)action.Value / 100f);
                        break;
                    case Server.ClientAction.MOVE_BACKWARD:
                        moveAngle(180, (float)action.Value / 100f);
                        break;
                    case Server.ClientAction.ROTATE_LEFT:
                        _vrAibo.Turn((float)action.Value / 100f);
                        if (_vrAibo.Rotation > 360f) _vrAibo.Rotation -= 360;
                        if (_vrAibo.Rotation < -360f) _vrAibo.Rotation += 360;
                        break;
                    case Server.ClientAction.ROTATE_RIGHT:
                        _vrAibo.Turn(-(float)action.Value / 100f);
                        if (_vrAibo.Rotation >  360f) _vrAibo.Rotation -= 360;
                        if (_vrAibo.Rotation < -360f) _vrAibo.Rotation += 360;
                        break;
                    case Server.ClientAction.TELEPORT:
                        Vector2 tmp = START_POS;
                        tmp.X += action.Value & 255;
                        tmp.Y += action.Value >> 8;
                        _vrAibo.Position = tmp;
                        break;
                    case Server.ClientAction.HEAD_LEFT:
                        _vrAibo.HeadYaw += action.Value;
                        break;
                    case Server.ClientAction.HEAD_RIGHT:
                        _vrAibo.HeadYaw -= action.Value;
                        break;
                    case Server.ClientAction.HEAD_UP:
                    case Server.ClientAction.HEAD_DOWN:
                        Logger.Instance.LogInfo("Head up/down currently not supported");
                        break;
                }
            });
            return true;
        }
    }
}
