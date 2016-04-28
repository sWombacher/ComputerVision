using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

// socket stuff
using System.Net;
using System.Net.Sockets;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.Structure;
using GLab.Core;

namespace GLab.Example.VrAibo
{
    class Server
    {
        public enum ClientAction
        {
            NO_ACTION,
            CLOSE_CONNECTION, DISCONNECTED,
            INC_SPEED, DEC_SPEED,
            MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACKWARD,
            ROTATE_LEFT, ROTATE_RIGHT,
            HEAD_LEFT, HEAD_RIGHT, HEAD_UP, HEAD_DOWN
        }

        Socket _listener;
        IPHostEntry _ipHostInfo;
        IPAddress _ipAddress;
        IPEndPoint _localEndPoint;
        Socket _handler;

        byte[] _buffer = new byte[256 * 256 * 3];

        public Server(int port)
        {
            // Establish the local endpoint for the socket.
            // Dns.GetHostName returns the name of the 
            // host running the application.
            _ipHostInfo = Dns.Resolve(Dns.GetHostName());
            _ipAddress = _ipHostInfo.AddressList[0];
            _localEndPoint = new IPEndPoint(_ipAddress, port);
            Logger.Instance.LogInfo("Adress: " + _ipAddress.ToString() + ", Port: " + port);

            // Create a TCP/IP socket.
            _listener = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            _listener.Bind(_localEndPoint);
            _listener.Listen(1);
        }

        // Incoming data from the client.
        public string _data = null;

        public void waitConnection()
        {
            // Start listening for connections.
            Console.WriteLine("Waiting for a connection...");

            // Program is suspended while waiting for an incoming connection.
            _handler = _listener.Accept();
            _data = null;
        }
        public void dissconnetct()
        {
            if (_handler != null)
            {
                if (_handler.Connected)
                    _handler.Shutdown(SocketShutdown.Both);
                _handler.Close();
            }

            if (_listener != null)
            {
                if (_listener.Connected)
                    _listener.Shutdown(SocketShutdown.Both);
                _listener.Close();
            }
        }

        public bool sendImage(Bitmap img)
        {
            for (int y = 0, bufferIter = 0; y < img.Height; ++y)
            {
                for (int x = 0; x < img.Width; ++x)
                {
                    _buffer[bufferIter++] = (byte)img.GetPixel(x, y).B;
                    _buffer[bufferIter++] = (byte)img.GetPixel(x, y).G;
                    _buffer[bufferIter++] = (byte)img.GetPixel(x, y).R;
                }
            }
            try
            {
                int sentData = 0;
                sentData += _handler.Send(_buffer);
                while (sentData != _buffer.Length) 
                {
                    byte[] shitCSharp = new byte[_buffer.Length - sentData];
                    for (int i = 0; i < shitCSharp.Length; ++i)
                        shitCSharp[i] = _buffer[sentData + i];
                    sentData += _handler.Send(shitCSharp);
                } 
                return true;
            }
            catch (Exception e)
            {
            }
            return false;
        }
        public Server.ClientAction receiveAction()
        {
            try
            {
                byte[] bytes = new byte[256];
                int bytesRec = _handler.Receive(bytes);
                _data = Encoding.ASCII.GetString(bytes, 0, bytesRec);
                #region
                if (_data.IndexOf("<EOF>") > -1)
                {
                    return Server.ClientAction.CLOSE_CONNECTION;
                }

                if (_data == "CLOSE_CONNECTION")
                {
                    return Server.ClientAction.CLOSE_CONNECTION;
                }
                else if (_data == "INC_SPEED")
                {
                    return Server.ClientAction.INC_SPEED;
                }
                else if (_data == "DEC_SPEED")
                {
                    return Server.ClientAction.DEC_SPEED;
                }
                else if (_data == "MOVE_FORWARD")
                {
                    return Server.ClientAction.MOVE_FORWARD;
                }
                else if (_data == "MOVE_LEFT")
                {
                    return Server.ClientAction.MOVE_LEFT;
                }
                else if (_data == "MOVE_RIGHT")
                {
                    return Server.ClientAction.MOVE_RIGHT;
                }
                else if (_data == "MOVE_BACKWARD")
                {
                    return Server.ClientAction.MOVE_BACKWARD;
                }
                else if (_data == "ROTATE_LEFT")
                {
                    return Server.ClientAction.ROTATE_LEFT;
                }
                else if (_data == "ROTATE_RIGHT")
                {
                    return Server.ClientAction.ROTATE_RIGHT;
                }
                else if (_data == "HEAD_LEFT")
                {
                    return Server.ClientAction.HEAD_LEFT;
                }
                else if (_data == "HEAD_RIGHT")
                {
                    return Server.ClientAction.HEAD_RIGHT;
                }
                else if (_data == "HEAD_UP")
                {
                    return Server.ClientAction.HEAD_UP;
                }
                else if (_data == "HEAD_DOWN")
                {
                    return Server.ClientAction.HEAD_DOWN;
                }
                else
                {
                    return Server.ClientAction.NO_ACTION;
                }
                #endregion
            }
            catch (Exception e)
            {
            }
            return Server.ClientAction.NO_ACTION;
        }
    }
}
