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

namespace Frame.VrAibo
{
    class Server
    {
        public enum ClientAction : byte {
            NO_ACTION = 0,

            // optional features
            TELEPORT,

            // actual commands
            MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACKWARD,
            ROTATE_LEFT, ROTATE_RIGHT,
            HEAD_LEFT, HEAD_RIGHT, HEAD_UP, HEAD_DOWN
        };

        Socket _listener;
        IPHostEntry _ipHostInfo;
        IPAddress _ipAddress;
        IPEndPoint _localEndPoint;
        Socket _handler;

        private const int CLIENT_ACTION_SIZE = 3;

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



        bool sendBytes(byte[] buffer)
        {
            try
            {
                int sentData = 0;
                sentData += _handler.Send(buffer);
                while (sentData != buffer.Length)
                {
                    byte[] shitCSharp = new byte[buffer.Length - sentData];
                    for (int i = 0; i < shitCSharp.Length; ++i)
                        shitCSharp[i] = buffer[sentData + i];
                    sentData += _handler.Send(shitCSharp);
                }
                return true;
            }
            catch (Exception e)
            {
            }
            return false;
        }


        byte[] _aiboPosBuffer = new byte[6];
        public bool sendAiboPosition(Int16 pos_x, Int16 pos_y, Int16 rotation)
        {
            byte[] tmp;
            tmp = BitConverter.GetBytes(pos_x);
            _aiboPosBuffer[0] = tmp[0];
            _aiboPosBuffer[1] = tmp[1];

            tmp = BitConverter.GetBytes(pos_y);
            _aiboPosBuffer[2] = tmp[0];
            _aiboPosBuffer[3] = tmp[1];

            tmp = BitConverter.GetBytes(rotation);
            _aiboPosBuffer[4] = tmp[0];
            _aiboPosBuffer[5] = tmp[1];

            return this.sendBytes(_aiboPosBuffer);
        }

        byte[] _buffer = new byte[256 * 256];
        public bool sendGrayImage(Image<Gray, byte> img)
        {
            for (int y = 0, bufferIter = 0; y < img.Height; ++y)
            {
                for (int x = 0; x < img.Width; ++x)
                {
                    _buffer[bufferIter++] = img.Data[y, x, 0];
                }
            }

            return this.sendBytes(_buffer);
        }

        public List<KeyValuePair<ClientAction, Int16>> receiveAction()
        {
            try
            {
                byte[] bytes = new byte[256];
                bytes[0] = 1 + CLIENT_ACTION_SIZE;
                int bytesRec = 0;
                {
                    SocketError ec;
                    int CONNECTION_LOST_COUNTER = 0;
                    do
                    {
                        int readBytes = _handler.Receive(bytes, bytesRec, bytes[0] - bytesRec, SocketFlags.None, out ec);
                        bytesRec += readBytes;
                        if (ec != SocketError.Success || readBytes == 0)
                        {
                            ++CONNECTION_LOST_COUNTER;
                            if (CONNECTION_LOST_COUNTER > 10 || ec == SocketError.ConnectionAborted)
                                return new List<KeyValuePair<ClientAction, Int16>>();
                            System.Threading.Thread.Sleep(20);
                        }

                    } while (bytes[0] != bytesRec);
                }

                int actionCount = bytesRec / Server.CLIENT_ACTION_SIZE;
                List<KeyValuePair<ClientAction, Int16>> toReturn = new List<KeyValuePair<ClientAction, Int16>>(actionCount);
                for (int currentAction = 0; currentAction < actionCount; ++currentAction)
                {
                    int byteIdx = currentAction * Server.CLIENT_ACTION_SIZE + 1;
                    Server.ClientAction clientAction = (Server.ClientAction)bytes[byteIdx];
                    Int16 parameter = (short)((bytes[byteIdx + 2] << 8) | bytes[byteIdx + 1]);
                    toReturn.Add(new KeyValuePair<ClientAction, Int16>(clientAction, parameter));
                }
                return toReturn;
            }
            catch (Exception e)
            {
                return new List<KeyValuePair<ClientAction, Int16>>();
            }
        }
    }
}