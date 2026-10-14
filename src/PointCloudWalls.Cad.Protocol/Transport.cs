using System;
using System.IO;
using System.IO.Pipes;
using System.Text;
using System.Threading;

namespace PointCloudWalls.Cad
{
    /// <summary>Length-prefixed UTF-8 frames over a stream (4-byte little-endian length, then the JSON text).</summary>
    public static class CadFraming
    {
        private const int MaxFrame = 256 * 1024 * 1024;

        public static void WriteFrame(Stream s, string text)
        {
            byte[] body = Encoding.UTF8.GetBytes(text);
            byte[] len = BitConverter.GetBytes(body.Length);
            if (!BitConverter.IsLittleEndian) Array.Reverse(len);
            s.Write(len, 0, 4);
            s.Write(body, 0, body.Length);
            s.Flush();
        }

        public static string ReadFrame(Stream s)
        {
            byte[] len = ReadExactly(s, 4);
            if (!BitConverter.IsLittleEndian) Array.Reverse(len);
            int n = BitConverter.ToInt32(len, 0);
            if (n < 0 || n > MaxFrame) throw new InvalidDataException("Bad frame length " + n);
            return Encoding.UTF8.GetString(ReadExactly(s, n));
        }

        private static byte[] ReadExactly(Stream s, int count)
        {
            var buf = new byte[count];
            int got = 0;
            while (got < count)
            {
                int r = s.Read(buf, got, count - got);
                if (r <= 0) throw new EndOfStreamException("Connection closed while reading");
                got += r;
            }
            return buf;
        }
    }

    public static class CadPipe
    {
        /// <summary>Per-user pipe name, so two Windows users on one machine never talk to each other's AutoCAD.</summary>
        public static string DefaultName => "PointCloudWalls.Cad." + Environment.UserName;
    }

    /// <summary>Runs inside AutoCAD: accepts one connection at a time, hands the request to <c>handler</c>, sends the answer back.</summary>
    public sealed class CadServer : IDisposable
    {
        private readonly string _name;
        private readonly Func<string, string> _handler;
        private Thread _thread;
        private volatile bool _stop;

        public bool Running => _thread != null && _thread.IsAlive;
        public int Handled { get; private set; }
        public string LastError { get; private set; } = "";
        public string PipeName => _name;

        public CadServer(string pipeName, Func<string, string> handler) { _name = pipeName; _handler = handler; }

        public void Start()
        {
            if (Running) return;
            _stop = false;
            _thread = new Thread(Loop) { IsBackground = true, Name = "PCW CAD listener" };
            _thread.Start();
        }

        private readonly object _gate = new object();

        /// <summary>
        /// Accept loop. The NEXT listening instance is created before the current request is handled, so a client that arrives
        /// while a previous one is being answered never hits a moment with nobody listening.
        /// </summary>
        private void Loop()
        {
            while (!_stop)
            {
                NamedPipeServerStream pipe = null;
                try
                {
                    pipe = new NamedPipeServerStream(_name, PipeDirection.InOut, NamedPipeServerStream.MaxAllowedServerInstances,
                                                     PipeTransmissionMode.Byte, PipeOptions.None);
                    pipe.WaitForConnection();
                    if (_stop) { pipe.Dispose(); break; }
                    var accepted = pipe; pipe = null;
                    ThreadPool.QueueUserWorkItem(_ => Serve(accepted));
                }
                catch (Exception ex)
                {
                    try { pipe?.Dispose(); } catch { }
                    if (_stop) break;
                    LastError = ex.Message;
                    Thread.Sleep(200);
                }
            }
        }

        private void Serve(NamedPipeServerStream pipe)
        {
            try
            {
                using (pipe)
                {
                    string request = CadFraming.ReadFrame(pipe);
                    string response;
                    lock (_gate)                                   // one request at a time reaches AutoCAD, in arrival order
                    {
                        try { response = _handler(request); }
                        catch (Exception ex)
                        {
                            LastError = ex.Message;
                            response = new PublishResult { Ok = false, Message = "CAD side error: " + ex.Message }.ToJson();
                        }
                        Handled++;
                    }
                    CadFraming.WriteFrame(pipe, response);
                    try { if (pipe.IsConnected) pipe.WaitForPipeDrain(); } catch { /* Windows only */ }
                }
            }
            catch (Exception ex) { LastError = ex.Message; }        // a client that vanished mid-message: ignore, keep listening
        }

        public void Stop()
        {
            _stop = true;
            try   // wake the blocked WaitForConnection
            {
                using (var c = new NamedPipeClientStream(".", _name, PipeDirection.InOut)) c.Connect(300);
            }
            catch { }
            _thread?.Join(1500);
        }

        public void Dispose() => Stop();
    }

    /// <summary>Runs inside Revit.</summary>
    public static class CadClient
    {
        /// <summary>Sends one request and returns the answer. Throws <see cref="TimeoutException"/> if nothing is listening.</summary>
        public static string Request(string pipeName, string json, int connectTimeoutMs = 3000)
        {
            using (var c = new NamedPipeClientStream(".", pipeName, PipeDirection.InOut))
            {
                try { c.Connect(connectTimeoutMs); }
                catch (TimeoutException)
                {
                    throw new TimeoutException("AutoCAD is not listening (pipe '" + pipeName + "').");
                }
                CadFraming.WriteFrame(c, json);
                return CadFraming.ReadFrame(c);
            }
        }

        public static PublishResult Publish(string pipeName, PublishBatch batch, int connectTimeoutMs = 3000)
            => PublishResult.FromJson(Request(pipeName, batch.ToJson(), connectTimeoutMs));

        /// <summary>Is an AutoCAD listener there? Returns the drawing name, or null.</summary>
        public static string Ping(string pipeName, int connectTimeoutMs = 1500)
        {
            try { var r = PublishResult.FromJson(Request(pipeName, CadMessage.PingJson(), connectTimeoutMs)); return r.Ok ? r.Drawing : null; }
            catch { return null; }
        }
    }
}
