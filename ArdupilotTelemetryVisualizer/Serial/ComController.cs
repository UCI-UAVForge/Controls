using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Threading;

namespace ArdupilotTelemetryVisualizer.Serial
{

	public class ComController
	{
		public static int[] BaudRates
		{
			get
			{
				return new[]
                {
                    110,
                    300,
                    600,
                    1200,
                    2400,
                    4800,
                    9600,
                    14400,
                    19200,
                    28800,
                    38400,
                    56000,
                    57600,
                    115200,
                };
			}
		}

		public static string[] ComPorts
		{
			get
			{
				string[] portNames = SerialPort.GetPortNames();
				Array.Sort(portNames);
				return portNames;
			}
		}

		protected readonly object syncRoot = new object();
		//private readonly Timer disconnectWatchdog;
		protected SerialPort sp;

		public string ComPort { get; set; }
		public int BaudRate { get; set; }
		public bool IsConnected
		{
			get
			{
				lock (syncRoot)
				{
					return sp != null && sp.IsOpen;
				}
			}
		}

		public event EventHandler Connected;
		public event EventHandler Disconnected;
		public event EventHandler<ComDataReceivedEventArgs> DataReceived;


		public ComController()
		{
			//disconnectWatchdog = new Timer(WatchdogCallback, null, Timeout.Infinite, Timeout.Infinite);
		}


		protected void OnConnected(EventArgs e)
		{
			EventHandler handler = Connected;
			if (handler != null)
			{
				handler(this, e);
			}
		}

		protected void OnDisconnected(EventArgs e)
		{
			EventHandler handler = Disconnected;
			if (handler != null)
			{
				handler(this, e);
			}
		}

		private void WatchdogCallback(object state)
		{
			Disconnect();
		}

		public virtual bool Connect()
		{
			lock (syncRoot)
			{
				sp = new SerialPort(ComPort, BaudRate, Parity.None, 8, StopBits.One);
				sp.Open();
				sp.DataReceived += HandleDataReceived;
			}

			OnConnected(EventArgs.Empty);
			//disconnectWatchdog.Change(15000, Timeout.Infinite);
			return true;
		}

		public virtual void Disconnect()
		{
			lock (syncRoot)
			{
				if (sp == null)
				{
					return;
				}

				sp.DataReceived -= HandleDataReceived;
				sp.Close();
				sp = null;
			}

			OnDisconnected(EventArgs.Empty);
		}

		public void Send(byte[] data)
		{
			if (data == null)
			{
				throw new ArgumentNullException("data");
			}

			lock (syncRoot)
			{
				if (sp == null || !sp.IsOpen)
				{
					throw new InvalidOperationException("Not connected");
				}

				sp.Write(data, 0, data.Length);
			}
		}

		protected void HandleDataReceived(object sender, SerialDataReceivedEventArgs e)
		{
			//disconnectWatchdog.Change(5000, Timeout.Infinite);
			byte[] data;
			lock (syncRoot)
			{
				if (sp == null || !sp.IsOpen)
				{
					return;
				}

				data = new byte[sp.BytesToRead];
				sp.Read(data, 0, data.Length);
			}

			EventHandler<ComDataReceivedEventArgs> handler = DataReceived;
			if (handler != null)
			{
				handler(this, new ComDataReceivedEventArgs(data));
			}
		}
	}
}
