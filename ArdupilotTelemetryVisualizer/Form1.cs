using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;

using ArdupilotTelemetryVisualizer.Serial;
using ArdupilotTelemetryVisualizer.Serial.Packets;

namespace ArdupilotTelemetryVisualizer
{
	public partial class Form1 : Form
	{
		ComController controller;
		InputHandler ih;
		OutputHandler oh;
		DateTime lastTimePacket = DateTime.Now;
		

		public Form1()
		{
			InitializeComponent();

			controller = new ComController();
			ih = new InputHandler(controller);
			oh = new OutputHandler(controller);
			PortComboBox.Items.AddRange(ComController.ComPorts.Cast<object>().ToArray());
			if(PortComboBox.Items.Count > 0)
			{
				PortComboBox.SelectedIndex = 0;
			}
			RateComboBox.Items.AddRange(ComController.BaudRates.Cast<object>().ToArray());
			RateComboBox.SelectedIndex = RateComboBox.Items.Count - 1;

			ih.PacketReceived += ih_PacketReceived;
		}

		protected override void OnFormClosing(FormClosingEventArgs e)
		{
			lock (ih)
			{
				ih.PacketReceived -= ih_PacketReceived;
			}
			base.OnFormClosing(e);
		}

		void ih_PacketReceived(object sender, PacketReceivedEventArgs e)
		{
			lock (ih)
			{
				switch (e.Packet.Type)
				{
					case PacketType.Ping:
						break;
					case PacketType.Pong:
						break;
					case PacketType.SSOutput:
						break;
					case PacketType.SSInput:
						break;
					case PacketType.Scalar16:
						break;
					case PacketType.Scalar32:
						{
							Scalar32Packet packet = (Scalar32Packet)e.Packet;
							switch (packet.ScalarType)
							{
								case Scalar32Type.Clock:
									SetClock(packet.Value.UInt32Value);
									break;
								case Scalar32Type.Heading:
									break;
								default:
									break;
							}
							break;
						}
					case PacketType.Vector2_16:
						break;
					case PacketType.Vector2_32:
						break;
					case PacketType.Vector3_16:
						break;
					case PacketType.Vector3_32:
						break;
					case PacketType.Vector4_16:
						{
							Vector4Packet packet = (Vector4Packet)e.Packet;
							switch (packet.VectorType)
							{
								case Vector4Type.RCInput:
									SetRCRaw(packet.Value.W, packet.Value.X, packet.Value.Y, packet.Value.Z);
									break;
								case Vector4Type.ControlInput:
									break;
								case Vector4Type.MotorOutput:
									break;
								default:
									break;
							}
							break;
						}
					default:
						break;
				}
			}
		}

		void SetClock(uint value)
		{
			if (InvokeRequired)
			{
				Invoke(new Action<uint>(SetClock), value);
			}
			else
			{
				MicrosTextBox.Text = value.ToString("#,###");
				DateTime now = DateTime.Now;
				TimeSpan dt = now - lastTimePacket;
				FreqLabel.Text = (20 / dt.TotalSeconds).ToString("#00.000") + "Hz";
				lastTimePacket = now;
			}
		}

		void SetRCRaw(ushort thr, ushort roll, ushort pitch, ushort yaw)
		{
			if (InvokeRequired)
			{
				Invoke(new Action<ushort, ushort, ushort, ushort>(SetRCRaw), thr, roll, pitch, yaw);
			}
			else
			{
				RawThrottleTextBox.Text = thr.ToString("#,###");
				RawRollTextBox.Text = roll.ToString("#,###");
				RawPitchTextBox.Text = pitch.ToString("#,###");
				RawYawTextBox.Text = yaw.ToString("#,###");
			}
		}

		private void ConnectButton_Click(object sender, EventArgs e)
		{
			controller.BaudRate = (int)RateComboBox.SelectedItem;
			controller.ComPort = PortComboBox.Text;
			controller.Connect();

			oh.SendSSOutput(0xFFFF);
		}
	}
}
