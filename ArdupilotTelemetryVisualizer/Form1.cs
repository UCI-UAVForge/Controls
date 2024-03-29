﻿using System;
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
using System.Threading;

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
			ih.PacketReceived -= ih_PacketReceived;
			if(controller.IsConnected)
			{
				oh.SendSSOutput(0x0000);
			}
			base.OnFormClosing(e);
		}

		void ih_PacketReceived(object sender, PacketReceivedEventArgs e)
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
				{
					Scalar16Packet packet = (Scalar16Packet)e.Packet;
					SetRCGain(packet.Value);
					break;
				}
				case PacketType.Scalar32:
				{
					Scalar32Packet packet = (Scalar32Packet)e.Packet;
					switch (packet.ScalarType)
					{
						case Scalar32Type.Clock:
							SetClock(packet.Value.UInt32Value);
							break;
						case Scalar32Type.Heading:
							SetHeading(packet.Value.SingleValue);
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
				{
					Vector3_16Packet packet = (Vector3_16Packet)e.Packet;
					switch (packet.VectorType)
					{
						case Vector3_16Type.RotPid:
							SetRotationRatePid(packet.Value.Int16X, packet.Value.Int16Y, packet.Value.Int16Z);
							break;
						default:
							break;
					}
					break;
				}
				case PacketType.Vector3_32:
				{
					Vector3_32Packet packet = (Vector3_32Packet)e.Packet;
					switch (packet.VectorType)
					{
						case Vector3_32Type.Gryo:
							SetGyro(packet.Value.SingleX, packet.Value.SingleY, packet.Value.SingleZ);
							break;
						case Vector3_32Type.Attitude:
							SetAttitude(packet.Value.SingleX, packet.Value.SingleY, packet.Value.SingleZ);
							break;
						case Vector3_32Type.Acceleration:
							break;
						case Vector3_32Type.Velocity:
							break;
						case Vector3_32Type.Position:
							SetPosition(packet.Value.SingleX, packet.Value.SingleY, packet.Value.SingleZ);
							break;
						case Vector3_32Type.Gps:
							break;
						case Vector3_32Type.AttitudePid:
							SetAttitudePid(packet.Value.SingleX, packet.Value.SingleY, packet.Value.SingleZ);
							break;
						case Vector3_32Type.VelocityPid:
							break;
						case Vector3_32Type.AltRatePid:
							break;
						case Vector3_32Type.AltPid:
							break;
						default:
							break;
					}
					break;
				}
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
							SetMotorOutput(packet.Value.W, packet.Value.X, packet.Value.Y, packet.Value.Z);
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

		void SetClock(uint value)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<uint>(SetClock), value);
			}
			else
			{
				MicrosLabel.Text = value.ToString("#,###");
				DateTime now = DateTime.Now;
				TimeSpan dt = now - lastTimePacket;
				FreqLabel.Text = (50 / dt.TotalSeconds).ToString("#00.000") + "Hz";
				lastTimePacket = now;
			}
		}

		void SetRCRaw(ushort thr, ushort roll, ushort pitch, ushort yaw)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<ushort, ushort, ushort, ushort>(SetRCRaw), thr, roll, pitch, yaw);
			}
			else
			{
				RawThrottleTextBox.Text = thr.ToString("#,##0");
				RawRollTextBox.Text = roll.ToString("#,##0");
				RawPitchTextBox.Text = pitch.ToString("#,##0");
				RawYawTextBox.Text = yaw.ToString("#,##0");
			}
		}

		void SetRCGain(ushort g)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<ushort>(SetRCGain), g);
			}
			else
			{
				GainTextBox.Text = g.ToString("#,##0");
			}
		}
		void SetMotorOutput(ushort fl, ushort bl, ushort fr, ushort br)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<ushort, ushort, ushort, ushort>(SetMotorOutput), fl, bl, fr, br);
			}
			else
			{
				FrontLeftTextBox.Text = fl.ToString("#,##0");
				BackLeftTextBox.Text = bl.ToString("#,##0");
				FrontRightTextBox.Text = fr.ToString("#,##0");
				BackRightTextBox.Text = br.ToString("#,##0");
			}
		}

		void SetGyro(float x, float y, float z)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<float, float, float>(SetGyro), x, y, z);
			}
			else
			{
				GyroRollTextBox.Text = x.ToString("#,#00.000");
				GyroPitchTextBox.Text = y.ToString("#,#00.000");
				GyroYawTextBox.Text = z.ToString("#,#00.000");
			}
		}

		void SetAttitude(float x, float y, float z)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<float, float, float>(SetAttitude), x, y, z);
			}
			else
			{
				OrientationRollTextBox.Text = x.ToString("#,#00.000");
				OrientationPitchTextBox.Text = y.ToString("#,#00.000");
				OrientationYawTextBox.Text = z.ToString("#,#00.000");
			}
		}

		void SetHeading(float h)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<float>(SetHeading), h);
			}
			else
			{
				CompassTextBox.Text = h.ToString("#,#00.000");
			}
		}

		void SetRotationRatePid(short x, short y, short z)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<short, short, short>(SetRotationRatePid), x, y, z);
			}
			else
			{
				RateRollTextBox.Text = x.ToString("#,##0");
				RatePitchTextBox.Text = y.ToString("#,##0");
				RateYawTextBox.Text = z.ToString("#,##0");
			}
		}

		void SetAttitudePid(float x, float y, float z)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<float, float, float>(SetAttitudePid), x, y, z);
			}
			else
			{
				StabRollTextBox.Text = x.ToString("#,##0.00");
				StabPitchTextBox.Text = y.ToString("#,##0.00");
				StabYawTextBox.Text = z.ToString("#,##0.00");
			}
		}

		void SetPosition(float x, float y, float z)
		{
			if (InvokeRequired)
			{
				TryInvoke(new Action<float, float, float>(SetPosition), x, y, z);
			}
			else
			{
				LocXTextBox.Text = x.ToString("#,##0.00");
				LocYTextBox.Text = y.ToString("#,##0.00");
				AltitudeTextBox.Text = z.ToString("#,##0.00");
			}
		}

		private void ConnectButton_Click(object sender, EventArgs e)
		{
			controller.BaudRate = (int)RateComboBox.SelectedItem;
			controller.ComPort = PortComboBox.Text;
			controller.Connect();

			oh.SendSSOutput(0xFFFF);
		}

		private void TryInvoke(Delegate d, params object[] args)
		{
			try
			{
				Invoke(d, args);
			}
			catch(ObjectDisposedException ex)
			{

			}
		}
	}
}
