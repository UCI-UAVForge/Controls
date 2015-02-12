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

namespace ArdupilotTelemetryVisualizer
{
	public partial class Form1 : Form
	{
		SerialPort sp;
		byte[] buffer = new byte[100];
		int buf_offset = 0;
		int counter = 0;
		long[] lastPacket = new long[20];
		int packetCur = 0;
		DateTime lastTime;

		public Form1()
		{
			InitializeComponent();
			sp = new SerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
			sp.DataReceived += sp_DataReceived;
			sp.Open();
			lastTime = DateTime.Now;
		}

		void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
		{
			int readCount;

			readCount = sp.Read(buffer, buf_offset, buffer.Length - buf_offset);
			buf_offset = (buf_offset + readCount) % buffer.Length;
			if(buf_offset == 0)
			{
				if(counter++ % 30 == 0)
				{
				this.Invoke(new Action(() =>
					{
						ThrottleTextBox.Text = BitConverter.ToInt32(buffer, 0).ToString();
						PitchTextBox.Text = BitConverter.ToSingle(buffer, 4).ToString();
						RollTextBox.Text = BitConverter.ToSingle(buffer, 8).ToString();
						YawTextBox.Text = BitConverter.ToSingle(buffer, 12).ToString();

						GyroPitchTextBox.Text = BitConverter.ToSingle(buffer, 16).ToString();
						GyroRollTextBox.Text = BitConverter.ToSingle(buffer, 20).ToString();
						GyroYawTextBox.Text = BitConverter.ToSingle(buffer, 24).ToString();
						CompassTextBox.Text = BitConverter.ToSingle(buffer, 28).ToString();
						OrientationPitchTextBox.Text = BitConverter.ToSingle(buffer, 32).ToString();
						OrientationRollTextBox.Text = BitConverter.ToSingle(buffer, 36).ToString();
						OrientationYawTextBox.Text = BitConverter.ToSingle(buffer, 40).ToString();

						StabPitchTextBox.Text = BitConverter.ToSingle(buffer, 44).ToString();
						StabRollTextBox.Text = BitConverter.ToSingle(buffer, 48).ToString();
						StabYawTextBox.Text = BitConverter.ToSingle(buffer, 52).ToString();

						RatePitchTextBox.Text = BitConverter.ToInt16(buffer, 56).ToString();
						RateRollTextBox.Text = BitConverter.ToInt16(buffer, 60).ToString();
						RateYawTextBox.Text = BitConverter.ToInt16(buffer, 64).ToString();

						FrontLeftTextBox.Text = BitConverter.ToUInt32(buffer, 68).ToString();
						FrontRightTextBox.Text = BitConverter.ToUInt32(buffer, 72).ToString();
						BackLeftTextBox.Text = BitConverter.ToUInt32(buffer, 76).ToString();
						BackRightTextBox.Text = BitConverter.ToUInt32(buffer, 80).ToString();

						RawThrottleTextBox.Text = BitConverter.ToUInt16(buffer, 88).ToString();
						RawPitchTextBox.Text = BitConverter.ToUInt16(buffer, 86).ToString();
						RawRollTextBox.Text = BitConverter.ToUInt16(buffer, 84).ToString();
						RawYawTextBox.Text = BitConverter.ToUInt16(buffer, 90).ToString();


						MillisTextBox.Text = BitConverter.ToInt32(buffer, 92).ToString("#,###");
						MicrosTextBox.Text = BitConverter.ToInt32(buffer, 96).ToString("#,###");
					}));
				}
				DateTime thisTime = DateTime.Now;
				lastPacket[packetCur] = thisTime.Ticks - lastTime.Ticks;
				packetCur = (packetCur + 1) % lastPacket.Length;
				double freq = (double)TimeSpan.TicksPerSecond/ ((double)lastPacket.Sum() / lastPacket.Length);
				this.Invoke(new Action(() =>
					{
						FreqLabel.Text = freq.ToString("00.0") + "Hz";
						PacketTimeLabel.Text = lastTime.Ticks.ToString("#,###");
					}));
				lastTime = thisTime;
			}
		}
	}
}
