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
		StreamWriter sw;

		public Form1()
		{
			InitializeComponent();
			sp = new SerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
			sp.DataReceived += sp_DataReceived;
			sp.Open();
			lastTime = DateTime.Now;
			FileStream fs = File.Open(DateTime.Now.ToString("yyyy-MM-dd mm.ss)") + ".log", FileMode.Create);
			sw = new StreamWriter(fs, Encoding.UTF8);
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
						sw.Write(DateTime.Now);

						ThrottleTextBox.Text = BitConverter.ToInt32(buffer, 0).ToString();
						PitchTextBox.Text = BitConverter.ToSingle(buffer, 4).ToString();
						RollTextBox.Text = BitConverter.ToSingle(buffer, 8).ToString();
						YawTextBox.Text = BitConverter.ToSingle(buffer, 12).ToString();
						sw.Write(ThrottleTextBox.Text);
						sw.Write(',');
						sw.Write(PitchTextBox.Text);
						sw.Write(',');
						sw.Write(RollTextBox.Text);
						sw.Write(',');
						sw.Write(YawTextBox.Text);
						sw.Write(',');
						

						GyroPitchTextBox.Text = BitConverter.ToSingle(buffer, 16).ToString();
						GyroRollTextBox.Text = BitConverter.ToSingle(buffer, 20).ToString();
						GyroYawTextBox.Text = BitConverter.ToSingle(buffer, 24).ToString();
						CompassTextBox.Text = BitConverter.ToSingle(buffer, 28).ToString();
						OrientationPitchTextBox.Text = BitConverter.ToSingle(buffer, 32).ToString();
						OrientationRollTextBox.Text = BitConverter.ToSingle(buffer, 36).ToString();
						OrientationYawTextBox.Text = BitConverter.ToSingle(buffer, 40).ToString();
						sw.Write(GyroPitchTextBox.Text);
						sw.Write(',');
						sw.Write(GyroRollTextBox.Text);
						sw.Write(',');
						sw.Write(GyroYawTextBox.Text);
						sw.Write(',');
						sw.Write(OrientationPitchTextBox.Text);
						sw.Write(',');
						sw.Write(OrientationRollTextBox.Text);
						sw.Write(',');
						sw.Write(OrientationYawTextBox.Text);
						sw.Write(',');

						StabPitchTextBox.Text = BitConverter.ToSingle(buffer, 44).ToString();
						StabRollTextBox.Text = BitConverter.ToSingle(buffer, 48).ToString();
						StabYawTextBox.Text = BitConverter.ToSingle(buffer, 52).ToString();
						sw.Write(StabPitchTextBox.Text);
						sw.Write(',');
						sw.Write(StabRollTextBox.Text);
						sw.Write(',');
						sw.Write(StabYawTextBox.Text);
						sw.Write(',');

						RatePitchTextBox.Text = BitConverter.ToInt16(buffer, 56).ToString();
						RateRollTextBox.Text = BitConverter.ToInt16(buffer, 60).ToString();
						RateYawTextBox.Text = BitConverter.ToInt16(buffer, 64).ToString();
						sw.Write(StabPitchTextBox.Text);
						sw.Write(',');
						sw.Write(StabRollTextBox.Text);
						sw.Write(',');
						sw.Write(StabYawTextBox.Text);
						sw.Write(',');

						FrontLeftTextBox.Text = BitConverter.ToUInt32(buffer, 68).ToString();
						FrontRightTextBox.Text = BitConverter.ToUInt32(buffer, 72).ToString();
						BackLeftTextBox.Text = BitConverter.ToUInt32(buffer, 76).ToString();
						BackRightTextBox.Text = BitConverter.ToUInt32(buffer, 80).ToString();
						sw.Write(FrontLeftTextBox.Text);
						sw.Write(',');
						sw.Write(FrontRightTextBox.Text);
						sw.Write(',');
						sw.Write(BackLeftTextBox.Text);
						sw.Write(',');
						sw.Write(BackRightTextBox.Text);
						sw.Write(',');

						RawThrottleTextBox.Text = BitConverter.ToUInt16(buffer, 88).ToString();
						RawPitchTextBox.Text = BitConverter.ToUInt16(buffer, 86).ToString();
						RawRollTextBox.Text = BitConverter.ToUInt16(buffer, 84).ToString();
						RawYawTextBox.Text = BitConverter.ToUInt16(buffer, 90).ToString();
						sw.Write(RawThrottleTextBox.Text);
						sw.Write(',');
						sw.Write(RawPitchTextBox.Text);
						sw.Write(',');
						sw.Write(RawRollTextBox.Text);
						sw.Write(',');
						sw.Write(RawYawTextBox.Text);
						sw.Write(',');


						MillisTextBox.Text = BitConverter.ToInt32(buffer, 92).ToString("#,###");
						MicrosTextBox.Text = BitConverter.ToInt32(buffer, 96).ToString("#,###");
						sw.Write(MicrosTextBox.Text);
						sw.WriteLine();
						sw.Flush();
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
