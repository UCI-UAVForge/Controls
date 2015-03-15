using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ArdupilotTelemetryVisualizer.Serial.Packets;
using System.IO;

namespace ArdupilotTelemetryVisualizer.Serial
{

	internal class InputHandler
	{
		private ComController controller;
		private byte[] buffer;
		string logName;

		public event EventHandler<PacketReceivedEventArgs> PacketReceived;


		public InputHandler(ComController controller)
		{
			if (controller == null)
			{
				throw new ArgumentNullException("controller");
			}
			this.controller = controller;
			controller.DataReceived += HandleDataReceived;
			logName = "SerialReceived-" + DateTime.Now.ToString("yyyy-MM-dd-hh-mm") + ".bin";
		}

		private void HandleDataReceived(object sender, ComDataReceivedEventArgs e)
		{
			byte[] data = e.Data;
			using (FileStream fs = File.Open(logName, FileMode.OpenOrCreate))
			{
				fs.Seek(0, SeekOrigin.End);
				fs.Write(data, 0, data.Length);				
			}
			if (buffer != null)
			{
				byte[] newBuffer = new byte[buffer.Length + data.Length];
				Buffer.BlockCopy(buffer, 0, newBuffer, 0, buffer.Length);
				Buffer.BlockCopy(data, 0, newBuffer, buffer.Length, data.Length);
				buffer = newBuffer;
			}
			else
			{
				buffer = data;
			}
			ParsePackets();
		}

		private void ParsePackets()
		{
			if (buffer == null || buffer.Length == 0)
			{
				return;
			}

			while (buffer.Length > 0)
			{
				PacketType pt = (PacketType)(buffer[0] & 0x0F);
				int len = buffer[0] >> 4;
				if (buffer.Length < len + 3)
				{
					return;
				}
				if (!ValidateChecksum(buffer, len + 3))
				{
					//throw new InvalidOperationException("Invalid cheksum");
					byte[] newBuffer = new byte[buffer.Length - 1];
					Buffer.BlockCopy(buffer, 1, newBuffer, 0, newBuffer.Length);
					buffer = newBuffer;
				}
				else
				{
					Packet p = null;
					switch (pt)
					{
						case PacketType.Scalar16:
						{
							ushort value = BitConverter.ToUInt16(buffer, 2);
							p = new Scalar16Packet(value);
							break;
						}
						case PacketType.Scalar32:
						{
							Scalar32Type scalarType = (Scalar32Type)buffer[1];
							Scalar32 value = new Scalar32(BitConverter.ToUInt32(buffer, 2));
							p = new Scalar32Packet(scalarType, value);
							break;
						}
						case PacketType.Vector3_16:
						{
							Vector3_16Type type = (Vector3_16Type)buffer[1];
							p = new Vector3_16Packet(type, new Vector3_16(buffer, 2));
							break;
						}
						case PacketType.Vector3_32:
						{
							Vector3_32Type type = (Vector3_32Type)buffer[1];
							p = new Vector3_32Packet(new Vector3_32(buffer, 2), type);
							break;
						}
						case PacketType.Vector4_16:
						{
							Vector4Type type = (Vector4Type)buffer[1];
							ushort w = BitConverter.ToUInt16(buffer, 2);
							ushort x = BitConverter.ToUInt16(buffer, 4);
							ushort y = BitConverter.ToUInt16(buffer, 6);
							ushort z = BitConverter.ToUInt16(buffer, 8);
							p = new Vector4Packet(type, new Vector4(w, x, y, z));
							break;
						}
						default:
							//throw new InvalidOperationException(String.Format("Unknown type: {0}", pt));
							break;
					}

					if (p != null)
					{
						EventHandler<PacketReceivedEventArgs> handler = PacketReceived;
						if (handler != null)
						{
							handler(this, new PacketReceivedEventArgs(p));
						}
					}

					byte[] newBuffer = new byte[buffer.Length - len - 3];
					Buffer.BlockCopy(buffer, len + 3, newBuffer, 0, newBuffer.Length);
					buffer = newBuffer;
				}

			}
		}

		private static bool ValidateChecksum(byte[] data, int count)
		{
			int sum1 = 0;
			int sum2 = 0;
			for (int i = 0; i < count; i++)
			{
				sum1 = (sum1 + data[i]) % 255;
				sum2 = (sum2 + sum1) % 255;
			}
			return sum1 == 0 && sum2 == 0;
		}
	}
}
