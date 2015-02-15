using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArdupilotTelemetryVisualizer.Serial.Packets
{
	internal abstract class Packet
	{
		private static readonly Packet pingPacket = new NoPayloadPacket(PacketType.Ping);
		private static readonly Packet pongPacket = new NoPayloadPacket(PacketType.Pong);

		public static Packet PingPacket { get { return pingPacket; } }
		public static Packet PongPacket { get { return pongPacket; } }

		public PacketType Type { get; private set; }

		protected Packet(PacketType type)
		{
			Type = type;
		}

		public abstract byte[] GetBytes();

		protected void AppendChecksum(byte[] data)
		{
			int sum1 = 0;
			int sum2 = 0;
			for (int i = 0; i < data.Length - 2; i++)
			{
				sum1 = (sum1 + data[i]) % 255;
				sum2 = (sum2 + sum1) % 255;
			}

			int check1 = 255 - ((sum1 + sum2) % 255);
			int check2 = 255 - ((sum1 + check1) % 255);
			data[data.Length - 2] = (byte)check1;
			data[data.Length - 1] = (byte)check2;
		}

		private class NoPayloadPacket : Packet
		{
			private readonly byte[] packetBytes;

			public NoPayloadPacket(PacketType type)
				: base(type)
			{
				packetBytes = new byte[3];
				packetBytes[0] = (byte)((int)Type);
				AppendChecksum(packetBytes);
			}

			public override byte[] GetBytes()
			{
				// Copy the bytes to a new array to prevent anyone from changing values in the chached array.
				byte[] packet = new byte[3];
				Buffer.BlockCopy(packetBytes, 0, packet, 0, 3);
				return packet;
			}
		}
	}
}
