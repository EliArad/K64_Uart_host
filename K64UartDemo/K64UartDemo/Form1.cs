using Common;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace K64UartDemo
{
    public partial class Form1 : Form
    {

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct Header
        {
            public ushort UartPrefix;
            public ushort opcode;
        }
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct MSG1
        {

            public Header header;
            //public uint data;
            //public byte value;
            //public ushort value1;
            //public ushort crc;
        }
        Thread m_thread;
        int m_firstByte = 0;
        Fifo m_fifo;
        public Form1()
        {
            InitializeComponent();
            m_fifo = new Fifo(2000);
        }

        SerialPort [] m_serialPort = new SerialPort[2];
        private void button1_Click(object sender, EventArgs e)
        {
            if (m_serialPort[0] != null && m_serialPort[0].IsOpen == true)
                return;
            m_serialPort[0] = new SerialPort("COM4");
            m_serialPort[0].Parity = Parity.None;
            m_serialPort[0].BaudRate = 115200;
            m_serialPort[0].StopBits = StopBits.One;
            m_serialPort[0].DataBits = 8;
            m_serialPort[0].Handshake = Handshake.None;

            m_serialPort[0].DataReceived += M_serialPort_DataReceived;

            m_serialPort[0].Open();
            if (m_serialPort[0].IsOpen == false)
            {
                MessageBox.Show("Failed to open com port 4");
                return;
            }
            m_thread = new Thread(ReadComThread);
            m_thread.Start();

        }
        bool m_running = true;
        void ReadComThread()
        {
            byte[] buffer = new byte[1000];
            MSG1 msg1 = new MSG1();
            while (m_running)
            {
                if (m_fifo.Read(buffer, Marshal.SizeOf(msg1)) == false)
                    return;
                msg1 = CommonUtils.StructFromByteArray<MSG1>(buffer);
                Console.WriteLine(msg1.header.UartPrefix.ToString("X"));
                Console.WriteLine(msg1.header.opcode.ToString("X"));

            }
        }
        byte[] inbuffer = new byte[1000];
       
        private void M_serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // we must use fifo because the data is not arrived at like it was sent.
            int x = m_serialPort[0].BytesToRead;
            if (m_firstByte == 1)
            {
                m_serialPort[0].Read(inbuffer, 0, 1);
                x -= 1;
                m_firstByte = 0;                
            }
            
            if (x == 0)
            {
                return;
            }
            m_serialPort[0].Read(inbuffer, 0, x);
            m_fifo.Write(inbuffer, x);
        }

        private void button3_Click(object sender, EventArgs e)
        {
          
            CloseComm();
        }
        void CloseComm()
        {
            m_running = false;
            m_fifo.Close();
            for (int i = 0; i < 2; i++)
            {
                if (m_serialPort[i] != null)
                {
                    if (m_serialPort[i].IsOpen == true)
                    {
                        m_serialPort[i].Close();
                    }
                }
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            if (m_serialPort[0] == null || m_serialPort[0].IsOpen == false)
            {
                MessageBox.Show("Com is closed");
                return;
            }
            MSG1 m = new MSG1();
            m.header.opcode = 0x1234;
            m.header.UartPrefix = 0xAABB;
            //m.header.msgSize = 4;
            //m.data = 0x12345678;
            //m.value = 6;
            //m.value1 = 0x1234;
            //m.crc = 0x1234;
            byte [] data = CommonUtils.StructToByteArray<MSG1>(m);
            m_serialPort[0].Write(data, 0, data.Length);

        }

        private void button4_Click(object sender, EventArgs e)
        {
            m_firstByte = 1;
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            CloseComm();

        }
    }
}
