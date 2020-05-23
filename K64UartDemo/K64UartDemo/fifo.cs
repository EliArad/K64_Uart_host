using System;
using System.Threading;

namespace K64UartDemo
{
    public class Fifo
    {
        byte[] m_fifo;
        int m_fifo_write = 0;
        int m_fifo_read = 0;
        int m_fifoSize;
        public bool m_running = true;
        public Fifo(int size)
        {
            m_fifo = new byte[size];
            m_fifoSize = size;
            m_fifo_write = 0;
            m_fifo_read = 0;
        }
        public void Restart()
        {
            m_fifo_write = 0;
            m_fifo_read = 0;
        }
        public void Close()
        {
            m_running = false;
        }
        int GetFifoEmptiness()
        {
            lock (this)
            {
                int x;
                if (m_fifo_write == m_fifo_read)
                    x = m_fifoSize;
                else
                if (m_fifo_write > m_fifo_read)
                    x = m_fifoSize - (m_fifo_write - m_fifo_read);
                else
                    x = m_fifoSize - (m_fifoSize - m_fifo_read + m_fifo_write);
             
                return x;
            }

        }
        public void Write(byte [] buffer , int size)
        {
            while (GetFifoEmptiness() < size)
            {
                Thread.Sleep(1);
            }
            lock (this)
            {
                int _size = size;
                if (m_fifo_write >= m_fifo_read)
                {
                    int x = m_fifoSize - m_fifo_write;
                    if (x >= size)                        
                        Array.Copy(buffer, 0, m_fifo, m_fifo_write, size);
                    else
                    {
                        Array.Copy(buffer, 0, m_fifo, m_fifo_write, x);
                        size -= x;
                        Array.Copy(buffer, x, m_fifo, 0, size);
                    }                   
                }
                else
                {
                    Array.Copy(buffer, 0, m_fifo, m_fifo_write, size);
                }
                m_fifo_write = (m_fifo_write + _size) % m_fifoSize;
            }
        }
        public bool Read(byte[] buffer, int size)
        {
            while (GetFifoFullnessSize() < size)
            {
                Thread.Sleep(1);
                if (m_running == false)
                    return false;
            }

            lock (this)
            {
               

                int _size = size;
                if (m_fifo_write > m_fifo_read)
                {
                    Array.Copy(m_fifo, m_fifo_read, buffer, 0, size);
                }
                else
                {
                    int x = m_fifoSize - m_fifo_read;
                    if (size <= x)
                    {
                        Array.Copy(m_fifo, m_fifo_read, buffer, 0, size);
                    }
                    else
                    {
                        Array.Copy(m_fifo, m_fifo_read, buffer, 0, x);
                        size -= x;
                        if (size > 0)
                        {
                            Array.Copy(m_fifo, 0, buffer, x, x);
                        }
                    }
                }
                m_fifo_read = (m_fifo_read + _size) % m_fifoSize;
            }
            return true;
        }
        public int GetFifoFullnessSize()
        {
            lock (this)
            {
                if (m_fifo_write == m_fifo_read)
                    return 0;
                if (m_fifo_write > m_fifo_read)
                    return m_fifo_write - m_fifo_read;

                return m_fifoSize - m_fifo_read + m_fifo_write;
            }
        }

    }
}
