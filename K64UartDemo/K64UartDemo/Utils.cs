using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Common
{
    public class CommonUtils
    {
        public static short Little16(short n)
        {
            uint value = (uint)(n);

            value = (uint)(n & 0xFF00) >> 8 | (uint)((n & 0xFF) << 8);

            return (short)value;
        }

        public static ushort Little16(ushort n)
        {
            uint value = (uint)(n);

            value = (uint)(n & 0xFF00) >> 8 | (uint)((n & 0xFF) << 8);

            return (ushort)value;
        }

        public static uint Little32(uint n)
        {
            var v1 = (n >> 0) & 0xFF;
            var v2 = (n >> 8) & 0xFF;
            var v3 = (n >> 16) & 0xFF;
            var v4 = (n >> 24) & 0xFF;

            return (uint)(v1 << 24 | v2 << 16 | v3 << 8 | v4 << 0);
        }

        public static byte[] GetBytes(ushort data, bool swap = false)
        {
            byte[] buffer = { 0, 0 };
            if (swap == false)
            {
                buffer[0] = (byte)data;
                buffer[1] = (byte)(data >> 8);
            }
            else
            {
                buffer[1] = (byte)data;
                buffer[0] = (byte)(data >> 8);
            }
            return buffer;
        }

        public static byte[] GetBytes(short data, bool swap = false)
        {
            byte[] buffer = { 0, 0 };
            if (swap == false)
            {
                buffer[0] = (byte)data;
                buffer[1] = (byte)(data >> 8);
            }
            else
            {
                buffer[1] = (byte)data;
                buffer[0] = (byte)(data >> 8);
            }
            return buffer;
        }

        public static byte[] GetBytes(uint data, bool swap = false)
        {
            byte[] buffer = { 0, 0 ,0,0};
            if (swap == false)
            {
                buffer[0] = (byte)data;
                buffer[1] = (byte)(data >> 8);
                buffer[2] = (byte)(data >> 16);
                buffer[3] = (byte)(data >> 24);
            }
            else
            {
                buffer[3] = (byte)data;
                buffer[2] = (byte)(data >> 8);
                buffer[1] = (byte)(data >> 16);
                buffer[0] = (byte)(data >> 24);
            }
            return buffer;
        }

        public static byte[] GetBytes(ulong data)
        {
            byte[] buffer = { 0, 0, 0, 0 ,0 , 0 , 0 ,0};
            buffer[0] = (byte)data;
            buffer[1] = (byte)(data >> 8);
            buffer[2] = (byte)(data >> 16);
            buffer[3] = (byte)(data >> 24);
            buffer[4] = (byte)(data >> 32);
            buffer[5] = (byte)(data >> 40);
            buffer[6] = (byte)(data >> 48);
            buffer[7] = (byte)(data >> 56);

            return buffer;
        }

        public static byte[] GetBytes(int data, bool swap = false)
        {
            byte[] buffer = { 0, 0, 0, 0 };
            if (swap == false)
            {
                buffer[0] = (byte)data;
                buffer[1] = (byte)(data >> 8);
                buffer[2] = (byte)(data >> 16);
                buffer[3] = (byte)(data >> 24);
            }
            else
            {
                buffer[3] = (byte)data;
                buffer[2] = (byte)(data >> 8);
                buffer[1] = (byte)(data >> 16);
                buffer[0] = (byte)(data >> 24);
            }
            return buffer;
        }

        public static T StructFromByteArray<T>(byte[] bytes) where T : struct
        {
            int sz = Marshal.SizeOf(typeof(T));
            IntPtr buff = Marshal.AllocHGlobal(sz);
            Marshal.Copy(bytes, 0, buff, sz);
            T ret = (T)Marshal.PtrToStructure(buff, typeof(T));
            Marshal.FreeHGlobal(buff);
            return ret;
        }

        public static T StructFromByteArray2<T>(byte[] bytes) 
        {
            int sz = Marshal.SizeOf(typeof(T));
            IntPtr buff = Marshal.AllocHGlobal(sz);
            Marshal.Copy(bytes, 0, buff, sz);
            T ret = (T)Marshal.PtrToStructure(buff, typeof(T));
            Marshal.FreeHGlobal(buff);
            return ret;
        }

        public static byte[] StructToByteArray<T>(T structVal) where T : struct
        {
            int size = Marshal.SizeOf(structVal);
            byte[] arr = new byte[size];
            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(structVal, ptr, true);
            Marshal.Copy(ptr, arr, 0, size);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        public static byte[] StructToByteArray2<T>(T structVal)
        {
            int size = Marshal.SizeOf(structVal);
            byte[] arr = new byte[size];
            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(structVal, ptr, true);
            Marshal.Copy(ptr, arr, 0, size);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }
    }
}
