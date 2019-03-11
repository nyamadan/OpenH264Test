using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace OpenH264Test
{
    class Program
    {
        const string OpenH264 = "openh264-1.8.0-win64.dll";

        [DllImport(OpenH264)]
        extern public static Int32 WelsCreateSVCEncoder(IntPtr ppEncoder);

        [DllImport(OpenH264)]
        extern public static Int32 WelsDestroySVCEncoder(IntPtr pEncoder);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        delegate Int32 Initialize(IntPtr self, IntPtr param);

        [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
        delegate Int32 SetOption(IntPtr self, Int32 eOptionId, IntPtr pOption);

        static void Main(string[] args)
        {
            IntPtr pEncoder = IntPtr.Zero;
            Int32 rv = 0;

            unsafe
            {
                rv = WelsCreateSVCEncoder(new IntPtr(&pEncoder));
            }

            var methods = new IntPtr[10];
            Marshal.Copy(Marshal.ReadIntPtr(pEncoder, 0), methods, 0, methods.Length);
            var _Initialize = Marshal.GetDelegateForFunctionPointer<Initialize>(methods[0]);
            var _SetOption = Marshal.GetDelegateForFunctionPointer<SetOption>(methods[7]);

            // I420
            unsafe
            {
                Int32 videoFormat = 23;
                rv = _SetOption(pEncoder, 0, new IntPtr(&videoFormat));
            }

            // Log: detail
            unsafe
            {
                Int32 logLevel = 1 << 4;
                rv = _SetOption(pEncoder, 25, new IntPtr(&logLevel));
            }

            var param = new Byte[24];
            BitConverter.GetBytes(0).CopyTo(param, 0);
            BitConverter.GetBytes(320).CopyTo(param, 4);
            BitConverter.GetBytes(240).CopyTo(param, 8);
            BitConverter.GetBytes(500000).CopyTo(param, 12);
            BitConverter.GetBytes(1).CopyTo(param, 16);
            BitConverter.GetBytes(15.0f).CopyTo(param, 20);

            unsafe
            {
                fixed(Byte *p = param)
                {
                    rv = _Initialize(pEncoder, new IntPtr(p));
                }
            }

            WelsDestroySVCEncoder(pEncoder);
        }
    }
}
