using System;
using System.Runtime.InteropServices;

namespace IsoTherm
{
    public static class Helpers
    {
        public static bool ParseStringToInt(string numString, out int num)
        {
            numString = numString.ToLower();
            if (numString.StartsWith("0x"))
            {
                try
                {
                    num = Convert.ToInt32(numString, 16);
                    return true;
                }
                catch
                {
                    num = -1;
                    return false;
                }

            }
            else
            {
                return int.TryParse(numString, out num);                
            }
        }

        public static int ConvertIntAsHex(int num)
        {                      
            return int.Parse(num.ToString(), System.Globalization.NumberStyles.HexNumber);
        }

        [DllImport("kernel32", SetLastError = true)]
        private static extern bool FreeLibrary(IntPtr hModule);

        public static void UnloadImportedDll(string DllPath)
        {
            foreach (System.Diagnostics.ProcessModule mod in System.Diagnostics.Process.GetCurrentProcess().Modules)
            {
                if (mod.FileName == DllPath)
                {
                    FreeLibrary(mod.BaseAddress);
                }
            }
        }
    }
}