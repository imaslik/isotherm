using System.Collections;
using System.Runtime.InteropServices;

namespace IsoTherm.Thermals
{
    [StructLayout(LayoutKind.Sequential)]
    public struct I2CStructure
    {
        public byte byTransType;
        public byte bySlvDevAddr; // Slave addres  [7:1] and [0] - read or write        
        public ushort wMemoryAddr;
        public ushort wCount;
        [MarshalAsAttribute(UnmanagedType.ByValArray, SizeConst = 1088, ArraySubType = UnmanagedType.I1)]
        public byte[] Data;

        public I2CStructure(byte addr, ushort len)
        {
            byTransType = 0;
            bySlvDevAddr = addr;
            wMemoryAddr = 0;
            wCount = len;
            Data = new byte[1088];
        }
    }

    internal class IsoThermHelper
    {
        #region Methods

        public static byte BitArray2Byte(BitArray array)
        {
            byte ret = 0x00;
            for (byte i = 0; i < array.Count; i++)
            {
                ret |= (byte)((array[i] == true) ? (0x01 << i) : 0x00);
            }
            return ret;
        }

        public static byte[] AdjustBuffer(byte version, byte[] cmd)
        {
            if (version < 0x20)
            {
                return (byte[])cmd.Clone();
            }
            else
            {
                byte[] ret = new byte[cmd.Length + 2];
                cmd.CopyTo(ret, 2);
                return ret;
            }
        }

        public static byte[] AdjustBuffer(byte version, byte cmd)
        {
            if (version < 0x20)
            {
                return new byte[1] { cmd };
            }
            else
            {
                return new byte[3] { 0, 0, cmd };
            }
        }
        public static byte[] DoubleTemperatureToByteArray(byte version, double temp)
        {
            Int32 intTemp = (int)(temp * 10);
            intTemp = -AdjustTemperature(version, -intTemp);
            return new byte[] { (byte)intTemp, (byte)(intTemp >> 8) };
        }

        public static double ByteArrayToDoubleTemperature(byte version, byte[] tampArr)
        {
            Int32 temp = IsoThermHelper.ExtractTemperature(version, tampArr);
            temp = IsoThermHelper.AdjustTemperature(version, temp);
            return (double)(temp) / 10.0;
        }
        public static Int32 AdjustTemperature(byte version, Int32 Temperature)
        {
            if (version >= 0x55)
            {
                return Temperature; // 2's Complement - no need for any adjustment
            }
            if (version >= 0x40)
            {
                return Temperature -= 150;
            }
            else if (version >= 0x30)
            {
                return Temperature -= 100;
            }
            else if (version >= 0x25)
            {
                return Temperature += 200;
            }

            return Temperature;

        }

        public static Int32 ExtractTemperature(byte version, byte[] result)
        {
            if (version >= 0x55)
            {
                // 2's Complement
                return ((Int32)((sbyte)result[1]) << 8) + result[0];
            }
            return ((Int32)(result[1]) << 8) + result[0];
        }

        public static byte[] ShiftBuffer(byte version, byte[] buffer)
        {
            if (version < 0x20)
            {
                return buffer;
            }
            return new byte[] { buffer[1], buffer[2] };
        }
        public static void ValidateValueEquality(string valueName, byte valueToCheck, params byte[] valuesToComapre)
        {
            if (valuesToComapre.Length == 0)
            {
                return;
            }
            foreach (byte value in valuesToComapre)
            {
                if (valueToCheck == value)
                {
                    return;
                }
            }

            throw new ArgumentException(valueName + " value us invalid.");
        }

        public static void ValidateValueRange(string valueName, byte valueToCheck, byte lowLimit, byte highLimit)
        {
            if (valueToCheck < lowLimit || valueToCheck > highLimit)
            {
                throw new ArgumentException("Wrong " + valueName + " value (use values in 0x" + lowLimit.ToString("X") + " .. 0x" + highLimit.ToString("X") + " range)");
            }
        }

        public static void ValidateValueRange(string valueName, int valueToCheck, int lowLimit, int highLimit)
        {
            if (valueToCheck < lowLimit || valueToCheck > highLimit)
            {
                throw new ArgumentException("Wrong " + valueName + " value (use values in " + lowLimit.ToString() + " .. " + highLimit.ToString() + " range)");
            }
        }

        public static void ValidateValueRange(string valueName, double valueToCheck, double lowLimit, double highLimit)
        {
            if (valueToCheck < lowLimit || valueToCheck > highLimit)
            {
                throw new ArgumentException("Wrong " + valueName + " value (use values in " + lowLimit.ToString("F1") + " .. " + highLimit.ToString("F1") + " range)");
            }
        }

        public static byte ExtractValueFromRegister(byte reg, byte mask, byte shiftAmount)
        {
            return (byte)((reg & mask) >> shiftAmount);
        }

        public static byte InsertValueToRegister(byte reg, byte value, byte mask, byte shiftAmount)
        {
            return (byte)((reg & mask) | (value << shiftAmount));
        }

        public static string MakeRevisionString(byte value)
        {
            string revision = (((int)value & 0xF0) >> 4).ToString("X1") + "." + ((int)value & 0x0F).ToString("X1");
            return revision;
        }
        #endregion
    }

    #region ENUMs

    /// <summary>
    /// IsoTherm register set enumeration - read addresses.
    /// </summary>
    public enum IsoThermReadCommand : byte
    {
        GetVersion = 0x00, GetTemperatureTc = 0x01, GetTemperatureTj = 0x02, GetCurrent = 0x03, GetVoltage = 0x04, GetPower = 0x05, SetTemperature = 0x06,
        SlopProcHot = 0x07, GetSite1TempTc = 0x08, GetSite2TempTc = 0x09, GetSite1TempTj = 0x0A, GetSite2TempTj = 0x0B, PECIFrequency = 0x0C,
        AAOffset = 0x0D, PECIConfig = 0x0E, DiodesTAUStatus = 0x0F, HeatSink = 0x10, P = 0x11, I = 0x12, D = 0x13, PWRGain = 0x14, PWRPersistence = 0x15,
        TcCalibrationOffset = 0x16, TjCalibrationOffset = 0x17, DTSPollingCtrl = 0x18, DTSError = 0x19, TjSensorType = 0x20, TjSensorConfig = 0x21,
        TempCalibLsb = 0x22, TempCalibMsb = 0x23, SafetyDelayCount = 0x24, SafetyDelta = 0x25, IsoThermControl1 = 0x27, TcaseLowLimit = 0x28, TcaseWatchdogDelay = 0x29
    }

    /// <summary>
    /// IsoTherm register set enumeration - write addresses.
    /// </summary>
    public enum IsoThermWriteCommand : byte
    {
        DTSPolling = 0x01, Slop = 0x02, ProcHot = 0x03, PECIConfig = 0x04, PECIFrequency = 0x05, SetTemperature = 0x06, DTSControl = 0x07,
        AAOffset = 0x0E, ControlMode = 0x0F, HeatSink = 0x10, P = 0x11, I = 0x12, D = 0x13, PWRGain = 0x14, PWRPersistence = 0x15, TcCalibrationOffset = 0x16,
        TjCalibrationOffset = 0x17, DiodesTAUStatus = 0x1C, Reset = 0x1E, TjSensorConfig = 0x21, TempCalibLsb = 0x22, TempCalibMsb = 0x23, SafetyDelayCount = 0x24,
        SafetyDelta = 0x25, RestoreTjReading = 0x26, IsoThermControl1 = 0x27, TcaseLowLimit = 0x28, TcaseWatchdogDelay = 0x29
    }

    /// <summary>
    /// Enumeration for possible IsoTherm temperature modes.
    /// </summary>
    public enum IsoThermMode { Tcase, Tjunction }

    /// <summary>
    /// Enumeration for possible IsoTherm control modes.
    /// </summary>
    public enum ControlMode { Min, Max, Avarage }

    /// <summary>
    /// Enumeration for possible Tj sensor types
    /// </summary>
    public enum TjSensors : byte { EMC1403 = 0x01, LM95245 = 0x02, EMC1043 = 0x04 }
    /// <summary>
    /// Enumeration of heatsink code fields.
    /// </summary>
    public enum HeatSinkCode { PowerFollowEnable = 0, RTDAlphaFactor, FanControl, PWMDisable, PIDSelectCodeLCB, PIDSelectCodeMCB, TemperatureSense, PowerFollowingParam }

    /// <summary>
    /// Enumeration for PECI parameters.
    /// </summary>
    public enum PECIParameters { Slop = 0, ProcHot }

    /// <summary>
    /// Enumeration for PECI confign register fields.
    /// </summary>
    public enum PECIConfig { PECIAddrLsb = 0, PECIAddrMsb, HWDisablePECI, SWDisablePECI, TrigMode, PECIModeLsb, PECIModeMsb, IntVttReg }

    /// <summary>
    /// Enumeration for possible PECI module modes.
    /// </summary>
    public enum PECIMode { Listener = 0, Master, Translator }

    /// <summary>
    /// Enumeration for TAU config / Diodes status register fields.
    /// </summary>
    public enum DiodesTAUStatus { Diode1, Diode2, PECIError, AutoCal, RetModeLsb, RetModeMsb = 6, DiodePECISelect }

    /// <summary>
    /// Enumeration for Diode/PECI select bit options.
    /// </summary>
    public enum DiodePECISelect { Diode = 0, PECI }

    /// <summary>
    /// Enumeration for possible TAU temperature return modes.
    /// </summary>
    public enum RetMode { Max = 0, Min, Average, AutoAverageMax, AutoAverageMin }

    /// <summary>
    /// Enumeration for available die sensores.
    /// </summary>
    public enum CoreSelection { Core0, Core1, Core2, Core3, SA, GT, Max };

    /// <summary>
    /// Enumeration for IsoTherm features register fields (Address 0x00, bytes 3-4 on the IsoTherm register set).
    /// </summary>
    public enum Features
    {
        TempRangeLsb = 0, TempRangeMsb = 2, Safety = 4, TcOffsetBug, SWReset, BJTModel, TwoTJs, TwoTCs, TAUDetected, HeatPipe = 14, Control
    }
    /// <summary>
    /// Enumeration for Tj sensor config register fields.
    /// </summary>
    public enum TjSensorConfig { FilterSettingsLSB = 0, FilterSettingsMSB, ConvRateEMC1403LSB, ConvRateEMC1403MSB = 5, ConvRateLM95245LSB, ConvRateLM95245MSB }

    public enum SafetyCount { SafetyCountLSB = 0, SafetyCountMSB = 5, SafetyModeStatus = 7 }

    public enum Control1 { StandbyModeStatus = 0, TcaseLowLimitModeStatus = 1 };

    public enum RestoreTjRead { Restore = 0 };

    #endregion

    /// <summary>
    /// Exception indicating IsoTherm initialization error.
    /// </summary>
    public class IsoThermInitException : Exception
    {
        /// <summary>
        /// Constructor receiving message.
        /// </summary>
        /// <param name="str">Message to attach to exception.</param>
        public IsoThermInitException(string str) : base(str) { }
    }

    /// <summary>
    /// This class contains methods for communicating with an IsoTherm tool on an I2C bus.
    /// <para>Usage Note: When modifiying any TAU related settings, it is necessary to reset the IsoTherm for the cahnges to take effect. It is possible to make several TAU
    /// settings modifications followed by a single IsoTherm reset.</para>
    /// </summary>
    public class IsoThermLib : IDisposable
    {
        #region Private Fields

        private uint _handle;
        private string _devName = "UsbI2cIo";
        private const uint InvalidHandle = 0xffffffff;

        private byte mAddr;
        private byte mIndex;
        private byte mVersion;
        private string mVersionStr = "0.0";
        private bool mTAUDetected;
        private BitArray mFeaturesRegister = null;
        private TjSensors mSensorInUse;

        private uint mI2CPreTransactionDelayMs = 10;

        private const byte mTAU_DETECTED_MASK = 0x85;
        private const byte mTAU_DETECTED_NOT_MASK = 0x81;

        private const int mPECI_FREQ_DIVISOR = 10000;

        private const byte mFILTER_SETTINGS_MASK = 0x03;
        private const byte mCONV_RATE_EMC1403_MASK = 0x3C;
        private const byte mCONV_RATE_LM95245_MASK = 0xC0;
        private const byte mSAFETY_COUNT_MASK = 0x3F;
        private const byte mSAFETY_COUNT_NOT_MASK = 0xC0;

        #endregion

        #region Public Constants

        /// <summary>
        /// The maximum possible Slop value, see <see cref="SetSlop"/>.
        /// <para>Value = 0x1F</para>
        /// </summary>
        public const byte SLOP_HIGH_LIMIT = 0x1F;

        /// <summary>
        /// The maximum possible ProcHot value, see <see cref="SetProcHot"/>.
        /// <para>Value = 0x96</para>
        /// </summary>
        public const byte PROCHOT_HIGH_LIMIT = 0x96;

        /// <summary>
        /// The maximum possible PECI Frequency value, see <see cref="SetPECIFrequency"/>.
        /// <para>Value = 2000</para>
        /// </summary>
        public const int PECI_FREQ_HIGH_LIMIT = 2000;

        /// <summary>
        /// The minimum possible PECI Frequency value, see <see cref="SetPECIFrequency"/>.
        /// <para>Value = 2</para>
        /// </summary>
        public const int PECI_FREQ_LOW_LIMIT = 2;

        /// <summary>
        /// The maximum possible Auto-Average Offset value, see <see cref="SetAAOffset"/>.
        /// <para>Value = 0x3F</para>
        /// </summary>
        public const byte AAOFFSET_HIGH_LIMIT = 0x3F;

        /// <summary>
        /// The highest possible PECI Address, see <see cref="SetPECIAddr"/>.
        /// <para>Value = 0x03</para>
        /// </summary>
        public const byte PECI_ADDR_HIGH_LIMIT = 0x03;

        /// <summary>
        /// The highest possible filter setting for the EMC1403 / EMC1043 sensor, see <see cref="SetTjSensorConfig"/>
        /// <para>Value = 0x03</para>
        /// </summary>
        public const byte FILTER_EMC_HIGH_LIMIT = 0x03;

        /// <summary>
        /// Value for turning the filter on on the LM95245 sensor <see cref="SetTjSensorConfig"/>
        /// <para>Value = 0x03</para>
        /// </summary>
        public const byte FILTER_ON_LM95245 = 0x03;

        /// <summary>
        /// Value for turning the filter off on the LM95245 sensor <see cref="SetTjSensorConfig"/>
        /// <para>Value = 0x00</para>
        /// </summary>
        public const byte FILTER_OFF_LM95245 = 0x00;

        /// <summary>
        /// The highest possible conversion rate setting for the EMC1403 sensor, see <see cref="SetTjSensorConfig"/>
        /// <para>Value = 0x0F</para>
        /// </summary>
        public const byte CONV_RATE_EMC1403_HIGH_LIMIT = 0x0F;

        /// <summary>
        /// The highest possible conversion rate setting for the LM95245 sensor, see <see cref="SetTjSensorConfig"/>
        /// <para>Value = 0x03</para>
        /// </summary>
        public const byte CONV_RATE_LM95245_HIGH_LIMIT = 0x03;

        /// <summary>
        /// Number of storable calibration temperatures, see <see cref="SetCalibrationTemperature"/>
        /// <para>Value = 5</para>
        /// </summary>
        public const byte NUMBER_OF_CALIBRATION_TEMPERATURES = 5;

        /// <summary>
        /// The lowest possible calibration temperature, see <see cref="SetCalibrationTemperature"/>
        /// <para>Value = -34.0</para>
        /// </summary>
        public const double CALIBRATION_TEMPERATURE_LOW_LIMIT = -34.0;

        /// <summary>
        /// The highest possible calibration temperature, see <see cref="SetCalibrationTemperature"/>
        /// <para>Value = 145.0</para>
        /// </summary>
        public const double CALIBRATION_TEMPERATURE_HIGH_LIMIT = 145.0;

        /// <summary>
        /// The highest possible safety delay count (in seconds), see <see cref="SetCalibrationTemperature"/>
        /// <para>Value = 60</para>
        /// </summary>
        public const byte SAFETY_ACTIVATION_DELAY_HIGH_LIMIT = 60;

        /// <summary>
        /// The lowest possible safety delta (in degrees Celsius), see <see cref="SetCalibrationTemperature"/>
        /// <para>Value = 80.0</para>
        /// </summary>
        public const double SAFETY_DELTA_HIGH_LIMIT = 80.0;

        /// <summary>
        /// The highest possible Tcase watchdog delay (in seconds), see <see cref="SetTcaseWatchdogDelay"/>
        /// <para>Value = 255</para>
        /// </summary>
        public const byte TCASE_WATCHDOG_DELAY_HIGH_LIMIT = 255;

        #endregion

        #region Public Properties

        /// <summary>
        /// Gets the firmware version of the IsoTherm in use.
        /// </summary>
        public byte Version
        {
            get { return mVersion; }
        }

        /// <summary>
        /// A boolean indicating if a TAU is connected to the IsoTherm.
        /// </summary>
        public bool TAUDetected
        {
            get { return mTAUDetected; }
        }

        /// <summary>
        /// Gets the IsoTherm features register (Address 0x00, bytes 3-4 on the IsoTherm register set).
        /// </summary>
        public BitArray FeaturesRegister
        {
            get { return mFeaturesRegister; }
        }

        #endregion

        #region CTOR

        /// <summary>

        /// <param name="connector">An <see cref="II2cConnector"/> object which will be used to communicate with the IsoTherm.</param>
        /// <param name="address">Address of connected IsoTherm on the I2C bus.</param>
        /// <exception cref="IsoThermInitException">Thrown if initialization process fails.</exception>
        public IsoThermLib(byte address, byte index)
        {
            mAddr = address;
            mIndex = index;
            try
            {
                int devicesCount = DAPI_GetDeviceCount(_devName);
                if (devicesCount == 0)
                    throw new Exception("No thermal devices connected");

                _handle = DAPI_OpenDeviceInstance(_devName, mIndex);
                if (_handle == InvalidHandle)
                    throw new Exception("Failed to open device with address " + mAddr);

                mVersion = GetIsoThermVersion();
                if (mVersion == 0) throw new Exception("Failed to get IsoTherm version");
                if (mVersion >= 0x56)
                {
                    mSensorInUse = GetSensorInUse();
                }
                mVersionStr = IsoThermHelper.MakeRevisionString(mVersion);
            }
            catch (Exception e)
            {
                throw new IsoThermInitException(e.Message);
            }
        }

        #endregion

        #region Methods

        #region Rx/Tx Methods

        /// <summary>
        /// I2C pre-transaction delay.
        /// </summary>
        /// <param name="delayMs">Delay in milliseconds.</param>
        /// <exception cref="ArgumentException">Thrown if input is non-positive.</exception>
        public uint I2CTransactionDelay
        {
            get
            {
                return mI2CPreTransactionDelayMs;
            }
            set
            {
                mI2CPreTransactionDelayMs = value;
            }
        }

        /// <summary>
        /// Wrapper for <see cref="II2CProvider.WriteI2c"/> with a delay before the call (workaround for USB Hub connectivity issues).
        /// </summary>
        /// <param name="write_buf">Buffer to write.</param>
        private void WriteI2CDelayWrapper(byte[] write_buf)
        {
            Thread.Sleep((int)mI2CPreTransactionDelayMs);
            WriteI2c(write_buf);
        }

        /// <summary>
        /// Wrapper for <see cref="II2CProvider.ReadI2c"/> with a delay before the call (workaround for USB Hub connectivity issues).
        /// </summary>
        /// <param name="length">Expected length of response.</param>
        /// <returns>Array of bytes containing the response.</returns>
        private byte[] ReadI2CDelayWrapper(int length)
        {
            Thread.Sleep((int)mI2CPreTransactionDelayMs);
            return ReadI2c(length);
        }

        /// <summary>
        /// Wrapper for a write command and get result operation.
        /// </summary>
        /// <param name="command">An <see cref="IsoThermReadCommand"/> member indicating the register to be read.</param>
        /// <param name="length">Length of expected response.</param>
        /// <param name="values">Required parameters for the write command.</param>
        /// <returns>Array of bytes containing the response.</returns>
        private byte[] ExecuteCommandRes(IsoThermReadCommand command, int length, params byte[] values)
        {
            WriteCommand(command, values);
            return ReadCommand(length);
        }

        /// <summary>
        /// Write to an IsoTherm read register.
        /// </summary>
        /// <param name="command">An <see cref="IsoThermReadCommand"/> member indicating the register to be written.</param>
        /// <param name="values">The values to be written.</param>
        private void WriteCommand(IsoThermReadCommand command, params byte[] values)
        {
            byte[] temp = new byte[values.Length + 1];
            temp[0] = (byte)command;
            values.CopyTo(temp, 1);
            byte[] write_buf = IsoThermHelper.AdjustBuffer(mVersion, temp);
            WriteI2CDelayWrapper(write_buf);
        }

        /// <summary>
        /// Write to an IsoTherm write register.
        /// </summary>
        /// <param name="command">An <see cref="IsoThermReadCommand"/> member indicating the register to be written.</param>
        /// <param name="values">The values to be written.</param>
        private void WriteCommand(IsoThermWriteCommand command, params byte[] values)
        {
            byte[] temp = new byte[values.Length + 1];
            temp[0] = (byte)command;
            values.CopyTo(temp, 1);
            byte[] write_buf = IsoThermHelper.AdjustBuffer(mVersion, temp);
            WriteI2CDelayWrapper(write_buf);
        }

        /// <summary>
        /// Read from the IsoTherm.
        /// </summary>
        /// <param name="length">Expected length of response.</param>
        /// <returns>Array of bytes containing the response.</returns>
        private byte[] ReadCommand(int length)
        {
            byte[] read_buf = new byte[length];
            if (mVersion >= 0x20)
            {
                byte[] read_buf_aux = ReadI2CDelayWrapper(length + 1);

                Array.Copy(read_buf_aux, 1, read_buf, 0, length);
            }
            else
            {
                read_buf = ReadI2CDelayWrapper(length);
            }
            return read_buf;
        }

        #endregion

        #region Commands and Register Getters / Setters

        /// <summary>
        /// Reset the connected IsoTherm.
        /// </summary>
        public void Reset()
        {
            if ((mVersion < 0x23) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.SWReset]))
            {
                throw new NotSupportedException(string.Format("Reset is not implemented in IsoTherm version {0}", mVersionStr));
            }
            WriteCommand(IsoThermWriteCommand.Reset, 1);
        }

        /// <summary>
        /// Gets IsoTherm version and obtains the features register when relevant (version 5.5 onwards).
        /// </summary>
        /// <returns>IsoTherm version.</returns>
        public byte GetIsoThermVersion()
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetVersion, 5, 0, 0);

            if (read_buf[1] == 0xA1)
            {
                if (read_buf[2] >= 0x55)
                {
                    short featRegShort = (short)((read_buf[4] << 8) + read_buf[3]);
                    mFeaturesRegister = new BitArray(new byte[] { read_buf[3], read_buf[4] });
                    mTAUDetected = mFeaturesRegister[(int)Features.TAUDetected];
                }
                return read_buf[2];
            }
            return read_buf[1];
        }

        /// <summary>
        /// Sets IsoTherm default temperature measurement mode. IsoTherm will try get the temperature measured by specified method to the setpoint.
        /// </summary>
        public void SetTemperatureMode(IsoThermMode mode)
        {
            SetHeatsinkFlag(HeatSinkCode.TemperatureSense, mode == IsoThermMode.Tjunction);
        }

        public IsoThermMode GetTemperatureMode()
        {
            bool flag = GetHeatsinkFlag(HeatSinkCode.TemperatureSense);
            var mode = flag ? IsoThermMode.Tjunction : IsoThermMode.Tcase;
            return mode;
        }

        /// <summary>
        /// Sets the IsoTherm's control mode.
        /// </summary>
        /// <param name="mode">A <see cref="ControlMode"/> member indicating desired control mode.</param>
        public void SetControlMode(ControlMode mode)
        {
            WriteCommand(IsoThermWriteCommand.ControlMode);
            byte[] read_buf = ReadCommand(1);
            BitArray flags = new BitArray(new byte[] { read_buf[0] });
            flags[0] = (mode == ControlMode.Avarage);
            flags[1] = (mode != ControlMode.Min);
            WriteCommand(IsoThermWriteCommand.ControlMode, IsoThermHelper.BitArray2Byte(flags));
        }

        /// <summary>
        /// Set a specific heatsink code bit.
        /// </summary>
        /// <param name="flag">A <see cref="HeatSinkCode"/> bit indicating the bit to be modify.</param>
        /// <param name="enable">Boolean indicating whether to enable or dsiable the bit.</param>
        private void SetHeatsinkFlag(HeatSinkCode flag, bool enable)
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.HeatSink, 1);
            BitArray flags = new BitArray(new byte[] { read_buf[0] });
            //set the bit
            flags[(int)flag] = (enable);
            //store heatsink byte
            byte[] write_buf = new byte[] { (byte)IsoThermWriteCommand.HeatSink, IsoThermHelper.BitArray2Byte(flags) };
            write_buf = IsoThermHelper.AdjustBuffer(mVersion, write_buf);
            WriteI2CDelayWrapper(write_buf);
        }

        /// <summary>
        /// Returns a specific heatsink code bit.
        /// </summary>
        /// <param name="flag">A <see cref="HeatSinkCode"/> bit indicating the bit to be modify.</param>
        /// <rereturns>Boolean value of flag</rereturns>
        private bool GetHeatsinkFlag(HeatSinkCode flag)
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.HeatSink, 1);
            BitArray flags = new BitArray(new byte[] { read_buf[0] });
            return flags[(int)flag];
        }

        /// <summary>
        /// Get the heatsink code byte.
        /// </summary>
        /// <returns>A <see cref="BitArray"/> object containing the heatsink code byte.</returns>
        public BitArray GetHeatsink()
        {
            byte[] write_buf = IsoThermHelper.AdjustBuffer(mVersion, (byte)IsoThermReadCommand.HeatSink);
            WriteI2CDelayWrapper(write_buf);
            //byte[] read_buf = new byte[2];  //@ Cahnged from [1]
            BitArray flags = new BitArray(new byte[] { ReadI2CDelayWrapper(2)[1] });  //@ Changed from [0]

            return flags;
        }

        /// <summary>
        /// Set the heatsink code byte.
        /// </summary>
        /// <param name="flags"><see cref="BitArray"/> of the heatsink code.</param>
        public void SetHeatsink(BitArray flags)
        {
            WriteCommand(IsoThermWriteCommand.HeatSink, IsoThermHelper.BitArray2Byte(flags));
        }

        /// <summary>
        /// Disable pulse-width modulation (disables temperature control and turns on the fan).
        /// </summary>
        public void DisablePWM()
        {
            SetHeatsinkFlag(HeatSinkCode.PWMDisable, true);
        }

        /// <summary>
        /// Enable pulse-width modulation (enables temperature control).
        /// </summary>
        public void EnablePWM()
        {
            SetHeatsinkFlag(HeatSinkCode.PWMDisable, false);
        }

        /// <summary>
        /// Get device's current.
        /// </summary>
        /// <returns>Device's current.</returns>
        public double GetCurrent()
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetCurrent, 5);
            int value = IsoThermHelper.ExtractTemperature(mVersion, read_buf);
            //Current = Value * 0.098 Amps
            return (double)value * 0.098;
        }

        /// <summary>
        /// Get device's voltage.
        /// </summary>
        /// <returns>Device's voltage.</returns>
        public double GetVoltage()
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetVoltage, 5);
            int value = IsoThermHelper.ExtractTemperature(mVersion, read_buf);
            //Voltage = Value * 0.00244 Volts
            return (double)value * 0.00244;
        }

        /// <summary>
        /// Get device's power consumption.
        /// </summary>
        /// <returns>Device's power consumption.</returns>
        public double GetPower()
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetPower, 5);
            int value = IsoThermHelper.ExtractTemperature(mVersion, read_buf);
            //Power = Value * 0.244 Watts
            return (double)value * 0.244;
        }

        /// <summary>
        /// Get the case temperature.
        /// </summary>
        /// <returns>Current Tcase.</returns>
        public double GetTemperature()
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetTemperatureTc, 5);
            return IsoThermHelper.ByteArrayToDoubleTemperature(mVersion, read_buf);
        }

        /// <summary>
        /// Get the junction temperature. When using version 5.5 with PECI module enabled, this method returns the DTS temperature. Otherwise the thermal diode's temperature is returned.
        /// </summary>
        /// <returns>Current Tj.</returns>
        public double GetTemperatureJ()
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetTemperatureTj, 5);

            Int32 temp = IsoThermHelper.ExtractTemperature(mVersion, read_buf);
            temp = IsoThermHelper.AdjustTemperature(mVersion, temp);

            return (double)(temp) / 10.0;
        }

        /// <summary>
        /// Get the current setpoint temperature.
        /// </summary>
        /// <returns>Current setpoint temperature.</returns>
        public double GetSetPoint()
        {
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.SetTemperature, 2);

            Int32 temp = IsoThermHelper.ExtractTemperature(mVersion, read_buf);
            temp = IsoThermHelper.AdjustTemperature(mVersion, temp);
            return (double)(temp) / 10.0;
        }

        /// <summary>
        /// Gets site 1 Tcase (only for IsoTherm versions with 2 QCTCs connected).
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        /// <returns>Site 1 Tcase.</returns>
        public double GetSite1TcTemp()
        {
            if ((mVersion < 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.TwoTCs]))
            {
                throw new NotSupportedException(string.Format("GetSite1Tc is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetSite1TempTc, 2);
            Int32 temp = (Int32)(read_buf[1] << 8) + read_buf[0];
            temp = IsoThermHelper.AdjustTemperature(mVersion, temp);
            return (double)(temp) / 10.0;
        }

        /// <summary>
        /// Gets site 2 Tcase (only for IsoTherm versions with 2 QCTCs connected).
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        /// <returns>Site 2 Tcase.</returns>
        public double GetSite2TcTemp()
        {
            if ((mVersion < 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.TwoTCs]))
            {
                throw new NotSupportedException(string.Format("GetSite2Tc() is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetSite2TempTc, 2);
            Int32 temp = (Int32)(read_buf[1] << 8) + read_buf[0];
            temp = IsoThermHelper.AdjustTemperature(mVersion, temp);
            return (double)(temp) / 10.0;
        }

        /// <summary>
        /// Gets site 1 Tj (only for IsoTherm versions with 2 QCTCs connected).
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        /// <returns>Site 1 Tj.</returns>
        public double GetSite1TjTemp()
        {
            if ((mVersion < 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.TwoTJs]))
            {
                throw new NotSupportedException(string.Format("GetSite1Tj() is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetSite1TempTj, 2);
            Int32 temp = (Int32)(read_buf[1] << 8) + read_buf[0];
            temp = IsoThermHelper.AdjustTemperature(mVersion, temp);
            return (double)(temp) / 10.0;
        }

        /// <summary>
        /// Gets site 2 Tj (only for IsoTherm versions with 2 QCTCs connected).
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        /// <returns>Site 2 Tj.</returns>
        public double GetSite2TjTemp()
        {
            if ((mVersion < 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.TwoTJs]))
            {
                throw new NotSupportedException(string.Format("GetSite2Tj is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.GetSite2TempTj, 2);
            Int32 temp = (Int32)(read_buf[1] << 8) + read_buf[0];
            temp = IsoThermHelper.AdjustTemperature(mVersion, temp);
            return (double)(temp) / 10.0;
        }

        /// <summary>
        /// Enable / disable power following.
        /// </summary>
        /// <param name="isTrue">Boolean indicating wheter to enable or disable the feature.</param>
        public void SetPowerFollow(bool isTrue)
        {
            SetHeatsinkFlag(HeatSinkCode.PowerFollowEnable, isTrue);
        }

        /// <summary>
        /// Set the power following gain parameter.
        /// </summary>
        /// <param name="value">Value to apply.</param>
        public void SetPwrGain(byte value)
        {
            WriteCommand(IsoThermWriteCommand.PWRGain, value);
        }

        /// <summary>
        /// Get the power following gain parameter.
        /// </summary>
        /// <returns>Power following gain parameter.</returns>
        public byte GetPwrGain()
        {
            return ExecuteCommandRes(IsoThermReadCommand.PWRGain, 1)[0];
        }

        /// <summary>
        /// Set the power following persistence parameter.
        /// </summary>
        /// <param name="value">Value to apply.</param>
        public void SetPwrPersistence(byte value)
        {
            WriteCommand(IsoThermWriteCommand.PWRPersistence, value);
        }

        /// <summary>
        /// Set the power following persistence parameter.
        /// </summary>
        /// <returns>Power following persistence parameter.</returns>
        public byte GetPwrPersistance()
        {
            return ExecuteCommandRes(IsoThermReadCommand.PWRPersistence, 1)[0];
        }

        /// <summary>
        /// Get the calibration offset for either Tcase or Tj.
        /// </summary>
        /// <param name="mode"><see cref="IsoThermMode"/> indicating  whether to get Tcase or Tj offset.</param>
        /// <exception cref="NotSupportedException">Thrown if trying to obtain Tj offset on an IsoTherm which uses ideality and beta.</exception>
        /// <returns>The desired offset.</returns>
        public double GetCalibrationOffset(IsoThermMode mode)
        {
            if (mode == IsoThermMode.Tjunction)
            {
                if ((mVersion >= 0x50 && mVersion < 0x55) || (mVersion >= 0x55 && true == mFeaturesRegister[(int)Features.BJTModel]))
                {
                    throw new NotSupportedException(string.Format("GetCalibrationOffset for Tjunction measurement is not implemented in IsoTherm version {0}, use GetTjIdealityBeta", mVersionStr));
                }
            }
            IsoThermReadCommand cmd = (mode == IsoThermMode.Tcase) ? IsoThermReadCommand.TcCalibrationOffset : IsoThermReadCommand.TjCalibrationOffset;
            byte[] read_buf = ExecuteCommandRes(cmd, 2);

            if (cmd == IsoThermReadCommand.TcCalibrationOffset)
            {
                sbyte msb = (sbyte)read_buf[0];
                byte lsb = (byte)read_buf[1];
                // lsb is not relevant
                return (double)msb;
            }
            else
            {
                sbyte msb = (sbyte)read_buf[1];
                byte lsb = (byte)read_buf[0];
                return (double)(((int)msb << 4) | ((int)lsb >> 4)) / 16;
            }
        }

        /// <summary>
        /// Set the calibration offset for either Tcase or Tj.
        /// </summary>
        /// <param name="mode"><see cref="IsoThermMode"/> indicating  whether to get Tcase or Tj offset.</param>
        /// <param name="value">Offset value to be applied.</param>
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range.</exception>
        /// <exception cref="NotSupportedException">Thrown if trying to set Tj offset on an IsoTherm which uses ideality and beta.</exception>
        /// <returns>The desired offset.</returns>
        public void SetCalibrationOffset(IsoThermMode mode, double value)
        {
            IsoThermWriteCommand cmd = (mode == IsoThermMode.Tcase) ? IsoThermWriteCommand.TcCalibrationOffset : IsoThermWriteCommand.TjCalibrationOffset;
            if (mode == IsoThermMode.Tjunction)
            {
                if ((mVersion >= 0x50 && mVersion < 0x55) || (mVersion >= 0x55 && true == mFeaturesRegister[(int)Features.BJTModel]))
                {
                    throw new NotSupportedException(string.Format("GetCalibrationOffset for Tjunction measurement is not implemented in IsoTherm version {0}, use GetTjIdealityBeta", mVersionStr));
                }
            }
            if (value > 127 || value < -127)
            {
                throw new ArgumentException("Wrong calibration value (use values in -127..127 range)");
            }

            byte msb = 0x00;
            byte lsb = 0x00;

            if (cmd == IsoThermWriteCommand.TcCalibrationOffset)
            {
                // Tc calibartion uses only integer values between -127 and 127, in 2's complement.
                int intValue = (int)Math.Round(value);
                msb = (byte)intValue;
                if (intValue < 0)
                {
                    // For negative value, must set lsb to 0xFF. Otherwise - leave as 0x00.
                    lsb = 0xFF;
                }
            }
            else
            {
                // Tj calibration is uses the msb for the integer value and the lsb for the fraction value, with 0.125 precision.
                int just_value = (int)(value * 256);// shift value 8 bits left
                msb = (byte)(just_value & 0x00F0); // get the lst (last zero in anded value gives 0.125 precision)
                lsb = (byte)(just_value >> 8);
            }

            WriteCommand(cmd, msb, lsb);
        }

        /// <summary>
        /// Get ideality and beta settings.
        /// </summary>
        /// <param name="ideality">Byte object which will hold the obtained ideality factor.</param>
        /// <param name="beta">Byte object which will hold the obtained beta configuration.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        public void GetTjIdealityBeta(out byte ideality, out byte beta)
        {
            if ((mVersion <= 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.BJTModel]))
            {
                throw new NotSupportedException(string.Format("GetTjIdealityBeta is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.TjCalibrationOffset, 2);

            ideality = read_buf[0];
            beta = read_buf[1];
        }

        /// <summary>
        /// Get ideality factor.
        /// </summary>
        /// <returns>Ideality factor</returns>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        public byte GetTjIdeality()
        {
            if ((mVersion <= 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.BJTModel]))
            {
                throw new NotSupportedException(string.Format("GetTjIdeality is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.TjCalibrationOffset, 2);

            return read_buf[0];
        }

        /// <summary>
        /// Get Beta configuration.
        /// </summary>
        /// <returns>Beta configuration.</returns>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        public byte GetTjBeta()
        {
            if ((mVersion <= 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.BJTModel]))
            {
                throw new NotSupportedException(string.Format("GetTjBeta is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.TjCalibrationOffset, 2);

            return read_buf[1];
        }

        /// <summary>
        /// Set ideality and beta settings.
        /// <para>Note: When using TAU, IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="ideality">Ideality factor value to apply.</param>
        /// <param name="beta">Beta configuration value to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetTjIdealityBeta(byte ideality, byte beta)
        {
            if ((mVersion <= 0x50) || (mVersion >= 0x55 && false == mFeaturesRegister[(int)Features.BJTModel]))
            {
                throw new NotSupportedException(string.Format("SetTjIdealityBeta is not implemented in IsoTherm version {0}", mVersionStr));
            }
            WriteCommand(IsoThermWriteCommand.TjCalibrationOffset, ideality, beta);
        }

        /// <summary>
        /// Set the setpoint temperature.
        /// </summary>
        /// <param name="Value">Setpoint temperature to apply.</param>
        public void SetTemperature(double Value)
        {
            byte[] tempArr = IsoThermHelper.DoubleTemperatureToByteArray(mVersion, Value);
            WriteCommand(IsoThermWriteCommand.SetTemperature, tempArr[0], tempArr[1]);
        }

        /// <summary>
        /// Get Slop and ProcHot settings.
        /// </summary>
        /// <param name="slop">Byte object which will hold the obtained Slop value.</param>
        /// <param name="procHot">Byte object which will hold the obtained ProcHot value.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void GetSlopProcHot(out byte slop, out byte procHot)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetSlopProcHot is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] readBuf = ExecuteCommandRes(IsoThermReadCommand.SlopProcHot, 2);
            slop = readBuf[0];
            procHot = readBuf[1];
        }

        /// <summary>
        /// Get Slop setting.
        /// </summary>
        /// <returns>Slop setting</returns>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public byte GetSlop()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetSlop is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] readBuf = ExecuteCommandRes(IsoThermReadCommand.SlopProcHot, 2);
            return readBuf[0];
        }

        /// <summary>
        /// Get ProcHot setting.
        /// </summary>
        /// <returns>ProcHot setting</returns>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public byte GetProcHot()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetProcHot is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] readBuf = ExecuteCommandRes(IsoThermReadCommand.SlopProcHot, 2);
            return readBuf[1];
        }

        /// <summary>
        /// Set Slop value.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="slop">Slop value to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="SLOP_HIGH_LIMIT"/>).</exception>
        public void SetSlop(byte slop)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetSlop is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (slop > SLOP_HIGH_LIMIT)
            {
                throw new ArgumentException("Wrong Slop value (use values in 0x00..0x" + SLOP_HIGH_LIMIT.ToString("X") + " range)");
            }
            WriteCommand(IsoThermWriteCommand.Slop, slop);
        }

        /// <summary>
        /// Set ProcHot value.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="procHot">ProcHot value to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (<see cref="PROCHOT_HIGH_LIMIT"/>).</exception>
        public void SetProcHot(byte procHot)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetProcHot is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (procHot > PROCHOT_HIGH_LIMIT)
            {
                throw new ArgumentException("Wrong Slop value (use values in 0x00..0x" + PROCHOT_HIGH_LIMIT.ToString("X") + " range)");
            }
            WriteCommand(IsoThermWriteCommand.ProcHot, procHot);
        }

        /// <summary>
        /// Get PECI master mode frequency.
        /// </summary>
        /// <returns>PECI master mode frequency.</returns>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public double GetPECIFrequency()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetPECIFrequency is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] readBuf = ExecuteCommandRes(IsoThermReadCommand.PECIFrequency, 2); // readBuf[0] gets the LSB, readBuf[1] gets the MSB
            int freqIndex = IsoThermHelper.ExtractTemperature(mVersion, readBuf);
            return ((double)mPECI_FREQ_DIVISOR / (double)freqIndex);    // Formula is Frequency = Divisor / freqIndex
        }

        /// <summary>
        /// Set PECI master mode frequency.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <returns>Frequency value to apply, in KHz units.</returns>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="PECI_FREQ_HIGH_LIMIT"/>, <see cref="PECI_FREQ_LOW_LIMIT"/>).</exception>
        public void SetPECIFrequency(double freq)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetPECIFrequency is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if ((freq < PECI_FREQ_LOW_LIMIT) || (freq > PECI_FREQ_HIGH_LIMIT))
            {
                throw new ArgumentException("Wrong PECI Frequency value (use values in " + PECI_FREQ_LOW_LIMIT + "[KHz] .. " + PECI_FREQ_HIGH_LIMIT + "[KHz] range)");
            }
            int freqIndex = (int)System.Math.Round(mPECI_FREQ_DIVISOR / freq);   // Formula is freqIndex = Divisor / Frequency
            byte[] freqIndexArr = System.BitConverter.GetBytes(freqIndex);  // Little endian - freqInexArr[0] = LSB, freqIndexArr[1] = MSB
            WriteCommand(IsoThermWriteCommand.PECIFrequency, freqIndexArr[0], freqIndexArr[1]);
        }

        /// <summary>
        /// Get the auto-average offset (offset used in Auto-Average Max and Auto-Average Min temperature return modes - see <see cref="RetMode"/>).
        /// </summary>
        /// <returns>Auto-average offset.</returns>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public byte GetAAOffset()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetAAOffset is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] readBuf = ExecuteCommandRes(IsoThermReadCommand.AAOffset, 1);
            return readBuf[0];
        }

        /// <summary>
        /// Set the auto-average offset (offset used in Auto-Average Max and Auto-Average Min temperature return modes - see <see cref="RetMode"/>).
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="offset">Auto-average offset to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="AAOFFSET_HIGH_LIMIT"/>).</exception>
        public void SetAAOffset(byte offset)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetAAOffset is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (offset > AAOFFSET_HIGH_LIMIT)
            {
                throw new ArgumentException("Wrong Slop value (use values in 0x00..0x" + AAOFFSET_HIGH_LIMIT.ToString("X") + " range)");
            }
            WriteCommand(IsoThermWriteCommand.AAOffset, offset);
        }

        /// <summary>
        /// Obtain a register in bitwise fashion.
        /// </summary>
        /// <param name="regAddr">Register to read.</param>
        /// <returns>Desired register as a <see cref="BitArray"/> object.</returns>
        private BitArray GetBitWiseRegister(IsoThermReadCommand regAddr)
        {
            WriteCommand(regAddr);
            byte[] readBuf = ExecuteCommandRes(regAddr, 1);
            BitArray bwArr = new BitArray(new byte[] { readBuf[0] });
            return bwArr;
        }

        /// <summary>
        /// Set a bitwise register.
        /// </summary>
        /// <param name="regAddr">Register to be written.</param>
        /// <param name="bwArr">Data to be written.</param>
        private void SetBitWiseRegister(IsoThermWriteCommand regAddr, BitArray bwArr)
        {
            WriteCommand(regAddr, IsoThermHelper.BitArray2Byte(bwArr));
        }

        /// <summary>
        /// Get entire PECI Config register.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>PECI Config register.</returns>
        public byte GetPECIConfig()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetPECIConfig is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return ExecuteCommandRes(IsoThermReadCommand.PECIConfig, 1)[0];
        }

        /// <summary>
        /// Set the PECI config register.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="bwPECIConfig"><see cref="BitArray"/> object representing the values to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetPECIConfigByte(BitArray bwPECIConfig)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetPECIConfig is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetBitWiseRegister(IsoThermWriteCommand.PECIConfig, bwPECIConfig);
        }

        /// <summary>
        /// Set all parameters in the PECI Config register. See specific methods documentation for details:
        /// <see cref="EnableIntVttReg"/>, <see cref="DisableIntVttReg"/>, <see cref="EnablePECIWaitForTrigger"/>, <see cref="DisablePECIWaitForTrigger"/>,
        /// <see cref="EnablePECIModule"/>, <see cref="DisablePECIModule"/>, <see cref="SetPECIMode"/>, <see cref="SetPECIAddr"/>.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="peciAddr"></param>
        /// <param name="enablePeciModule"></param>
        /// <param name="enablePeciWaitForTrigger"></param>
        /// <param name="peciMode"></param>
        /// <param name="enableIntVttReg"></param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="PECI_ADDR_HIGH_LIMIT"/>).</exception>
        public void SetPECIConfig(byte peciAddr, bool enablePeciModule, bool enablePeciWaitForTrigger, PECIMode peciMode, bool enableIntVttReg)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetPECIConfigAll is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (peciAddr > PECI_ADDR_HIGH_LIMIT)
            {
                throw new ArgumentException("Wrong PECI Address value (use values in 0x00..0x" + PECI_ADDR_HIGH_LIMIT.ToString("X") + " range)");
            }
            BitArray bwPECIConfig = new BitArray(8, false);
            bwPECIConfig[(int)PECIConfig.SWDisablePECI] = enablePeciModule;
            bwPECIConfig[(int)PECIConfig.TrigMode] = enablePeciWaitForTrigger;
            bwPECIConfig[(int)PECIConfig.IntVttReg] = enableIntVttReg;
            BitArray encodedPeciMode = new BitArray(BitConverter.GetBytes((byte)peciMode));
            bwPECIConfig[(int)PECIConfig.PECIModeLsb] = encodedPeciMode[0];
            bwPECIConfig[(int)PECIConfig.PECIModeMsb] = encodedPeciMode[1];
            BitArray encodedAddr = new BitArray(new byte[] { peciAddr });
            bwPECIConfig[(int)PECIConfig.PECIAddrLsb] = encodedAddr[0];
            bwPECIConfig[(int)PECIConfig.PECIAddrMsb] = encodedAddr[1];
            SetPECIConfigByte(bwPECIConfig);
        }

        /// <summary>
        /// Set a specific bit in the PECIConfig register.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="flag">A <see cref="PECIConfig"/> member indicating which bit is being modified.</param>
        /// <param name="newSetting">Boolean indicating whether to enable or disable the bit.</param>
        private void SetPECIConfigFlag(PECIConfig flag, bool newSetting)
        {
            BitArray bwPECIConfig = GetBitWiseRegister(IsoThermReadCommand.PECIConfig);
            bwPECIConfig[(int)flag] = newSetting;
            SetPECIConfigByte(bwPECIConfig);
        }

        /// <summary>
        /// Enable the internal VTT reference voltage regulator.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void EnableIntVttReg()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("EnableIntVttReg is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetPECIConfigFlag(PECIConfig.IntVttReg, true);
        }

        /// <summary>
        /// Disable the internal VTT reference voltage regulator.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void DisableIntVttReg()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("DisableIntVttReg is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetPECIConfigFlag(PECIConfig.IntVttReg, false);
        }

        /// <summary>
        /// Enable PECI trigger mode (PECI module will wait for trigger to make a transaction).
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void EnablePECIWaitForTrigger()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("EnablePECIWaitForTrigger is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetPECIConfigFlag(PECIConfig.TrigMode, true);
        }

        /// <summary>
        /// Disable PECI trigger mode (PECI module will not wait for trigger to make a transaction).
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void DisablePECIWaitForTrigger()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("DisablePECIWaitForTrigger is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetPECIConfigFlag(PECIConfig.TrigMode, false);
        }

        /// <summary>
        /// Enable PECI module.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void EnablePECIModule()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("EnablePECI is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetPECIConfigFlag(PECIConfig.SWDisablePECI, true);
        }

        /// <summary>
        /// Disable PECI module.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void DisablePECIModule()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("DisablePECI is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetPECIConfigFlag(PECIConfig.SWDisablePECI, false);
        }

        /// <summary>
        /// Set the TAU's PECI module mode:
        /// <list type="bullet">
        ///     <item>
        ///         <term>Master Mode:</term>
        ///         <description> PECI module initiates PECI messages over the PECI bus at a predfined frequency (see <see cref="GetPECIFrequency"/>, 
        ///                      <see cref="SetPECIFrequency"/>) according to the DTS Polling Configuration (see <see cref="GetDTSPolling"/>, <see cref="SetDTSPolling(BitArray)"/>, 
        ///                      <see cref="SetDTSPolling(bool, bool, bool, bool, bool, bool)"/>, <see cref="SetDTSPollingCore"/>). Temperature data is obtained using 
        ///                      <see cref="GetTemperatureJ"/>.
        ///         </description>
        ///     </item>
        ///     <item>
        ///         <term>Listener Mode:</term>
        ///         <description> PECI module listens to transactions on th PECI bus and stores temperature data in the TAU. The data is retrieved in the same manner as
        ///                      in Master Mode.
        ///         </description>
        ///     </item>
        ///     <item>
        ///         <term>Translator Mode:</term>
        ///         <description> Irrelevant for TAU usage through IsoTherm. Note that setting this mode will automatically switch the TAU to diode temperature readout.</description>
        ///     </item>
        /// </list>
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="mode">A <see cref="PECIMode"/> member indicating the mode to be set.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        public void SetPECIMode(PECIMode mode)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetPECIMode is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray encodedMode = new BitArray(BitConverter.GetBytes((byte)mode));
            BitArray bwPECIConfig = GetBitWiseRegister(IsoThermReadCommand.PECIConfig);
            bwPECIConfig[(int)PECIConfig.PECIModeLsb] = encodedMode[0];
            bwPECIConfig[(int)PECIConfig.PECIModeMsb] = encodedMode[1];
            SetPECIConfigByte(bwPECIConfig);
        }

        /// <summary>
        /// Set the PECI address to monitor - 00 for address 0x30, 01 for address 0x31, etc.
        /// <para>Note: if only one PECI client is detected, it will be monitored automatically, regardless of any user set value.</para>
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="addr">PECI address to monitor.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="PECI_ADDR_HIGH_LIMIT"/>).</exception>
        public void SetPECIAddr(byte addr)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetPECIAddr is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (addr > PECI_ADDR_HIGH_LIMIT)
            {
                throw new ArgumentException("Wrong PECI Address value (use values in 0x00..0x" + PECI_ADDR_HIGH_LIMIT.ToString("X") + " range)");
            }
            BitArray encodedAddr = new BitArray(new byte[] { addr });
            BitArray bwPECIConfig = GetBitWiseRegister(IsoThermReadCommand.PECIConfig);
            bwPECIConfig[(int)PECIConfig.PECIAddrLsb] = encodedAddr[0];
            bwPECIConfig[(int)PECIConfig.PECIAddrMsb] = encodedAddr[1];
            SetPECIConfigByte(bwPECIConfig);
        }

        /// <summary>
        /// Get the TAU config and diodes status byte.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>TAU config and diodes status byte.</returns>
        public byte GetDiodesTAUStatus()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetDiodesTAUStatus is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return ExecuteCommandRes(IsoThermReadCommand.DiodesTAUStatus, 1)[0];
        }

        /// <summary>
        /// Set the TAU config and diodes status byte.
        /// <para>Note: The 3 LSBs will not be modified regardless of the input, as they hold diode and PECI status data.</para>
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="bwDiodesTAUStatus">Values to be apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDiodesTAUStatusByte(BitArray bwDiodesTAUStatus)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDiodesTAUStatus is not implemented in IsoTherm version {0}", mVersionStr));
            }

            // IsoTherm uses 3 lsbs of this byte for diode / PECI error flags, so we leave these bits untouched (i.e. = 0)
            for (int i = 0; i < 3; i++)
            {
                bwDiodesTAUStatus[i] = false;
            }
            byte test = IsoThermHelper.BitArray2Byte(bwDiodesTAUStatus);
            SetBitWiseRegister(IsoThermWriteCommand.DiodesTAUStatus, bwDiodesTAUStatus);
        }

        /// <summary>
        /// Set all parameters in the TAU config and diodes status byte. See specific methods documentation for details:
        /// <see cref="EnableAutoCalibration"/>, <see cref="DisableAutoCalibration"/>, <see cref="SetTAUTempSource"/>, <see cref="SetTAUTempReturnMode"/>.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="enableAutoCal"></param>
        /// <param name="tempReturnMode"></param>
        /// <param name="tempSource"></param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDiodesTAUStatus(bool enableAutoCal, RetMode tempReturnMode, DiodePECISelect tempSource)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDiodesTAUStatusAll is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray bwDiodesTAUStatus = new BitArray(8, false);
            bwDiodesTAUStatus[(int)DiodesTAUStatus.AutoCal] = enableAutoCal;
            bwDiodesTAUStatus[(int)DiodesTAUStatus.DiodePECISelect] = (tempSource == DiodePECISelect.Diode) ? false : true;
            BitArray encodedTempReturnMode = new BitArray(BitConverter.GetBytes((byte)tempReturnMode));
            for (int i = 0; i < 3; i++)
            {
                bwDiodesTAUStatus[(int)DiodesTAUStatus.RetModeLsb + i] = encodedTempReturnMode[i];
            }
            SetDiodesTAUStatusByte(bwDiodesTAUStatus);
        }

        /// <summary>
        /// Set a specific TAU config bit.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="flag">A <see cref="DiodesTAUStatus"/> member indicating which bit to modify.</param>
        /// <param name="newSetting">Boolean indicating whether to enable or disable the bit.</param>
        private void SetDiodesTAUStatusFlag(DiodesTAUStatus flag, bool newSetting)
        {
            BitArray bwDiodesTAUStatus = GetBitWiseRegister(IsoThermReadCommand.DiodesTAUStatus);
            bwDiodesTAUStatus[(int)flag] = newSetting;
            SetDiodesTAUStatusByte(bwDiodesTAUStatus);
        }

        /// <summary>
        /// Check for error on diode 1.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>If fault is detected on diode 1 - true. Otherwise - false.</returns>
        public bool GetDiode1Error()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetDiode1Error is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return GetBitWiseRegister(IsoThermReadCommand.DiodesTAUStatus)[(int)DiodesTAUStatus.Diode1];
        }

        /// <summary>
        /// Check for error on diode 2.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>If fault is detected on diode 2 - true. Otherwise - false.</returns>
        public bool GetDiode2Error()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetDiode2Error is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return GetBitWiseRegister(IsoThermReadCommand.DiodesTAUStatus)[(int)DiodesTAUStatus.Diode2];
        }

        /// <summary>
        /// Check for error on the PECI module.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>If fault is detected on the PECI module - true. Otherwise - false.</returns>
        public bool GetPECIError()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetPECIError is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return GetBitWiseRegister(IsoThermReadCommand.DiodesTAUStatus)[(int)DiodesTAUStatus.PECIError];
        }

        /// <summary>
        /// Enable an auto-calibration cycle. Used for non-fused units.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void EnableAutoCalibration()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("EnableAutoCalibration is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetDiodesTAUStatusFlag(DiodesTAUStatus.AutoCal, true);
        }

        /// <summary>
        /// Disable auto-calibration cycle. Used for non-fused units.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void DisableAutoCalibration()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("DisableAutoCalibration is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetDiodesTAUStatusFlag(DiodesTAUStatus.AutoCal, false);
        }

        /// <summary>
        /// Set the TAU's temperature readout source.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="source">A <see cref="DiodePECISelect"/> member indicating the temperature source to use.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetTAUTempSource(DiodePECISelect source)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetTAUTempSource is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (source == DiodePECISelect.Diode)
            {
                SetDiodesTAUStatusFlag(DiodesTAUStatus.DiodePECISelect, false);
            }
            else
            {
                SetDiodesTAUStatusFlag(DiodesTAUStatus.DiodePECISelect, true);
            }
        }

        /// <summary>
        /// Set the TAU's temperature return mode.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="mode">A <see cref="RetMode"/> member indicating the temperature source to use.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetTAUTempReturnMode(RetMode mode)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetTAUTempReturnMode is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray encodedMode = new BitArray(BitConverter.GetBytes((byte)mode));
            byte encodedMode_Byte = IsoThermHelper.BitArray2Byte(encodedMode);
            BitArray bwDiodesTAUStatus = GetBitWiseRegister(IsoThermReadCommand.DiodesTAUStatus);
            byte bwDiodesTAUStatus_Byte = IsoThermHelper.BitArray2Byte(bwDiodesTAUStatus);
            for (int i = 0; i < 3; i++)
            {
                bwDiodesTAUStatus[(int)DiodesTAUStatus.RetModeLsb + i] = encodedMode[i];
            }
            bwDiodesTAUStatus_Byte = IsoThermHelper.BitArray2Byte(bwDiodesTAUStatus);
            SetDiodesTAUStatusByte(bwDiodesTAUStatus);
        }

        /// <summary>
        /// Get the DTS control and polling bytes.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>Byte array containing the control (LSB) and polling (MSB) bytes.</returns>
        private byte[] GetDTSControlAndPolling()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetDTSControlAndPolling is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] readBuf = ExecuteCommandRes(IsoThermReadCommand.DTSPollingCtrl, 2);
            return readBuf;
        }

        /// <summary>
        /// Get the DTS control byte.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>DTS control byte.</returns>
        public byte GetDTSControl()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetDTSControl is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return GetDTSControlAndPolling()[0];
        }

        /// <summary>
        /// Set the DTS control byte, indicating which sensors on the die to will be used for temperature control.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="bwDTSControl"><see cref="BitArray"/> object representing the bitwise data to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDTSControlByte(BitArray bwDTSControl)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDTSControl is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetBitWiseRegister(IsoThermWriteCommand.DTSControl, bwDTSControl);
        }

        /// <summary>
        /// Set the DTS control byte, indicating which sensors on the die to will be used for temperature control.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="ctrlCore0">Boolean indicating whether to use Core 0 for control.</param>
        /// <param name="ctrlCore1">Boolean indicating whether to use Core 1 for control.</param>
        /// <param name="ctrlCore2">Boolean indicating whether to use Core 2 for control.</param>
        /// <param name="ctrlCore3">Boolean indicating whether to use Core 3 for control.</param>
        /// <param name="ctrlSA">Boolean indicating whether to use the System Agent for control.</param>
        /// <param name="ctrlGT">Boolean indicating whether to use the Graphics Engine for control.</param>
        /// <param name="ctrlMaxDieTemp">Boolean indicating whether to use the maximum die temperature for control.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDTSControl(bool ctrlCore0, bool ctrlCore1, bool ctrlCore2, bool ctrlCore3, bool ctrlSA, bool ctrlGT, bool ctrlMaxDieTemp)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDTSControl is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray bwDTSControl = new BitArray(new byte[] { GetDTSControlAndPolling()[0] });
            bwDTSControl[(int)CoreSelection.Core0] = ctrlCore0;
            bwDTSControl[(int)CoreSelection.Core1] = ctrlCore1;
            bwDTSControl[(int)CoreSelection.Core2] = ctrlCore2;
            bwDTSControl[(int)CoreSelection.Core3] = ctrlCore3;
            bwDTSControl[(int)CoreSelection.SA] = ctrlSA;
            bwDTSControl[(int)CoreSelection.GT] = ctrlGT;
            bwDTSControl[(int)CoreSelection.Max] = ctrlMaxDieTemp;
            SetDTSControlByte(bwDTSControl);
        }

        /// <summary>
        /// Enable / disable usage of a specific die sensor for temperature control.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="core"><see cref="CoreSelection"/> member indicating which sensor is being modified.</param>
        /// <param name="newSetting">Boolean indicating whether to use the sensor for control.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDTSControlCore(CoreSelection core, bool newSetting)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDTSControlCore is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray bwDTSControl = new BitArray(new byte[] { GetDTSControlAndPolling()[0] });
            bwDTSControl[(int)core] = newSetting;
            SetDTSControlByte(bwDTSControl);
        }

        /// <summary>
        /// Get the DTS polling byte.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>DTS control byte.</returns>
        public byte GetDTSPolling()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetDTSPolling is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return GetDTSControlAndPolling()[1];
        }

        /// <summary>
        /// Set the DTS polling byte, indicating which sensors on the die to will be used for temperature polling.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="bwDTSPolling"><see cref="BitArray"/> object representing the bitwise data to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDTSPollingByte(BitArray bwDTSPolling)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDTSPolling is not implemented in IsoTherm version {0}", mVersionStr));
            }
            SetBitWiseRegister(IsoThermWriteCommand.DTSPolling, bwDTSPolling);
        }

        /// <summary>
        /// Set the DTS control byte, indicating which sensors on the die to will be used for temperature polling.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="pollCore0">Boolean indicating whether to poll Core 0 temperature.</param>
        /// <param name="pollCore1">Boolean indicating whether to poll Core 1 temperature.</param>
        /// <param name="pollCore2">Boolean indicating whether to poll Core 2 temperature.</param>
        /// <param name="pollCore3">Boolean indicating whether to poll Core 3 temperature.</param>
        /// <param name="pollSA">Boolean indicating whether to poll the System Agent temperature.</param>
        /// <param name="pollGT">Boolean indicating whether to poll the Graphics Engine temperature.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDTSPolling(bool pollCore0, bool pollCore1, bool pollCore2, bool pollCore3, bool pollSA, bool pollGT)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDTSPolling is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray bwDTSPolling = new BitArray(new byte[] { GetDTSControlAndPolling()[1] });
            bwDTSPolling[(int)CoreSelection.Core0] = pollCore0;
            bwDTSPolling[(int)CoreSelection.Core1] = pollCore1;
            bwDTSPolling[(int)CoreSelection.Core2] = pollCore2;
            bwDTSPolling[(int)CoreSelection.Core3] = pollCore3;
            bwDTSPolling[(int)CoreSelection.SA] = pollSA;
            bwDTSPolling[(int)CoreSelection.GT] = pollGT;
            SetDTSPollingByte(bwDTSPolling);
        }

        /// <summary>
        /// Enable / disable polling the temperature of a specific die sensor.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="core"><see cref="CoreSelection"/> member indicating which sensor is being modified.</param>
        /// <param name="newSetting">Boolean indicating whether to poll the sensor.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void SetDTSPollingCore(CoreSelection core, bool newSetting)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("SetDTSPollingCore is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (core == CoreSelection.Max)
            {
                throw new ArgumentException("Maximum die temperature is polled by default. This setting cannot be changed.");
            }
            BitArray bwDTSPolling = new BitArray(new byte[] { GetDTSControlAndPolling()[1] });
            bwDTSPolling[(int)core] = newSetting;
            SetDTSPollingByte(bwDTSPolling);
        }

        /// <summary>
        /// Get the Tj sensor in use.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns><see cref="TjSensors"/> enumeration member indicating the sensor in use.</returns>
        public TjSensors GetSensorInUse()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetSensorInUse is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return (TjSensors)ExecuteCommandRes(IsoThermReadCommand.TjSensorType, 1)[0];
        }

        /// <summary>
        /// Get the string representation of the name of the Tj sensor in use.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>Name of used sensor.</returns>
        public string GetSensorInUseName()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetSensorInUseName is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return GetSensorInUse().ToString();
        }

        /// <summary>
        /// Get the entire Tj Sensor Config register.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns></returns>
        public byte GetTjSensorConfig()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetTjSensorConfig is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return ExecuteCommandRes(IsoThermReadCommand.TjSensorConfig, 1)[0];
        }

        /// <summary>
        /// Set the entire Tj Sensor Config register.
        /// <para>Note: IsoTherm reset is required for changes to take effect.</para>
        /// </summary>
        /// <param name="filter">Filter setting to apply.</param>
        /// <param name="convRateEMC1403">EMC1403 sensor conversion rate to apply.</param>
        /// <param name="convRateLM95245">LM95245 sensor conversion rate to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if any of the input values is out of allowed range 
        ///                                     (see <see cref="FILTER_ON_LM95245"/>, <see cref="FILTER_OFF_LM95245"/>, <see cref="FILTER_EMC_HIGH_LIMIT"/>, 
        ///                                     <see cref="CONV_RATE_EMC1403_HIGH_LIMIT"/>, <see cref="CONV_RATE_LM95245_HIGH_LIMIT"/>).</exception>
        public void SetTjSensorConfig(byte filter, byte convRateEMC1403, byte convRateLM95245)
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("SetTjSensorConfig is not implemented in IsoTherm version {0}", mVersionStr));
            }

            if (TjSensors.LM95245 == mSensorInUse)
            {
                IsoThermHelper.ValidateValueEquality("Filter Settings", filter, FILTER_ON_LM95245, FILTER_OFF_LM95245);
            }
            else
            {
                IsoThermHelper.ValidateValueRange("Filter Settings", filter, (byte)0, FILTER_EMC_HIGH_LIMIT);
            }
            IsoThermHelper.ValidateValueRange("EMC1403 Conversion Rate", convRateEMC1403, (byte)0, CONV_RATE_EMC1403_HIGH_LIMIT);
            IsoThermHelper.ValidateValueRange("LM95245 Conversion Rate", convRateLM95245, (byte)0, CONV_RATE_LM95245_HIGH_LIMIT);

            byte sensorConfig = (byte)(filter |
                                        (convRateEMC1403 << (int)TjSensorConfig.ConvRateEMC1403LSB) |
                                        (convRateLM95245 << (int)TjSensorConfig.ConvRateLM95245LSB));
            WriteCommand(IsoThermWriteCommand.TjSensorConfig, sensorConfig);
        }

        /// <summary>
        /// Get the current Tj sensor filter setting.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>The current filter setting.</returns>
        public byte GetTjSensorFilterSettings()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetTjSensorFilterSettings is not implemented in IsoTherm version {0}", mVersionStr));
            }

            return IsoThermHelper.ExtractValueFromRegister(ExecuteCommandRes(IsoThermReadCommand.TjSensorConfig, 1)[0],
                                                           mFILTER_SETTINGS_MASK,
                                                           (byte)TjSensorConfig.FilterSettingsLSB);
        }

        /// <summary>
        /// Get the current EMC1403 sensor conversion rate setting.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>The current EMC1403 sensor conversion rate setting.</returns>
        public byte GetConvRateEMC1403()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetConvRateEMC1403 is not implemented in IsoTherm version {0}", mVersionStr));
            }

            return IsoThermHelper.ExtractValueFromRegister(ExecuteCommandRes(IsoThermReadCommand.TjSensorConfig, 1)[0],
                                                           mCONV_RATE_EMC1403_MASK,
                                                           (byte)TjSensorConfig.ConvRateEMC1403LSB);
        }

        /// <summary>
        /// Get the current LM95245 sensor conversion rate setting.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>The current LM95245 sensor conversion rate setting.</returns>
        public byte GetConvRateLM95245()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetConvRateLM95245 is not implemented in IsoTherm version {0}", mVersionStr));
            }

            return IsoThermHelper.ExtractValueFromRegister(ExecuteCommandRes(IsoThermReadCommand.TjSensorConfig, 1)[0],
                                                           mCONV_RATE_LM95245_MASK,
                                                           (byte)TjSensorConfig.ConvRateLM95245LSB);
        }

        /// <summary>
        /// Get a calibration temperature.
        /// </summary>
        /// <param name="tempIdx">The index of the required temperature.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="NUMBER_OF_CALIBRATION_TEMPERATURES"/>).</exception>
        /// <returns>Calibration temperature.</returns>
        public double GetCalibrationTemperature(int tempIdx)
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetCalibrationTemperature is not implemented in IsoTherm version {0}", mVersionStr));
            }

            IsoThermHelper.ValidateValueRange("Calibration Temperature Index", tempIdx, 1, NUMBER_OF_CALIBRATION_TEMPERATURES);

            byte[] calibTempsLSBs = ExecuteCommandRes(IsoThermReadCommand.TempCalibLsb, NUMBER_OF_CALIBRATION_TEMPERATURES);
            byte[] calibTempsMSBs = ExecuteCommandRes(IsoThermReadCommand.TempCalibMsb, NUMBER_OF_CALIBRATION_TEMPERATURES);

            byte[] tempArr = new byte[] { calibTempsLSBs[tempIdx - 1], calibTempsMSBs[tempIdx - 1] };

            return IsoThermHelper.ByteArrayToDoubleTemperature(mVersion, tempArr);
        }

        /// <summary>
        /// Set a calibration temperature.
        /// </summary>
        /// <param name="tempIdx">The index of the temperature being set.</param>
        /// <param name="temp">Temperature value.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if any of the input values is out of allowed range (see 
        ///                                     <see cref="NUMBER_OF_CALIBRATION_TEMPERATURES"/>, 
        ///                                     <see cref="CALIBRATION_TEMPERATURE_LOW_LIMIT"/>, 
        ///                                     <see cref="CALIBRATION_TEMPERATURE_HIGH_LIMIT"/>).</exception>
        public void SetCalibrationTemperature(int tempIdx, double temp)
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("SetCalibrationTemperature is not implemented in IsoTherm version {0}", mVersionStr));
            }

            IsoThermHelper.ValidateValueRange("Calibration Temperature Index", tempIdx, 1, NUMBER_OF_CALIBRATION_TEMPERATURES);
            IsoThermHelper.ValidateValueRange("Calibration Temperature", temp, CALIBRATION_TEMPERATURE_LOW_LIMIT, CALIBRATION_TEMPERATURE_HIGH_LIMIT);

            byte[] tempArr = IsoThermHelper.DoubleTemperatureToByteArray(mVersion, temp);

            byte[] calibTempsLSBs = ExecuteCommandRes(IsoThermReadCommand.TempCalibLsb, NUMBER_OF_CALIBRATION_TEMPERATURES);
            byte[] calibTempsMSBs = ExecuteCommandRes(IsoThermReadCommand.TempCalibMsb, NUMBER_OF_CALIBRATION_TEMPERATURES);
            calibTempsLSBs[tempIdx - 1] = tempArr[0];
            calibTempsMSBs[tempIdx - 1] = tempArr[1];

            WriteCommand(IsoThermWriteCommand.TempCalibLsb, calibTempsLSBs);
            WriteCommand(IsoThermWriteCommand.TempCalibMsb, calibTempsMSBs);
        }

        /// <summary>
        /// Check if safety mode is on / off.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>Boolean indicating safety mode status.</returns>
        public bool IsSafetyModeOn()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("IsSafetyModeOn is not implemented in IsoTherm version {0}", mVersionStr));
            }

            BitArray safetyReg = GetBitWiseRegister(IsoThermReadCommand.SafetyDelayCount);
            return safetyReg[(int)SafetyCount.SafetyModeStatus];
        }

        /// <summary>
        /// Get the delay (in seconds) between the time the predefined delta between Tj and Tc is detected and the time safety mode is activated.
        /// See <see cref="SetSafetyActivationDelay"/> for more details.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>Safety mode activation delay (in seconds).</returns>
        public byte GetSafetyActivationDelay()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetSafetyCount is not implemented in IsoTherm version {0}", mVersionStr));
            }

            byte safetyReg = ExecuteCommandRes(IsoThermReadCommand.SafetyDelayCount, 1)[0];
            return IsoThermHelper.ExtractValueFromRegister(safetyReg, mSAFETY_COUNT_MASK, (byte)SafetyCount.SafetyCountLSB);
        }

        /// <summary>
        /// Set the delay (in seconds) between the time the predefined delta between Tj and Tc is detected (see <see cref="SetSafetyDelta"/>) 
        /// and the time safety mode is activated. This value is considered only when Tj > Tc. If Tc > Tj, the delay is approximately 5 seconds (un-modifiable value).
        /// </summary>
        /// <param name="delay">Delay value (in seconds) to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="SAFETY_ACTIVATION_DELAY_HIGH_LIMIT"/>).</exception>
        public void SetSafetyActivationDelay(byte delay)
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("SetSafetyCount is not implemented in IsoTherm version {0}", mVersionStr));
            }

            IsoThermHelper.ValidateValueRange("Safety Count", (int)delay, 0, SAFETY_ACTIVATION_DELAY_HIGH_LIMIT);
            byte safetyReg = ExecuteCommandRes(IsoThermReadCommand.SafetyDelayCount, 1)[0];
            WriteCommand(IsoThermWriteCommand.SafetyDelayCount, IsoThermHelper.InsertValueToRegister(safetyReg, delay, mSAFETY_COUNT_NOT_MASK, (byte)SafetyCount.SafetyCountLSB));
        }

        /// <summary>
        /// Get the temperature delta between Tj and Tc which will cause safety mode to be activated. See <see cref="SetSafetyDelta"/> for more details.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>The safety mode delta (in degrees Celsius).</returns>
        public double GetSafetyDelta()
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("GetSafetyDelta is not implemented in IsoTherm version {0}", mVersionStr));
            }

            byte[] deltaArr = ExecuteCommandRes(IsoThermReadCommand.SafetyDelta, 2);

            return IsoThermHelper.ByteArrayToDoubleTemperature(mVersion, deltaArr);
        }

        /// <summary>
        /// Get the temperature delta between Tj and Tc which will cause safety mode to be activated (after the predefined delay
        ///  - see <see cref="SetSafetyActivationDelay"/>). This value is considered only if the IsoTherm is in Tj Control mode, 
        ///  and only if Tj > Tc. If Tc > Tj, the safety delta value is 60 degrees Celsius (un-modifiable value).
        /// <para>
        /// Effects of safety mode:
        ///     <list type="bullet">
        ///         <item>
        ///             <description>"SAFE" message appears on Therm1 7-Segment dispaly.</description>
        ///         </item>
        ///         <item>
        ///             <description>Set point is changed to 35 degrees Celsius.</description>
        ///         </item>
        ///         <item>
        ///             <description>PWM signal is shut off (after the predefined delay from the moment the "SAFE" message appeared has passed).</description>
        ///         </item>
        ///     </list>
        /// </para>
        /// </summary>
        /// <param name="delta">Delta value (in degrees Celsius) to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception>
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="SAFETY_DELTA_HIGH_LIMIT"/>).</exception>
        public void SetSafetyDelta(double delta)
        {
            if (mVersion < 0x56)
            {
                throw new NotSupportedException(string.Format("SetSafetyDelta is not implemented in IsoTherm version {0}", mVersionStr));
            }

            IsoThermHelper.ValidateValueRange("Safety Delta", delta, 0, SAFETY_DELTA_HIGH_LIMIT);

            byte[] deltaArr = IsoThermHelper.DoubleTemperatureToByteArray(mVersion, delta);

            WriteCommand(IsoThermWriteCommand.SafetyDelta, deltaArr[0], deltaArr[1]);
        }

        /// <summary>
        /// Check if standby mode is on / off.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>Boolean indicating standby mode status.</returns>
        public bool IsStandbyModeOn()
        {
            if (mVersion < 0x57)
            {
                throw new NotSupportedException(string.Format("IsStandbyModeOn is not implemented in IsoTherm version {0}", mVersionStr));
            }

            BitArray stbyReg = GetBitWiseRegister(IsoThermReadCommand.IsoThermControl1);
            return stbyReg[(int)Control1.StandbyModeStatus];
        }

        /// <summary>
        /// Set standby mode on / off.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <param name="newSetting">Boolean indicating whether to enable or disable standby mode.</param>
        public void SetStandbyMode(bool newSetting)
        {
            if (mVersion < 0x57)
            {
                throw new NotSupportedException(string.Format("SetStandbyModeOn is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray bwStbyReg = GetBitWiseRegister(IsoThermReadCommand.IsoThermControl1);
            bwStbyReg[(int)Control1.StandbyModeStatus] = newSetting;
            SetBitWiseRegister(IsoThermWriteCommand.IsoThermControl1, bwStbyReg);
        }

        /// <summary>
        /// If during the IsoTherm's run Tj reading has stopped (for example if the diode was disconnected), then the Tj reading can be
        /// restored without an IsoTherm reset by calling this method.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        public void RestoreTjReading()
        {
            if (mVersion < 0x57)
            {
                throw new NotSupportedException(string.Format("RestoreTjReading is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray bwRestoreTjReg = new BitArray(new byte[] { 0x01 });
            SetBitWiseRegister(IsoThermWriteCommand.RestoreTjReading, bwRestoreTjReg);
        }

        /// <summary>
        /// Check if Tcase low limit is enabled.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>Boolean indicating Tcase low limit mode status.</returns>
        public bool IsTcaseLowLimitModeOn()
        {
            if (mVersion < 0x58)
            {
                throw new NotSupportedException(string.Format("IsTcaseLowLimitOn is not implemented in IsoTherm version {0}", mVersionStr));
            }

            BitArray stbyReg = GetBitWiseRegister(IsoThermReadCommand.IsoThermControl1);
            return stbyReg[(int)Control1.TcaseLowLimitModeStatus];
        }

        /// <summary>
        /// Set Tcase low limit mode on / off.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <param name="newSetting">Boolean indicating whether to enable or disable Tcase low limit mode.</param>
        public void SetTcaseLowLimitMode(bool newSetting)
        {
            if (mVersion < 0x58)
            {
                throw new NotSupportedException(string.Format("SetTcaseLowLimitMode is not implemented in IsoTherm version {0}", mVersionStr));
            }
            BitArray bwStbyReg = GetBitWiseRegister(IsoThermReadCommand.IsoThermControl1);
            bwStbyReg[(int)Control1.TcaseLowLimitModeStatus] = newSetting;
            SetBitWiseRegister(IsoThermWriteCommand.IsoThermControl1, bwStbyReg);

        }

        /// <summary>
        /// Get the Tcase low limit temperature value.
        /// </summary>
        /// <returns>Current Tcase low limit temperature value.</returns>
        public double GetTcaseLowLimitValue()
        {
            if (mVersion < 0x58)
            {
                throw new NotSupportedException(string.Format("GetTcaseLowLimitValue is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] read_buf = ExecuteCommandRes(IsoThermReadCommand.TcaseLowLimit, 2);

            Int32 temp = IsoThermHelper.ExtractTemperature(mVersion, read_buf);
            temp = IsoThermHelper.AdjustTemperature(mVersion, temp);
            return (double)(temp) / 10.0;
        }

        /// <summary>
        /// Set the Tcase low limit temperature value.
        /// </summary>
        /// <param name="Value">Temperature value to apply.</param>
        public void SetTcaseLowLimitValue(double Value)
        {
            if (mVersion < 0x58)
            {
                throw new NotSupportedException(string.Format("SetTcaseLowLimitValue is not implemented in IsoTherm version {0}", mVersionStr));
            }
            byte[] tempArr = IsoThermHelper.DoubleTemperatureToByteArray(mVersion, Value);
            WriteCommand(IsoThermWriteCommand.TcaseLowLimit, tempArr[0], tempArr[1]);
        }

        /// <summary>
        /// Get the Tcase watchdog delay (in seconds). See <see cref="SetTcaseWatchdogDelay"/> for more details.
        /// </summary>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <returns>Tcase watchdog delay (in seconds).</returns>
        public byte GetTcaseWatchdogDelay()
        {
            if (mVersion < 0x58)
            {
                throw new NotSupportedException(string.Format("GetTcaseWatchdogDelay is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return ExecuteCommandRes(IsoThermReadCommand.TcaseWatchdogDelay, 1)[0];
        }

        /// <summary>
        /// Set the Tcase watchdog delay (in seconds). After the safety activation delay expires, the set point is automaically set to 35 degrees. From this point the
        /// watchdog is activated. When the watchdog delay expires, if the temperature is not betweeen 15 to 50 degrees, the PWM is shut off.
        /// </summary>
        /// <param name="delay">Delay value (in seconds) to apply.</param>
        /// <exception cref="NotSupportedException">Thrown if operation not supported by the connected IsoTherm.</exception> 
        /// <exception cref="ArgumentException">Thrown if input is out of allowed range (see <see cref="TCASE_WATCHDOG_DELAY_HIGH_LIMIT"/>).</exception>
        public void SetTcaseWatchdogDelay(byte delay)
        {
            if (mVersion < 0x58)
            {
                throw new NotSupportedException(string.Format("SetTcaseWatchdogDelay is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (delay > TCASE_WATCHDOG_DELAY_HIGH_LIMIT)
            {
                throw new ArgumentException("Wrong Tcase Watchodg Delay value (use values in 0.." + TCASE_WATCHDOG_DELAY_HIGH_LIMIT + " range)");
            }
            WriteCommand(IsoThermWriteCommand.TcaseWatchdogDelay, delay);
        }

        /* DTS Error functionality not yet implemented
        public byte GetDTSError()
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("GetDTSError is not implemented in IsoTherm version {0}", mVersionStr));
            }
            return ExecuteCommandRes(IsoThermReadCommand.DTSError, 1)[0]; 
        }

        public bool CheckDTSCoreError(CoreSelection core)
        {
            if (mVersion < 0x55)
            {
                throw new NotSupportedException(string.Format("CheckDTSCoreError is not implemented in IsoTherm version {0}", mVersionStr));
            }
            if (core == CoreSelection.Max)
            {
                throw new ArgumentException("Choose a specific core");
            }
            BitArray bwDTSError = GetBitWiseRegister(IsoThermReadCommand.DTSError);
            return bwDTSError[core];
        }
        */

        #endregion

        #endregion

        #region Low Level usbi2cio wrappers

        [DllImport("usbi2cio.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int DAPI_GetDllVersion();

        [DllImport("usbi2cio.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int DAPI_GetDriverVersion();

        [DllImport("usbi2cio.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern byte DAPI_GetDeviceCount([MarshalAs(UnmanagedType.LPTStr)] string devName);

        [DllImport("usbi2cio.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern uint DAPI_OpenDeviceInstance([MarshalAs(UnmanagedType.LPTStr)] string devName, byte devNumber);

        [DllImport("usbi2cio.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern bool DAPI_CloseDeviceInstance(uint handle);

        [DllImport("usbi2cio.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int DAPI_ReadI2c(uint handle, ref I2CStructure i2CStructure);

        [DllImport("usbi2cio.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern int DAPI_WriteI2c(uint handle, ref I2CStructure i2CStructure);

        public void ReleaseDevice()
        {
            try
            {
                DAPI_CloseDeviceInstance(_handle);
            }
            catch
            {
            }
        }

        private void WriteI2c(byte[] write_buf)
        {
            I2CStructure sw = new I2CStructure(mAddr, (ushort)write_buf.Length);
            write_buf.CopyTo(sw.Data, 0);
            int bytesWritten = DAPI_WriteI2c(_handle, ref sw);
            if (bytesWritten != write_buf.Length)
            {
                throw new Exception("Isotherm lib failed to write I2C, bytes written should be " + write_buf.Length + " but written " + bytesWritten);
            }
        }

        private byte[] ReadI2c(int length)
        {
            byte[] result = new byte[length];

            I2CStructure sr = new I2CStructure(mAddr, (byte)length);
            int bytesRead = DAPI_ReadI2c(_handle, ref sr);
            if (bytesRead != length)
            {
                throw new Exception("Isotherm lib failed to read I2C, bytes read should be " + length + " but written " + bytesRead);
            }

            Array.Copy(sr.Data, result, length);
            return result;
        }

        #endregion

        #region IDisposable Members

        private void Dispose(bool disposing)
        {
            if (disposing)
            {
                GC.SuppressFinalize(this);
            }

            ReleaseDevice();
        }

        public void Dispose()
        {
            Dispose(true);
        }

        ~IsoThermLib()
        {
            Dispose(false);
        }
        #endregion
    }
}