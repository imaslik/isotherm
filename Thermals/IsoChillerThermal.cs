using System.Reflection;
using SysCommand.ConsoleApp;

namespace IsoTherm.Thermals
{
    public class IsoChillerThermal : Thermal
    {
        private int _usbAddress;
        private IsoThermLib _isoThermLib;       
        private uint _i2cDelay;      

        public IsoChillerThermal(App currentApp) : base(currentApp)
        {
            _minTemperature = -5;
            _maxTemperature = 125;
        }

        public void Initialize(out string errorMessage)
        {
            errorMessage = null;

            // UsbConnection usbConnection = _configuration.Connection as UsbConnection;
            // if (usbConnection == null)
            // {
            //     errorMessage = "Wrong connection type, must be USB connection";
            //     return;
            // }

            _usbAddress = 128;//Helpers.ConvertIntAsHex(50);
            _i2cDelay = 1;

            try
            {
                _isoThermLib = new IsoThermLib((byte)_usbAddress, (byte)ThermalIndex) { I2CTransactionDelay = _i2cDelay };
                int resetCounter = 0;
                while (resetCounter <= 10)
                {
                    try
                    {
                        CheckPwmFlag();
                        int version = _isoThermLib.GetIsoThermVersion();
                        if (version == 0)
                        {
                            errorMessage = "IsoTherm device not found";
                            return;
                        }

                        _isoThermLib.SetTemperatureMode(IsoThermMode.Tcase);
                        return;
                    }
                    catch (Exception ex)
                    {
                        resetCounter++;
                        if (resetCounter == 10)
                        {
                            errorMessage = "Failed to init IsoTherm device, error: " + ex.Message;
                            return;
                        }
                    }
                    Thread.Sleep(11 * 1000);
                }
            }
            catch (Exception ex)
            {
                errorMessage = ex.Message;
            }
        }

        public void Release()
        {
            _isoThermLib.ReleaseDevice();
        }

        public override double GetTemperature()
        {
            
            _currentApp.Console.Write("Getting temperature");

            lock (_locker)
            {                
                for (int i = 0; i < 10; i++)
                {
                    try
                    {                        
                        double temp = _isoThermLib.GetTemperature();
                        _currentApp.Console.Write("Temperature read " + temp);
                        return temp;
                    }
                    catch (Exception ex)
                    {
                        _currentApp.Console.Write("Failed to read temperature, error: " + ex.Message);
                        if (i == 9)
                            throw;

                        string error;
                        Release();
                        Thread.Sleep(100);
                        Initialize(out error);
                    }
                }
            }

            string message = string.Format("IsoTherm: Failed to read temperature after reset");
            // AddLogMessage(message, LogKind.Error);
            throw new Exception(message);
        }

        protected override void SetTemperatureReal(double temperature)
        {
            _currentApp.Console.Write("Setting real temperature to " + temperature);

            if (temperature < MinTemperature || temperature > MaxTemperature)
                return; // Throw exception???

            lock (_locker)
            {
                for (int i = 0; i < 10; i++)
                {
                    try
                    {
                        _isoThermLib.SetTemperature(temperature);
                    }
                    catch (Exception ex)
                    {
                        _currentApp.Console.Write("Failed to set temperature, error: " + ex.Message);
                        if (i == 9)
                            throw;

                        string error;
                        Release();
                        Thread.Sleep(100);
                        Initialize(out error);
                    }
                }
            }

            _targetTemperature = temperature;
        }       

        private void CheckPwmFlag()
        {
            try
            {
                MethodInfo mInfoExecCmdRes = _isoThermLib.GetType().GetMethod("ExecuteCommandRes", BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.OptionalParamBinding);
                MethodInfo mInfoSetHeatsinkFlag = _isoThermLib.GetType().GetMethod("SetHeatsinkFlag", BindingFlags.NonPublic | BindingFlags.Instance);
                if (mInfoExecCmdRes != null && mInfoSetHeatsinkFlag != null)
                {
                    byte[] readBuf = (byte[])mInfoExecCmdRes.Invoke(_isoThermLib, new object[] { IsoThermReadCommand.HeatSink, 1, new byte[] { } }); // Read array of codes
                    System.Collections.BitArray flags = new System.Collections.BitArray(new byte[] { readBuf[0] });
                    bool pwmDisabledState = flags[(int)HeatSinkCode.PWMDisable]; // Retrieve needed code state
                    if (pwmDisabledState) // PWS is disabled
                    {
                        string baseMsg = "PWM detected as Disabled during initialization - ";
                        mInfoSetHeatsinkFlag.Invoke(_isoThermLib, new object[] { HeatSinkCode.PWMDisable, false }); // Set to enabled ('disabled' to false)
                        readBuf = (byte[])mInfoExecCmdRes.Invoke(_isoThermLib, new object[] { IsoThermReadCommand.HeatSink, 1, new byte[] { } }); // Re-read array of codes
                        flags = new System.Collections.BitArray(new byte[] { readBuf[0] });
                        pwmDisabledState = flags[(int)HeatSinkCode.PWMDisable]; // Re-retrieve needed code state for re-checking
                                                                                //if (pwmDisabledState) // Forcing failed
                                                                                //logger.Log(baseMsg + "failed to set Enabled state", LogLevel.Debug, LogType.Warning);

                    }
                }
                else
                    throw new Exception("IsoThermLib changed");
            }
            catch (Exception ex)
            {
                //logger.Log("Can't check PWM flag - " + ex.Message, LogLevel.Debug, LogType.Warning);
            }
        }

    }
}