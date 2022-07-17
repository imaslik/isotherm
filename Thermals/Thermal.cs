using SysCommand.ConsoleApp;

namespace IsoTherm.Thermals
{
    public abstract class Thermal
    {
        protected double _targetTemperature;
        protected readonly object _locker = new object();

        // Everey inheriting class should set these values!
        protected double _minTemperature;
        protected double _maxTemperature;

        protected App _currentApp;
        // protected static ThermalFeedbackType _temperatureSource = ThermalFeedbackType.Tc;

        public Thermal(App currentApp) => _currentApp = currentApp;

        public virtual void SetTemperature(double temperature)
        {
            // AddLogMessage("Setting temperature " + temperature, LogKind.SystemDebug);
            SetTemperatureReal(temperature);
            WaitForTemperature(temperature,() => GetTemperature());
        }

        protected void WaitForTemperature(double temperature, Func<double> getTemperatureDel)
        {
            int timeToWait = 1000;
            int waited = 0;
            double currentTemperature;
            bool tempReachedTarget = false;
            bool tempStable = false;
            int state = 0;
            _currentApp.Console.Write(ThermalAccuracy);
            Thread.Sleep(timeToWait);

            while (state < 2)
            {
                if (state == 0)
                {
                    _currentApp.Console.Write("Waiting for temperature to reach target temperature " + temperature);
                    currentTemperature = getTemperatureDel();
                    while (true)
                    {
                        Thread.Sleep(timeToWait);
                        currentTemperature = getTemperatureDel();
                        tempReachedTarget = Math.Round(Math.Abs(currentTemperature - temperature), 1) <= ThermalAccuracy;
                        if (tempReachedTarget)
                            break;
                    }
                    state = 1;
                }
                else if (state == 1)
                {
                    waited = 0;
                    _currentApp.Console.Write("Waiting for temperature to stabilize");
                    while (true)
                    {
                        Thread.Sleep(timeToWait);
                        waited += timeToWait;
                        currentTemperature = getTemperatureDel();
                        tempStable = Math.Round(Math.Abs(currentTemperature - temperature), 1) <= ThermalAccuracy;

                        if (!tempStable)
                        {
                            state = 0; // back to waiting for reaching target temperature
                            break;
                        }

                        if (waited >= ThermalStabilizationTime)
                        {
                            state = 2;
                            break;
                        }
                    }
                }
            }

            // AddLogMessage("Reached target temperature " + temperature, LogKind.SystemDebug);
        }


        public virtual void SetTemperatureAsync(double temperature)
        {
            // AddLogMessage("Setting async temperature " + temperature, LogKind.SystemDebug);
            SetTemperatureReal(temperature);
        }

        public abstract double GetTemperature();
        protected abstract void SetTemperatureReal(double temperature);

        // [TextSpecificParameter("Thermal Index", false)]
        public string ThermalIndexWrapper
        {
            get
            {
                return ThermalIndex.ToString();
            }
            set
            {
                uint index;
                if (!uint.TryParse(value, out index))
                    throw new Exception("Thermal Index value must be positive number");

                ThermalIndex = index;
            }
        }

        // [TextSpecificParameter("Thermal Accuracy", false)]
        public string ThermalAccuracyWrapper
        {
            get
            {
                return ThermalAccuracy.ToString();
            }
            set
            {
                double accuracy;
                if (!double.TryParse(value, out accuracy))
                    throw new Exception("Accuracy value must be number");

                if (accuracy < 0)
                    throw new Exception("Accuracy value must be positive");

                ThermalAccuracy = accuracy;
            }
        }

        // [TextSpecificParameter("Stabilization Time [s]", false)]
        public string ThermalStabilizationWrapper
        {
            get
            {
                return ThermalStabilizationTime.ToString();
            }
            set
            {
                uint time;
                if (!uint.TryParse(value, out time))
                    throw new Exception("Stabilization time value must be positive number");

                ThermalStabilizationTime = time * 1000;
            }
        }

        // [TextSpecificParameter("DMM", true)]
        public uint ThermalIndex { get; private set; }
        public double ThermalAccuracy { get; private set; }
        public uint ThermalStabilizationTime { get; private set; }

        public double MinTemperature
        {
            get { return _minTemperature; }
        }
        public double MaxTemperature
        {
            get { return _maxTemperature; }
        }

        // virtual public bool FeedbackLoopRequired
        // {
        //     get { return true; }
        // }
        
    }
}