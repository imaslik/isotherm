using IsoTherm.Thermals;
using SysCommand.ConsoleApp;
using SysCommand.Mapping;

public class Program
{
    public static int Main(string[] args)
    {
        return App.RunApplication();
    }

    public class MyCommand:Command
    {
        IsoChillerThermal isoChiller ;

         [Argument(LongName ="accuracy", ShortName = 'a')]
        public double ThermalAccuracy {get;set;} = 0.5;

         [Argument(LongName ="stabilization",ShortName = 's')]
        public double ThermalStabilization {get;set;} = 1;
        public void Main()
        {
            
        }

        private void InitializeIsoTherm(){
            try
            {
                isoChiller =  new IsoChillerThermal(this.App);
                isoChiller.ThermalAccuracyWrapper = ThermalAccuracy.ToString();
                isoChiller.ThermalStabilizationWrapper = ThermalStabilization.ToString();
                string errorMessage = string.Empty;

                isoChiller.Initialize(out errorMessage);
                if (!string.IsNullOrEmpty(errorMessage)){
                    throw new Exception(errorMessage);
                }
            }
            catch (System.Exception ex)
            {
                
                throw;
            }
        }

        private void ReleaseIsoTherm(){
            isoChiller.Release();
        }

        public void SetTemperature(
            [Argument(ShortName = 't')]
            double temperature
        )
        {
            InitializeIsoTherm();
            isoChiller.SetTemperature(temperature);
            ReleaseIsoTherm();
        }

        public void SetTemperatureAsync(
            [Argument(ShortName = 't')]
            double temperature
        )
        {
            InitializeIsoTherm();
            isoChiller.SetTemperatureAsync(temperature);
            ReleaseIsoTherm();
        }

        public void GetTemperature()
        {
            InitializeIsoTherm();
            App.Console.Write(isoChiller.GetTemperature(),forceWrite:true);
            ReleaseIsoTherm();
        }
    }

}

