using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using ExtrusionAnalysis.ViewModel;

namespace ExtrusionAnalysis.Models
{
    public class BarrelSection : ObservableObject
    {
        private string _id;
        private double _lengthMM; // Pascal: L, мм (в расчетах будет в м)
        private double _wallThicknessMM; // Pascal: Delta, мм (в расчетах будет в м)
        private double _temperatureC; // Pascal: T, °C
        private double _heatTransferCoefficientWM2K; // Pascal: Al, Вт/(м^2 К)
        private double _thermalDiffusivityM2S_E8; // Pascal: a, м^2/с, ×10^8 (в расчетах будет в м^2/с)
        private double _thermalConductivityWMK; // Pascal: Lam, Вт/(м К)
        private double _flowRateDM3Min; // Pascal: Q, дм^3/мин (в расчетах будет в м^3/с)

        public string Id
        {
            get => _id;
            set => SetProperty(ref _id, value);
        }

        public double LengthMM // L, мм
        {
            get => _lengthMM;
            set => SetProperty(ref _lengthMM, value);
        }

        public double WallThicknessMM // Delta, мм
        {
            get => _wallThicknessMM;
            set => SetProperty(ref _wallThicknessMM, value);
        }

        public double TemperatureC // T, °C
        {
            get => _temperatureC;
            set => SetProperty(ref _temperatureC, value);
        }

        public double HeatTransferCoefficientWM2K // Al
        {
            get => _heatTransferCoefficientWM2K;
            set => SetProperty(ref _heatTransferCoefficientWM2K, value);
        }

        public double ThermalDiffusivityM2S_E8 // a (в м^2/с * 10^8, в расчетах будет в м^2/с)
        {
            get => _thermalDiffusivityM2S_E8;
            set => SetProperty(ref _thermalDiffusivityM2S_E8, value);
        }

        public double ThermalConductivityWMK // Lam
        {
            get => _thermalConductivityWMK;
            set => SetProperty(ref _thermalConductivityWMK, value);
        }

        public double FlowRateDM3Min // Q, дм^3/мин (в расчетах будет в м^3/с)
        {
            get => _flowRateDM3Min;
            set => SetProperty(ref _flowRateDM3Min, value);
        }
    }
}