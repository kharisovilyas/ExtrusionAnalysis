using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using ExtrusionAnalysis.ViewModel;

namespace ExtrusionAnalysis.Models
{
    public class HeadSection : ObservableObject
    {
        private string _id;
        private int _order; // Pascal: Order
        private double _dStartMM; // Pascal: D_st, мм (в расчетах будет в м)
        private double _dFinalMM; // Pascal: D_fin, мм (в расчетах будет в м)
        private double _lengthMM; // Pascal: L_sect, мм (в расчетах будет в м)
        private double _wallTemperatureC; // Pascal: T_st, °C
        private int _integrationCycles; // Pascal: n_cykle (число циклов интегрирования вдоль секции)

        public int Order
        {
            get => _order;
            set => SetProperty(ref _order, value);
        }

        public double DStartMM // D_start, мм
        {
            get => _dStartMM;
            set => SetProperty(ref _dStartMM, value);
        }

        public double DFinalMM // D_final, мм
        {
            get => _dFinalMM;
            set => SetProperty(ref _dFinalMM, value);
        }

        public double LengthMM // L, мм
        {
            get => _lengthMM;
            set => SetProperty(ref _lengthMM, value);
        }

        public double WallTemperatureC // T_st, °C
        {
            get => _wallTemperatureC;
            set => SetProperty(ref _wallTemperatureC, value);
        }

        public int IntegrationCycles // k_integr
        {
            get => _integrationCycles;
            set => SetProperty(ref _integrationCycles, value);
        }

        public string Id
        {
            get => _id;
            internal set => SetProperty(ref _id, value);
        }
    }

}