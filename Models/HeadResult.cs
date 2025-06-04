// ExtrusionAnalysis.Models/HeadResult.cs
using ExtrusionAnalysis.ViewModel;

namespace ExtrusionAnalysis.Models
{
    public class HeadResult : ObservableObject
    {
        // Pascal ROUND.exe Final Parameters Output
        private string _id;
        private double _volumeExpenseSM3S; // Pascal Q=... sm^3/sec (Объемный расход рабочей точки)
        private double _velocityMMin; // Pascal v=... m/min (Скорость экструзии рабочей точки)
        private double _crossSectionAreaCM2; // Pascal S=... sm^2 (Площадь сечения головки в рабочей точке)
        private double _temperatureMaterialEnteringHeadC; // Pascal T_in=... centigr. (Температура на входе в головку, т.е. на выходе из винта)
        private double _temperatureMaterialAfterExtrusionC; // Pascal T_out=... centigr. (Температура на выходе из головки в рабочей точке)
        private double _partInductionPeriodScrewPercent; // Pascal A part of induction period which is spent: % (Доля индукционного периода, накопленная в винте)
        private double _additionalPartInductionPeriodHeadPercent; // Pascal Additional part of induction period in a head: % (Дополнительная доля индукционного периода, накопленная в головке)
        private double _specificPressureNearHeadMPa; // Pascal p=... MPa (Удельное давление возле головки, т.е. давление рабочей точки)

        // Радиальные профили для рабочей точки (из скриншотов Pascal)
        private double[] _radialTemperatureProfileC;
        private double[] _radialViscosityProfilePaS;
        private double[] _radialVulcanizationProfilePercent;
        private double[] _radialTimeEqProfileS;
        private double[] _radialRadiiMM; // Радиусы, соответствующие точкам радиальных профилей

        public string Id
        {
            get => _id;
            set => SetProperty(ref _id, value);
        }

        public double VolumeExpenseSM3S // Q
        {
            get => _volumeExpenseSM3S;
            set => SetProperty(ref _volumeExpenseSM3S, value);
        }

        public double VelocityMMin // v
        {
            get => _velocityMMin;
            set => SetProperty(ref _velocityMMin, value);
        }

        public double CrossSectionAreaCM2 // S
        {
            get => _crossSectionAreaCM2;
            set => SetProperty(ref _crossSectionAreaCM2, value);
        }

        public double TemperatureMaterialEnteringHeadC // T_in
        {
            get => _temperatureMaterialEnteringHeadC;
            set => SetProperty(ref _temperatureMaterialEnteringHeadC, value);
        }

        public double TemperatureMaterialAfterExtrusionC // T_out
        {
            get => _temperatureMaterialAfterExtrusionC;
            set => SetProperty(ref _temperatureMaterialAfterExtrusionC, value);
        }

        public double PartInductionPeriodScrewPercent // % from screw machine
        {
            get => _partInductionPeriodScrewPercent;
            set => SetProperty(ref _partInductionPeriodScrewPercent, value);
        }

        public double AdditionalPartInductionPeriodHeadPercent // % from head
        {
            get => _additionalPartInductionPeriodHeadPercent;
            set => SetProperty(ref _additionalPartInductionPeriodHeadPercent, value);
        }

        public double SpecificPressureNearHeadMPa // p
        {
            get => _specificPressureNearHeadMPa;
            set => SetProperty(ref _specificPressureNearHeadMPa, value);
        }

        // Radial profiles (for display)
        public double[] RadialTemperatureProfileC
        {
            get => _radialTemperatureProfileC;
            set => SetProperty(ref _radialTemperatureProfileC, value);
        }

        public double[] RadialViscosityProfilePaS
        {
            get => _radialViscosityProfilePaS;
            set => SetProperty(ref _radialViscosityProfilePaS, value);
        }

        public double[] RadialVulcanizationProfilePercent
        {
            get => _radialVulcanizationProfilePercent;
            set => SetProperty(ref _radialVulcanizationProfilePercent, value);
        }

        public double[] RadialTimeEqProfileS
        {
            get => _radialTimeEqProfileS;
            set => SetProperty(ref _radialTimeEqProfileS, value);
        }

        public double[] RadialRadiiMM
        {
            get => _radialRadiiMM;
            set => SetProperty(ref _radialRadiiMM, value);
        }
    }
}