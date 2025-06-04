using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using ExtrusionAnalysis.ViewModel;

namespace ExtrusionAnalysis.Models
{
    public class GeneralParameters : ObservableObject
    {
        // --- DATA.exe: Общие данные (Таблица 1.2, Pascal DataRec) ---
        private double _rotationFrequency; // Pascal: N (частота вращения червяка, об/мин)
        private double _initialMixtureTemperature; // Pascal: T_start (начальная температура смеси, °C)
        private double _barrelMaterialHeatConductivity; // Pascal: Lam_bar (теплопроводность материала корпуса, Вт/(м К))
        private double _screwMaterialHeatConductivity; // Pascal: Lam_scr (теплопроводность материала червяка, Вт/(м К))
        private double _mixtureToBarrelHeatTransferCoeff; // Pascal: Alpha_b (коэффициент теплоотдачи смеси к корпусу, Вт/(м^2 К))
        private double _mixtureToScrewHeatTransferCoeff; // Pascal: Alpha_s (коэффициент теплоотдачи смеси к червяку, Вт/(м^2 К))
        private double _screwWallThicknessMM; // Pascal: Delta_s (толщина теплопередающей стенки червяка, мм)
        private double _wallToMediumHeatTransferCoeff; // Pascal: Al_med (коэффиц. ее теплоотдачи к теплоносителю, Вт/(м^2 К))
        private double _mediumTemperatureInScrewCavity; // Pascal: T_med (температура теплоносителя в полости червяка, °C)
        private double _mediumThermalDiffusivityE8; // Pascal: a_med (его температуропроводность, м^2/с, ×10^8)
        private double _mediumHeatConductivity; // Pascal: Lam_med (его теплопроводность, Вт/(м К))
        private double _mediumFlowRateDM3Min; // Pascal: Q_med (его расход через полость червяка, дм^3/мин)
        private int _numberOfIntegrationCycles; // Pascal: n_Integ (число циклов интегрирования вдоль оси червяка)
        private int _numberOfDrawingSteps; // Pascal: n_Graph (число шагов вдоль оси червяка при его рисовании)

        // --- QPT.exe: Дополнительные данные (Таблица 2.1, Pascal DOP_DATA) ---
        private double _airFractionAtFirstSectionPercent; // Pascal: k_V_air (объемная доля воздуха в материале в 1-й секции, %)
        private double _minimalRelativeExpenditure; // Pascal: Q_min/Q_b (минимальный относительный расход материала)
        private double _maximalRelativeExpenditure; // Pascal: Q_max/Q_b (максимальный относительный расход материала)
        private int _numberOfStepsByRelativeExpense; // Pascal: n_Step (число шагов по относит. расходу Q/Q_b)

        // --- ROUND.exe: Общие данные для головки (Pascal FluxData) ---
        private int _numberOfMaterialSlicesInFlood; // Pascal: N (число сопряженных материальных слоев в потоке)
        private int _numberOfStepsByROverR; // Pascal: iR (число регулярных шагов по r/R для построения v(r))
        private double _irregularityCoeffAlpha1; // Pascal: AH (коэффициент неравномерности шага по "alpha")
        private double _irregularityCoeffAlpha2; // Pascal: AB (коэффициент неравномерности шага по "alpha")
        private double _irregularityCoeffROverR1; // Pascal: kH (коэффициент неравномерности шага по r/R)
        private double _irregularityCoeffROverR2; // Pascal: kB (коэффициент неравномерности шага по r/R)

        // --- Свойства для DATA.exe (Таблица 1.2) ---
        public double RotationFrequency // N
        {
            get => _rotationFrequency;
            set => SetProperty(ref _rotationFrequency, value);
        }

        public double InitialMixtureTemperature // T_start
        {
            get => _initialMixtureTemperature;
            set => SetProperty(ref _initialMixtureTemperature, value);
        }

        public double BarrelMaterialHeatConductivity // Lam_bar
        {
            get => _barrelMaterialHeatConductivity;
            set => SetProperty(ref _barrelMaterialHeatConductivity, value);
        }

        public double ScrewMaterialHeatConductivity // Lam_scr
        {
            get => _screwMaterialHeatConductivity;
            set => SetProperty(ref _screwMaterialHeatConductivity, value);
        }

        public double MixtureToBarrelHeatTransferCoeff // Alpha_b
        {
            get => _mixtureToBarrelHeatTransferCoeff;
            set => SetProperty(ref _mixtureToBarrelHeatTransferCoeff, value);
        }

        public double MixtureToScrewHeatTransferCoeff // Alpha_s
        {
            get => _mixtureToScrewHeatTransferCoeff;
            set => SetProperty(ref _mixtureToScrewHeatTransferCoeff, value);
        }

        public double ScrewWallThicknessMM // Delta_s (в мм, в расчетах будет в м)
        {
            get => _screwWallThicknessMM;
            set => SetProperty(ref _screwWallThicknessMM, value);
        }

        public double WallToMediumHeatTransferCoeff // Al_med
        {
            get => _wallToMediumHeatTransferCoeff;
            set => SetProperty(ref _wallToMediumHeatTransferCoeff, value);
        }

        public double MediumTemperatureInScrewCavity // T_med
        {
            get => _mediumTemperatureInScrewCavity;
            set => SetProperty(ref _mediumTemperatureInScrewCavity, value);
        }

        public double MediumThermalDiffusivityE8 // a_med (в м^2/с * 10^8, в расчетах будет в м^2/с)
        {
            get => _mediumThermalDiffusivityE8;
            set => SetProperty(ref _mediumThermalDiffusivityE8, value);
        }

        public double MediumHeatConductivity // Lam_med
        {
            get => _mediumHeatConductivity;
            set => SetProperty(ref _mediumHeatConductivity, value);
        }

        public double MediumFlowRateDM3Min // Q_med (в дм^3/мин, в расчетах будет в м^3/с)
        {
            get => _mediumFlowRateDM3Min;
            set => SetProperty(ref _mediumFlowRateDM3Min, value);
        }

        public int NumberOfIntegrationCycles // n_Integ
        {
            get => _numberOfIntegrationCycles;
            set => SetProperty(ref _numberOfIntegrationCycles, value);
        }

        public int NumberOfDrawingSteps // n_Graph
        {
            get => _numberOfDrawingSteps;
            set => SetProperty(ref _numberOfDrawingSteps, value);
        }

        // --- Свойства для QPT.exe (Таблица 2.1) ---
        public double AirFractionAtFirstSectionPercent // k_V_air (в %, в расчетах будет в долях)
        {
            get => _airFractionAtFirstSectionPercent;
            set => SetProperty(ref _airFractionAtFirstSectionPercent, value);
        }

        public double MinimalRelativeExpenditure // Q_min/Q_b
        {
            get => _minimalRelativeExpenditure;
            set => SetProperty(ref _minimalRelativeExpenditure, value);
        }

        public double MaximalRelativeExpenditure // Q_max/Q_b
        {
            get => _maximalRelativeExpenditure;
            set => SetProperty(ref _maximalRelativeExpenditure, value);
        }

        public int NumberOfStepsByRelativeExpense // n_Step
        {
            get => _numberOfStepsByRelativeExpense;
            set => SetProperty(ref _numberOfStepsByRelativeExpense, value);
        }

        // --- Свойства для ROUND.exe (Общие данные) ---
        public int NumberOfMaterialSlicesInFlood // N
        {
            get => _numberOfMaterialSlicesInFlood;
            set => SetProperty(ref _numberOfMaterialSlicesInFlood, value);
        }

        public int NumberOfStepsByROverR // iR
        {
            get => _numberOfStepsByROverR;
            set => SetProperty(ref _numberOfStepsByROverR, value);
        }

        public double IrregularityCoeffAlpha1 // AH
        {
            get => _irregularityCoeffAlpha1;
            set => SetProperty(ref _irregularityCoeffAlpha1, value);
        }

        public double IrregularityCoeffAlpha2 // AB
        {
            get => _irregularityCoeffAlpha2;
            set => SetProperty(ref _irregularityCoeffAlpha2, value);
        }

        public double IrregularityCoeffROverR1 // kH
        {
            get => _irregularityCoeffROverR1;
            set => SetProperty(ref _irregularityCoeffROverR1, value);
        }

        public double IrregularityCoeffROverR2 // kB
        {
            get => _irregularityCoeffROverR2;
            set => SetProperty(ref _irregularityCoeffROverR2, value);
        }
    }
}