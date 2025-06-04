using System;
using System.ComponentModel;
using ExtrusionAnalysis.ViewModel;

namespace ExtrusionAnalysis.Models
{
    public class ScrewSection : ObservableObject
    {
        private string _id;
        private SectionType _type; // Pascal: S_Type (1-нарезанная, 2-гладкая)
        private int _order; // Pascal: Order (порядковый номер)
        private int _monolit; // Pascal: Monolit (1 или другой, смысл уточняется)
        private double _dStartMM; // Pascal: D_start, мм (в расчетах будет в м)
        private double _dFinalMM; // Pascal: D_final, мм (в расчетах будет в м)
        private double _hStartMM; // Pascal: H_start, мм (глубина канала)
        private double _hFinalMM; // Pascal: H_final, мм (глубина канала)
        private double _lengthMM; // Pascal: L_Sect, мм (длина секции)

        // Только для нарезанных секций (S_Type=1)
        private ThreadDirection? _threadDirection; // Pascal: R or L (1 - Right)
        private int? _numberOfThreads; // Pascal: i (число заходов)
        private double? _threadPitchMM; // Pascal: Step, мм (шаг нарезки)
        private double? _threadThicknessStartMM; // Pascal: e_start, мм (ширина гребня)
        private double? _threadThicknessFinalMM; // Pascal: e_final, мм (ширина гребня)
        private double? _radialClearanceMM; // Pascal: delta, мм (радиальный зазор)
        private double? _angleDegrees; // Pascal: Angle, град. (угол подъема винтовой линии)
        private double? _cutWidthMM; // Pascal: W_cut, мм (ширина выреза)

        // Только для гладких секций (S_Type=2) или общее для всех
        private int? _integrationCycles; // Pascal: n_cykle / k_integr (число циклов интегрирования)

        public string Id
        {
            get => _id;
            set => SetProperty(ref _id, value);
        }

        public SectionType Type
        {
            get => _type;
            set => SetProperty(ref _type, value);
        }

        public int Order
        {
            get => _order;
            set => SetProperty(ref _order, value);
        }

        public int Monolit // Monolit
        {
            get => _monolit;
            set => SetProperty(ref _monolit, value);
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

        public double HStartMM // H_start, мм
        {
            get => _hStartMM;
            set => SetProperty(ref _hStartMM, value);
        }

        public double HFinalMM // H_final, мм
        {
            get => _hFinalMM;
            set => SetProperty(ref _hFinalMM, value);
        }

        public double LengthMM // L_Sect, мм
        {
            get => _lengthMM;
            set => SetProperty(ref _lengthMM, value);
        }

        // --- Только для нарезанных секций ---
        public ThreadDirection? ThreadDirection // R or L
        {
            get => _threadDirection;
            set => SetProperty(ref _threadDirection, value);
        }

        public int? NumberOfThreads // i
        {
            get => _numberOfThreads;
            set => SetProperty(ref _numberOfThreads, value);
        }

        public double? ThreadPitchMM // Step, мм
        {
            get => _threadPitchMM;
            set => SetProperty(ref _threadPitchMM, value);
        }

        public double? ThreadThicknessStartMM // e_start, мм
        {
            get => _threadThicknessStartMM;
            set => SetProperty(ref _threadThicknessStartMM, value);
        }

        public double? ThreadThicknessFinalMM // e_final, мм
        {
            get => _threadThicknessFinalMM;
            set => SetProperty(ref _threadThicknessFinalMM, value);
        }

        public double? RadialClearanceMM // delta, мм
        {
            get => _radialClearanceMM;
            set => SetProperty(ref _radialClearanceMM, value);
        }

        public double? AngleDegrees // Angle, град.
        {
            get => _angleDegrees;
            set => SetProperty(ref _angleDegrees, value);
        }

        public double? CutWidthMM // W_cut, мм
        {
            get => _cutWidthMM;
            set => SetProperty(ref _cutWidthMM, value);
        }

        // --- Общее, но для гладких обычно называется k_integr ---
        public int? IntegrationCycles // n_cykle / k_integr
        {
            get => _integrationCycles;
            set => SetProperty(ref _integrationCycles, value);
        }
    }
}