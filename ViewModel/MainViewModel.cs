using ExtrusionAnalysis.Calculators;
using ExtrusionAnalysis.Models;
using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Input;
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using ExtrusionAnalysis.ViewModel;

namespace ExtrusionAnalysis.ViewModels
{
    public class MainViewModel : ObservableObject
    {
        private MaterialProperties _materialProperties;
        private GeneralParameters _generalParameters;
        private ObservableCollection<ScrewSection> _screwSections;
        private ObservableCollection<BarrelSection> _barrelSections;
        private KineticsParameters _kineticsParameters;
        private FluxData _fluxData; // Для ROUND.exe (больше не используется напрямую в калькуляторе, но оставил как входную модель)
        private ObservableCollection<HeadSection> _headSections;

        // Полные коллекции результатов до фильтрации
        private ObservableCollection<ExtrusionResult> _allScrewResults;
        private ObservableCollection<HeadResult> _allHeadResults;

        // Отфильтрованные коллекции для отображения в DataGrid
        private ObservableCollection<ExtrusionResult> _filteredScrewResults;
        private ObservableCollection<HeadResult> _filteredHeadResults;

        private ScrewSection _selectedScrewSection;
        private HeadSection _selectedHeadSection;

        // Plot Models (Сокращенный список для основных графиков)
        private PlotModel _pressurePlotModel; // QPT: Давление по длине
        private PlotModel _temperaturePlotModel; // QPT: Температура по длине
        private PlotModel _pressureFlowPlotModel; // QPT: Характеристика винта (P vs Q)

        private PlotModel _pressureVsFlowPlotModel; // ROUND: Характеристика головки (P vs Q)
        private PlotModel _temperatureVsFlowPlotModel; // ROUND: Характеристика головки (T vs Q)
        private PlotModel _radialTemperaturePlotModel; // ROUND: Радиальный профиль температуры
        private PlotModel _radialViscosityPlotModel; // ROUND: Радиальный профиль вязкости
        private PlotModel _radialVulcanizationPlotModel; // ROUND: Радиальный профиль вулканизации
        private PlotModel _radialTimeEqPlotModel; // ROUND: Радиальный профиль эквивалентного времени

        // Summary results
        private double _maxPressure;
        private double _finalTemperature;
        private double _finalFlowRate;
        private double _finalPressureHead;
        private double _summaryShearDeformation; // Итоговая деформация сдвига

        public PlotModel PressurePlotModel { get => _pressurePlotModel; set => SetProperty(ref _pressurePlotModel, value); }
        public PlotModel TemperaturePlotModel { get => _temperaturePlotModel; set => SetProperty(ref _temperaturePlotModel, value); }
        public PlotModel PressureFlowPlotModel { get => _pressureFlowPlotModel; set => SetProperty(ref _pressureFlowPlotModel, value); }
        public PlotModel PressureVsFlowPlotModel { get => _pressureVsFlowPlotModel; set => SetProperty(ref _pressureVsFlowPlotModel, value); }
        public PlotModel TemperatureVsFlowPlotModel { get => _temperatureVsFlowPlotModel; set => SetProperty(ref _temperatureVsFlowPlotModel, value); }
        public PlotModel RadialTemperaturePlotModel { get => _radialTemperaturePlotModel; set => SetProperty(ref _radialTemperaturePlotModel, value); }
        public PlotModel RadialViscosityPlotModel { get => _radialViscosityPlotModel; set => SetProperty(ref _radialViscosityPlotModel, value); }
        public PlotModel RadialVulcanizationPlotModel { get => _radialVulcanizationPlotModel; set => SetProperty(ref _radialVulcanizationPlotModel, value); }
        public PlotModel RadialTimeEqPlotModel { get => _radialTimeEqPlotModel; set => SetProperty(ref _radialTimeEqPlotModel, value); }

        public MaterialProperties MaterialProperties { get => _materialProperties; set => SetProperty(ref _materialProperties, value); }
        public GeneralParameters GeneralParameters { get => _generalParameters; set => SetProperty(ref _generalParameters, value); }
        public ObservableCollection<ScrewSection> ScrewSections { get => _screwSections; set => SetProperty(ref _screwSections, value); }
        public ObservableCollection<BarrelSection> BarrelSections { get => _barrelSections; set => SetProperty(ref _barrelSections, value); }
        public KineticsParameters KineticsParameters { get => _kineticsParameters; set => SetProperty(ref _kineticsParameters, value); }
        public FluxData FluxData { get => _fluxData; set => SetProperty(ref _fluxData, value); }
        public ObservableCollection<HeadSection> HeadSections { get => _headSections; set => SetProperty(ref _headSections, value); }

        public ObservableCollection<ExtrusionResult> FilteredScrewResults { get => _filteredScrewResults; set => SetProperty(ref _filteredScrewResults, value); }
        public ObservableCollection<HeadResult> FilteredHeadResults { get => _filteredHeadResults; set => SetProperty(ref _filteredHeadResults, value); }

        public ScrewSection SelectedScrewSection
        {
            get => _selectedScrewSection;
            set
            {
                if (SetProperty(ref _selectedScrewSection, value))
                {
                    UpdateFilteredScrewResults();
                }
            }
        }

        public HeadSection SelectedHeadSection
        {
            get => _selectedHeadSection;
            set
            {
                if (SetProperty(ref _selectedHeadSection, value))
                {
                    UpdateFilteredHeadResults();
                }
            }
        }

        public double MaxPressure { get => _maxPressure; set => SetProperty(ref _maxPressure, value); }
        public double FinalTemperature { get => _finalTemperature; set => SetProperty(ref _finalTemperature, value); }
        public double FinalFlowRate { get => _finalFlowRate; set => SetProperty(ref _finalFlowRate, value); }
        public double FinalPressureHead { get => _finalPressureHead; set => SetProperty(ref _finalPressureHead, value); }
        public double SummaryShearDeformation { get => _summaryShearDeformation; set => SetProperty(ref _summaryShearDeformation, value); }


        public ICommand LoadDataCommand { get; }
        public ICommand SaveDataCommand { get; }
        public ICommand CalculateCommand { get; }
        public ICommand SaveResultsCommand { get; }

        public MainViewModel()
        {
            MaterialProperties = new MaterialProperties();
            GeneralParameters = new GeneralParameters();
            ScrewSections = new ObservableCollection<ScrewSection>();
            BarrelSections = new ObservableCollection<BarrelSection>();
            KineticsParameters = new KineticsParameters();
            FluxData = new FluxData();
            HeadSections = new ObservableCollection<HeadSection>();

            _allScrewResults = new ObservableCollection<ExtrusionResult>();
            FilteredScrewResults = new ObservableCollection<ExtrusionResult>();
            _allHeadResults = new ObservableCollection<HeadResult>();
            FilteredHeadResults = new ObservableCollection<HeadResult>();

            // Инициализация моделей графиков
            PressurePlotModel = new PlotModel { Title = "Профиль удельного давления" };
            TemperaturePlotModel = new PlotModel { Title = "Изменение температуры материала" };
            PressureFlowPlotModel = new PlotModel { Title = "Внешняя характеристика червяка" };

            PressureVsFlowPlotModel = new PlotModel { Title = "Давление от расхода (головка)" };
            TemperatureVsFlowPlotModel = new PlotModel { Title = "Температура от расхода (головка)" };
            RadialTemperaturePlotModel = new PlotModel { Title = "Температура по радиусу" };
            RadialViscosityPlotModel = new PlotModel { Title = "Вязкость по радиусу" };
            RadialVulcanizationPlotModel = new PlotModel { Title = "Вулканизация по радиусу" };
            RadialTimeEqPlotModel = new PlotModel { Title = "Эквивалентное время по радиусу" };

            LoadDataCommand = new RelayCommand(LoadData);
            SaveDataCommand = new RelayCommand(SaveData);
            CalculateCommand = new RelayCommand(Calculate);
            SaveResultsCommand = new RelayCommand(SaveResults);
            InitializePlotModels();
            InitializeDefaultData();
        }

        private void InitializePlotModels()
        {
            // Очистить оси для перенастройки
            PressurePlotModel.Axes.Clear();
            TemperaturePlotModel.Axes.Clear();
            PressureFlowPlotModel.Axes.Clear();
            PressureVsFlowPlotModel.Axes.Clear();
            TemperatureVsFlowPlotModel.Axes.Clear();
            RadialTemperaturePlotModel.Axes.Clear();
            RadialViscosityPlotModel.Axes.Clear();
            RadialVulcanizationPlotModel.Axes.Clear();
            RadialTimeEqPlotModel.Axes.Clear();

            // QPT Plots
            PressurePlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Z (мм)" });
            PressurePlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "p (МПа)" });
            TemperaturePlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Z (мм)" });
            TemperaturePlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "T (°C)" }); // No fixed min/max
            PressureFlowPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Q (см³/с)" });
            PressureFlowPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "p (МПа)" });

            // ROUND Plots
            PressureVsFlowPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Q (см³/с)" });
            PressureVsFlowPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "p (МПа)" });
            TemperatureVsFlowPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Q (см³/с)" });
            TemperatureVsFlowPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "T (°C)" });
            RadialTemperaturePlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Радиус (мм)" });
            RadialTemperaturePlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "Температура (°C)" });
            RadialViscosityPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Радиус (мм)" });
            RadialViscosityPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "Вязкость (Па·с)" });
            RadialVulcanizationPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Радиус (мм)" });
            RadialVulcanizationPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "Вулканизация (%)" });
            RadialTimeEqPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "Радиус (мм)" });
            RadialTimeEqPlotModel.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "Эквивалентное время (с)" });
        }


        private void InitializeDefaultData()
        {
            // --- DATA.exe: Таблица 1.1 (Material Properties) ---
            MaterialProperties = new MaterialProperties
            {
                FlowIndex = 0.200, // n
                CharacteristicTemperature = 170.0, // T0, °C
                ConsistencyCoefficient = 200.0, // Mu0, кПа с^n
                TemperatureViscosityInfluence = 0.0400, // b, 1/K
                Density = 1100.0, // Ro, кг/м^3
                ThermalDiffusivity = 11.0, // a, м^2/с, ×10^8 (будет 11.0 / 1e8 в расчетах)
                HeatConductivity = 0.190, // Lam, Вт/(м К)
                EquivalentTemperature = 190.0, // T_eqv, °C
                InductionTime = 120.0, // t_in_eq, с
                OptimalTime = 600.0, // t_op_eq, с
                InitialTemperature = 170.0 // T_ini, °C
            };

            // --- DATA.exe: Таблица 1.2 (General Extruder Data) ---
            GeneralParameters = new GeneralParameters
            {
                RotationFrequency = 30.0, // N, об/мин (будет об/с в расчетах)
                InitialMixtureTemperature = 160.0, // T_start, °C
                BarrelMaterialHeatConductivity = 45.4, // Lam_bar, Вт/(м К)
                ScrewMaterialHeatConductivity = 45.4, // Lam_scr, Вт/(м К)
                MixtureToBarrelHeatTransferCoeff = 100.0, // Alpha_b, Вт/(м^2 К)
                MixtureToScrewHeatTransferCoeff = 100.0, // Alpha_s, Вт/(м^2 К)
                ScrewWallThicknessMM = 8.0, // Delta_s, мм (будет в м в расчетах)
                WallToMediumHeatTransferCoeff = 0.1, // Al_med, Вт/(м^2 К)
                MediumTemperatureInScrewCavity = 90.0, // T_med, °C
                MediumThermalDiffusivityE8 = 2000.0, // a_med, м^2/с, ×10^8 (будет /1e8 в расчетах)
                MediumHeatConductivity = 0.026, // Lam_med, Вт/(м К)
                MediumFlowRateDM3Min = 2.0, // Q_med, дм^3/мин (будет в м^3/с в расчетах)
                NumberOfIntegrationCycles = 250, // n_Integ
                NumberOfDrawingSteps = 1000 // n_Graph
            };

            // --- DATA.exe: Таблица 1.3 (Screw Sections - Cut) ---
            ScrewSections.Clear();
            ScrewSections.Add(new ScrewSection
            {
                Id = "S1",
                Type = SectionType.Cut,
                Order = 1,
                Monolit = 1, // Pascal: Monolit
                DStartMM = 90.0, // D_start, мм
                DFinalMM = 90.0, // D_final, мм
                HStartMM = 15.0, // H_start, мм
                HFinalMM = 15.0, // H_final, мм
                LengthMM = 400.0, // L, мм
                ThreadDirection = ThreadDirection.Right, // R or L (1 - Right)
                NumberOfThreads = 1, // i
                ThreadPitchMM = 90.0, // Step, мм
                ThreadThicknessStartMM = 15.0, // e_start, мм
                ThreadThicknessFinalMM = 15.0, // e_final, мм
                RadialClearanceMM = 0.0010, // delta, мм
                AngleDegrees = 0.0, // Angle, град.
                CutWidthMM = 0.0, // W_cut, мм
                IntegrationCycles = null // n_cykle (не используется для нарезанных в Pascal в этой таблице)
            });
            ScrewSections.Add(new ScrewSection
            {
                Id = "S2",
                Type = SectionType.Cut,
                Order = 2,
                Monolit = 1,
                DStartMM = 90.0,
                DFinalMM = 90.0,
                HStartMM = 15.0,
                HFinalMM = 8.0,
                LengthMM = 500.0,
                ThreadDirection = ThreadDirection.Right,
                NumberOfThreads = 1,
                ThreadPitchMM = 80.0,
                ThreadThicknessStartMM = 15.0,
                ThreadThicknessFinalMM = 10.0,
                RadialClearanceMM = 0.0010,
                AngleDegrees = 0.0,
                CutWidthMM = 0.0,
                IntegrationCycles = null
            });
            // --- DATA.exe: Таблица 1.4 (Screw Sections - Smooth) ---
            ScrewSections.Add(new ScrewSection
            {
                Id = "S3",
                Type = SectionType.Smooth,
                Order = 3,
                Monolit = 1,
                DStartMM = 90.0,
                DFinalMM = 60.0,
                HStartMM = 10.0,
                HFinalMM = 12.0,
                LengthMM = 100.0,
                // ThreadDirection, NumberOfThreads, ThreadPitch, ThreadThickness, RadialClearance, Angle, CutWidth - null для гладких
                IntegrationCycles = 20 // k_integr
            });

            // --- DATA.exe: Таблица 1.5 (Barrel Sections) ---
            BarrelSections.Clear();
            BarrelSections.Add(new BarrelSection
            {
                Id = "B1",
                LengthMM = 500.0, // L, мм
                WallThicknessMM = 20.0, // Delta, мм
                TemperatureC = 151.0, // T, °C
                HeatTransferCoefficientWM2K = 2000.0, // Al, Вт/(м^2 К)
                ThermalDiffusivityM2S_E8 = 14.0, // a, м^2/с, ×10^8
                ThermalConductivityWMK = 0.60, // Lam, Вт/(м К)
                FlowRateDM3Min = 2.0 // Q, дм^3/мин
            });
            BarrelSections.Add(new BarrelSection
            {
                Id = "B2",
                LengthMM = 530.0,
                WallThicknessMM = 15.0,
                TemperatureC = 151.0, // На скриншоте 90, но в DOC_1.doc Table 1.5 - 151. Используем 151 из DOC_1
                HeatTransferCoefficientWM2K = 2000.0, // На скриншоте 350, но в DOC_1.doc Table 1.5 - 2000. Используем 2000 из DOC_1
                ThermalDiffusivityM2S_E8 = 14.0,
                ThermalConductivityWMK = 0.60,
                FlowRateDM3Min = 2.0
            });

            // --- DATA.exe: Таблица 1.6 (Cure Kinetics) ---
            KineticsParameters = new KineticsParameters(); // Создаем новый экземпляр
            KineticsParameters.AddInterval(120.0, 8560.0); // t_eqv, E/R
            KineticsParameters.AddInterval(1800.0, 8560.0);
            // Дополнительные поля KineticsParameters (из MaterialProperties или GeneralParameters)
            // Эти поля дублируются из MaterialProperties, но Pascal их хранит в DATA_REC
            // Здесь просто заполняем, чтобы не было NullReferenceException
            KineticsParameters.EquivalentTemperature = MaterialProperties.EquivalentTemperature;
            KineticsParameters.InductionTime = MaterialProperties.InductionTime;
            KineticsParameters.OptimalTime = MaterialProperties.OptimalTime;
            KineticsParameters.InitialTemperature = MaterialProperties.InitialTemperature;


            // --- QPT.exe: Дополнительные данные (Таблица 2.1) ---
            GeneralParameters.AirFractionAtFirstSectionPercent = 0.00; // k_V_air, %
            GeneralParameters.MinimalRelativeExpenditure = 0.760; // Q_min/Q_b
            GeneralParameters.MaximalRelativeExpenditure = 0.850; // Q_max/Q_b
            GeneralParameters.NumberOfStepsByRelativeExpense = 2; // n_Step

            // --- ROUND.exe: Общие данные для головки ---
            GeneralParameters.NumberOfMaterialSlicesInFlood = 20; // N_Alpha
            GeneralParameters.NumberOfStepsByROverR = 30; // N_r/R
            GeneralParameters.IrregularityCoeffAlpha1 = 1.100; // k1_Alph
            GeneralParameters.IrregularityCoeffAlpha2 = 1.100; // k2_Alph
            GeneralParameters.IrregularityCoeffROverR1 = 1.050; // k1_r/R
            GeneralParameters.IrregularityCoeffROverR2 = 1.050; // k2_r/R

            // --- ROUND.exe: Секции головки ---
            HeadSections.Clear();
            HeadSections.Add(new HeadSection
            {
                Id = "H1",
                Order = 1,
                DStartMM = 60.0, // D_start, мм
                DFinalMM = 12.0, // D_final, мм
                LengthMM = 100.0, // L, мм
                WallTemperatureC = 180.0, // T_st, °C
                IntegrationCycles = 50 // k_integr
            });
            HeadSections.Add(new HeadSection
            {
                Id = "H2",
                Order = 2,
                DStartMM = 12.0,
                DFinalMM = 8.0,
                LengthMM = 50.0, // На скриншоте 50, в DOC_1.doc - 80. Используем 50.
                WallTemperatureC = 180.0,
                IntegrationCycles = 25
            });
            HeadSections.Add(new HeadSection
            {
                Id = "H3",
                Order = 3,
                DStartMM = 8.0,
                DFinalMM = 8.0,
                LengthMM = 30.0, // На скриншоте 30, в DOC_1.doc - 180. Используем 30.
                WallTemperatureC = 180.0,
                IntegrationCycles = 15
            });


            // Установка выбранной секции по умолчанию для DataGrid фильтрации
            if (ScrewSections.Any())
            {
                SelectedScrewSection = ScrewSections.First();
            }
            if (HeadSections.Any())
            {
                SelectedHeadSection = HeadSections.First();
            }
        }


        // Метод Calculate будет заполнять _allScrewResults и _allHeadResults
        // И уже оттуда FilteredScrewResults и FilteredHeadResults будут брать данные

        private void Calculate()
        {
            try
            {
                // Clear previous results
                _allScrewResults.Clear();
                _allHeadResults.Clear();
                // Clear all series for all plots
                PressurePlotModel.Series.Clear();
                TemperaturePlotModel.Series.Clear();
                PressureFlowPlotModel.Series.Clear();
                PressureVsFlowPlotModel.Series.Clear();
                TemperatureVsFlowPlotModel.Series.Clear();
                RadialTemperaturePlotModel.Series.Clear();
                RadialViscosityPlotModel.Series.Clear();
                RadialVulcanizationPlotModel.Series.Clear();
                RadialTimeEqPlotModel.Series.Clear();


                // --- Валидация входных данных ---
                // (ваш код валидации, убрал для краткости, он был в MainViewModel)

                // --- Подготовка данных для ExtrusionCalculator ---
                // Здесь мы преобразуем MM в M, % в доли и т.д., чтобы Calculator работал с SI
                var calculatorMaterialProps = new Models.MaterialProperties
                {
                    FlowIndex = MaterialProperties.FlowIndex,
                    CharacteristicTemperature = MaterialProperties.CharacteristicTemperature,
                    ConsistencyCoefficient = MaterialProperties.ConsistencyCoefficient * 1000.0, // кПа в Па
                    TemperatureViscosityInfluence = MaterialProperties.TemperatureViscosityInfluence,
                    Density = MaterialProperties.Density,
                    ThermalDiffusivity = MaterialProperties.ThermalDiffusivity / 1e8, // *1e8 в SI
                    HeatConductivity = MaterialProperties.HeatConductivity,
                    EquivalentTemperature = MaterialProperties.EquivalentTemperature,
                    InductionTime = MaterialProperties.InductionTime,
                    OptimalTime = MaterialProperties.OptimalTime,
                    InitialTemperature = MaterialProperties.InitialTemperature
                };

                var calculatorGeneralParams = new Models.GeneralParameters
                {
                    RotationFrequency = GeneralParameters.RotationFrequency / 60.0, // об/мин в об/с
                    InitialMixtureTemperature = GeneralParameters.InitialMixtureTemperature,
                    BarrelMaterialHeatConductivity = GeneralParameters.BarrelMaterialHeatConductivity,
                    ScrewMaterialHeatConductivity = GeneralParameters.ScrewMaterialHeatConductivity,
                    MixtureToBarrelHeatTransferCoeff = GeneralParameters.MixtureToBarrelHeatTransferCoeff,
                    MixtureToScrewHeatTransferCoeff = GeneralParameters.MixtureToScrewHeatTransferCoeff,
                    ScrewWallThicknessMM = GeneralParameters.ScrewWallThicknessMM / 1000.0, // мм в м
                    WallToMediumHeatTransferCoeff = GeneralParameters.WallToMediumHeatTransferCoeff,
                    MediumTemperatureInScrewCavity = GeneralParameters.MediumTemperatureInScrewCavity,
                    MediumThermalDiffusivityE8 = GeneralParameters.MediumThermalDiffusivityE8 / 1e8, // *1e8 в SI
                    MediumHeatConductivity = GeneralParameters.MediumHeatConductivity,
                    MediumFlowRateDM3Min = GeneralParameters.MediumFlowRateDM3Min, // Pascal Q_med (дм3/мин) used directly for char calc
                    NumberOfIntegrationCycles = GeneralParameters.NumberOfIntegrationCycles,
                    NumberOfDrawingSteps = GeneralParameters.NumberOfDrawingSteps,

                    // QPT.exe specific
                    AirFractionAtFirstSectionPercent = GeneralParameters.AirFractionAtFirstSectionPercent / 100.0, // % в доли
                    MinimalRelativeExpenditure = GeneralParameters.MinimalRelativeExpenditure,
                    MaximalRelativeExpenditure = GeneralParameters.MaximalRelativeExpenditure,
                    NumberOfStepsByRelativeExpense = GeneralParameters.NumberOfStepsByRelativeExpense,

                    // ROUND.exe specific
                    NumberOfMaterialSlicesInFlood = GeneralParameters.NumberOfMaterialSlicesInFlood,
                    NumberOfStepsByROverR = GeneralParameters.NumberOfStepsByROverR,
                    IrregularityCoeffAlpha1 = GeneralParameters.IrregularityCoeffAlpha1,
                    IrregularityCoeffAlpha2 = GeneralParameters.IrregularityCoeffAlpha2,
                    IrregularityCoeffROverR1 = GeneralParameters.IrregularityCoeffROverR1,
                    IrregularityCoeffROverR2 = GeneralParameters.IrregularityCoeffROverR2
                };

                var calculatorScrewSections = ScrewSections.Select(s => new Models.ScrewSection
                {
                    Id = s.Id,
                    Type = s.Type,
                    Order = s.Order,
                    Monolit = s.Monolit,
                    DStartMM = s.DStartMM / 1000.0, // мм в м
                    DFinalMM = s.DFinalMM / 1000.0, // мм в м
                    HStartMM = s.HStartMM / 1000.0, // мм в м
                    HFinalMM = s.HFinalMM / 1000.0, // мм в м
                    LengthMM = s.LengthMM / 1000.0, // мм в м
                    ThreadDirection = s.ThreadDirection,
                    NumberOfThreads = s.NumberOfThreads,
                    ThreadPitchMM = s.ThreadPitchMM / 1000.0, // мм в м
                    ThreadThicknessStartMM = s.ThreadThicknessStartMM / 1000.0, // мм в м
                    ThreadThicknessFinalMM = s.ThreadThicknessFinalMM / 1000.0, // мм в м
                    RadialClearanceMM = s.RadialClearanceMM / 1000.0, // мм в м
                    AngleDegrees = s.AngleDegrees,
                    CutWidthMM = s.CutWidthMM / 1000.0, // мм в м
                    IntegrationCycles = s.IntegrationCycles
                }).ToList();

                var calculatorBarrelSections = BarrelSections.Select(b => new Models.BarrelSection
                {
                    Id = b.Id,
                    LengthMM = b.LengthMM / 1000.0, // мм в м
                    WallThicknessMM = b.WallThicknessMM / 1000.0, // мм в м
                    TemperatureC = b.TemperatureC,
                    HeatTransferCoefficientWM2K = b.HeatTransferCoefficientWM2K,
                    ThermalDiffusivityM2S_E8 = b.ThermalDiffusivityM2S_E8 / 1e8, // *1e8 в SI
                    ThermalConductivityWMK = b.ThermalConductivityWMK,
                    FlowRateDM3Min = b.FlowRateDM3Min // дм3/мин (используется как есть для Char calc)
                }).ToList();

                var calculatorKineticsParams = new Models.KineticsParameters
                {
                    NumberOfIntervals = KineticsParameters.NumberOfIntervals,
                    EquivalentTemperature = KineticsParameters.EquivalentTemperature,
                    InductionTime = KineticsParameters.InductionTime,
                    OptimalTime = KineticsParameters.OptimalTime,
                    InitialTemperature = KineticsParameters.InitialTemperature,
                    Intervals = new ObservableCollection<Models.KineticsParameters.CureKineticsInterval>(
                        KineticsParameters.Intervals.Select(i => new Models.KineticsParameters.CureKineticsInterval
                        {
                            Order = i.Order,
                            EquivalentTime = i.EquivalentTime,
                            ActivationEnergyOverR = i.ActivationEnergyOverR
                        })
                    )
                };

                var calculatorHeadSections = HeadSections.Select(h => new Models.HeadSection
                {
                    Id = h.Id,
                    Order = h.Order,
                    DStartMM = h.DStartMM / 1000.0, // мм в м
                    DFinalMM = h.DFinalMM / 1000.0, // мм в м
                    LengthMM = h.LengthMM / 1000.0, // мм в м
                    WallTemperatureC = h.WallTemperatureC,
                    IntegrationCycles = h.IntegrationCycles
                }).ToList();

                // Создаем экземпляр калькулятора с подготовленными данными
                var calculator = new ExtrusionCalculator(
                    calculatorMaterialProps,
                    calculatorGeneralParams,
                    calculatorScrewSections,
                    calculatorBarrelSections,
                    calculatorKineticsParams,
                    calculatorHeadSections);


                // --- Выполняем расчеты QPT (Screw Process) ---
                var screwProcessResults = calculator.CalculateScrewProcess();
                _allScrewResults.Clear();
                foreach (var result in screwProcessResults)
                {
                    _allScrewResults.Add(result);
                }
                UpdateFilteredScrewResults(); // Обновить отфильтрованный список для DataGrid

                // --- Получаем характеристику винта (Screw Characteristic Curve) ---
                var screwCharacteristicCurve = calculator.BuildScrewCharacteristicCurve();
                PressureFlowPlotModel.Series.Clear();
                var screwCharSeries = new LineSeries { Title = "Внешняя характеристика винта" };
                foreach (var (flowRate, pressure) in screwCharacteristicCurve)
                {
                    screwCharSeries.Points.Add(new DataPoint(flowRate, pressure));
                }
                PressureFlowPlotModel.Series.Add(screwCharSeries);
                PressureFlowPlotModel.InvalidatePlot(true);


                // --- Выполняем расчеты ROUND (Head Process) ---
                // Temperature at head entrance (from last screw section)
                double initialHeadTemperatureC = _allScrewResults.Any() ? _allScrewResults.Last().TemperatureC : GeneralParameters.InitialMixtureTemperature;
                double finalScrewShearDeformationSummary = _allScrewResults.Any() ? _allScrewResults.Last().ShearDeformationSummary : 0;

                var headProcessResult = calculator.CalculateHeadProcess(
                    screwCharacteristicCurve.ToDictionary(k => k.FlowRate, v => v.Pressure), // Characteristic of screw
                    initialHeadTemperatureC, // Temp at head entrance
                    finalScrewShearDeformationSummary // Total shear deformation from screw
                );

                _allHeadResults.Clear();
                if (headProcessResult != null)
                {
                    _allHeadResults.Add(headProcessResult);
                }
                UpdateFilteredHeadResults(); // Update filtered list for DataGrid


                // --- Итоговые значения ---
                MaxPressure = _allScrewResults.Any() ? _allScrewResults.Max(r => r.PressureMPa) : 0;
                FinalTemperature = _allHeadResults.Any() ? _allHeadResults.First().TemperatureMaterialAfterExtrusionC :
                                 (_allScrewResults.Any() ? _allScrewResults.Last().TemperatureC : 0);
                FinalFlowRate = _allHeadResults.Any() ? _allHeadResults.First().VolumeExpenseSM3S :
                                 (_allScrewResults.Any() ? _allScrewResults.Last().FlowRateDeltaSM3S : 0);
                FinalPressureHead = _allHeadResults.Any() ? _allHeadResults.First().SpecificPressureNearHeadMPa : 0;
                SummaryShearDeformation = _allScrewResults.Any() ? _allScrewResults.Last().ShearDeformationSummary : 0; // Итоговая деформация сдвига

                Debug.WriteLine($"Calculation complete: MaxPressure={MaxPressure:F3} MPa, FinalTemperature={FinalTemperature:F3} °C, FinalFlowRate={FinalFlowRate:F3} cm³/s, FinalPressureHead={FinalPressureHead:F3} MPa, SummaryShearDeformation={SummaryShearDeformation:F1} relative units");
            }
            catch (ArgumentException ex)
            {
                MessageBox.Show(ex.Message, "Ошибка валидации", MessageBoxButton.OK, MessageBoxImage.Error);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Ошибка расчета: {ex.Message}\n{ex.StackTrace}", "Ошибка", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void UpdateFilteredScrewResults()
        {
            if (SelectedScrewSection != null && _allScrewResults != null)
            {
                FilteredScrewResults = new ObservableCollection<ExtrusionResult>(
                    _allScrewResults.Where(r => r.Id == SelectedScrewSection.Id));
            }
            else if (_allScrewResults != null)
            {
                FilteredScrewResults = new ObservableCollection<ExtrusionResult>(_allScrewResults);
            }
            else
            {
                FilteredScrewResults = new ObservableCollection<ExtrusionResult>();
            }
            UpdateScrewPlots(FilteredScrewResults);
        }

        private void UpdateFilteredHeadResults()
        {
            if (SelectedHeadSection != null && _allHeadResults != null)
            {
                FilteredHeadResults = new ObservableCollection<HeadResult>(
                    _allHeadResults.Where(r => r.Id == SelectedHeadSection.Id));
            }
            else if (_allHeadResults != null)
            {
                FilteredHeadResults = new ObservableCollection<HeadResult>(_allHeadResults);
            }
            else
            {
                FilteredHeadResults = new ObservableCollection<HeadResult>();
            }
            UpdateHeadPlots(FilteredHeadResults);
        }

        // Обновление графиков для винта
        private void UpdateScrewPlots(ObservableCollection<ExtrusionResult> resultsToDisplay)
        {
            PressurePlotModel.Series.Clear();
            TemperaturePlotModel.Series.Clear();

            if (!resultsToDisplay.Any())
            {
                PressurePlotModel.InvalidatePlot(true);
                TemperaturePlotModel.InvalidatePlot(true);
                return;
            }

            var pressureSeries = new LineSeries { Title = "Давление" };
            var temperatureSeries = new LineSeries { Title = "Температура" };

            foreach (var result in resultsToDisplay.OrderBy(r => r.PositionMM))
            {
                pressureSeries.Points.Add(new DataPoint(result.PositionMM, result.PressureMPa));
                temperatureSeries.Points.Add(new DataPoint(result.PositionMM, result.TemperatureC));
            }

            PressurePlotModel.Series.Add(pressureSeries);
            TemperaturePlotModel.Series.Add(temperatureSeries);

            PressurePlotModel.InvalidatePlot(true);
            TemperaturePlotModel.InvalidatePlot(true);
        }

        // Обновление графиков для головки
        private void UpdateHeadPlots(ObservableCollection<HeadResult> resultsToDisplay)
        {
            RadialTemperaturePlotModel.Series.Clear();
            RadialViscosityPlotModel.Series.Clear();
            RadialVulcanizationPlotModel.Series.Clear();
            RadialTimeEqPlotModel.Series.Clear();
            PressureVsFlowPlotModel.Series.Clear();
            TemperatureVsFlowPlotModel.Series.Clear();

            if (!resultsToDisplay.Any())
            {
                RadialTemperaturePlotModel.InvalidatePlot(true);
                RadialViscosityPlotModel.InvalidatePlot(true);
                RadialVulcanizationPlotModel.InvalidatePlot(true);
                RadialTimeEqPlotModel.InvalidatePlot(true);
                PressureVsFlowPlotModel.InvalidatePlot(true);
                TemperatureVsFlowPlotModel.InvalidatePlot(true);
                return;
            }

            var headResult = resultsToDisplay.First();

            // Радиальные профили
            if (headResult.RadialTemperatureProfileC != null && headResult.RadialRadiiMM != null && headResult.RadialRadiiMM.Length > 0)
            {
                var radialTempSeries = new LineSeries { Title = "T(r)" };
                for (int i = 0; i < headResult.RadialTemperatureProfileC.Length; i++)
                {
                    radialTempSeries.Points.Add(new DataPoint(headResult.RadialRadiiMM[i], headResult.RadialTemperatureProfileC[i]));
                }
                RadialTemperaturePlotModel.Series.Add(radialTempSeries);
            }

            if (headResult.RadialViscosityProfilePaS != null && headResult.RadialRadiiMM != null && headResult.RadialRadiiMM.Length > 0)
            {
                var radialViscSeries = new LineSeries { Title = "μ(r)" };
                for (int i = 0; i < headResult.RadialViscosityProfilePaS.Length; i++)
                {
                    radialViscSeries.Points.Add(new DataPoint(headResult.RadialRadiiMM[i], headResult.RadialViscosityProfilePaS[i]));
                }
                RadialViscosityPlotModel.Series.Add(radialViscSeries);
            }

            if (headResult.RadialVulcanizationProfilePercent != null && headResult.RadialRadiiMM != null && headResult.RadialRadiiMM.Length > 0)
            {
                var radialVulcSeries = new LineSeries { Title = "Vul(r)" };
                for (int i = 0; i < headResult.RadialVulcanizationProfilePercent.Length; i++)
                {
                    radialVulcSeries.Points.Add(new DataPoint(headResult.RadialRadiiMM[i], headResult.RadialVulcanizationProfilePercent[i]));
                }
                RadialVulcanizationPlotModel.Series.Add(radialVulcSeries);
            }

            if (headResult.RadialTimeEqProfileS != null && headResult.RadialRadiiMM != null && headResult.RadialRadiiMM.Length > 0)
            {
                var radialTimeEqSeries = new LineSeries { Title = "t_eq(r)" };
                for (int i = 0; i < headResult.RadialTimeEqProfileS.Length; i++)
                {
                    radialTimeEqSeries.Points.Add(new DataPoint(headResult.RadialRadiiMM[i], headResult.RadialTimeEqProfileS[i]));
                }
                RadialTimeEqPlotModel.Series.Add(radialTimeEqSeries);
            }

            // Характеристики головки (p vs Q, T vs Q) - используем _allHeadResults для построения кривых
            var pressureVsFlowSeries = new LineSeries { Title = "Давление от расхода (головка)" };
            var tempVsFlowSeries = new LineSeries { Title = "Температура от расхода (головка)" };

            foreach (var result in _allHeadResults.OrderBy(r => r.VolumeExpenseSM3S))
            {
                pressureVsFlowSeries.Points.Add(new DataPoint(result.VolumeExpenseSM3S, result.SpecificPressureNearHeadMPa));
                tempVsFlowSeries.Points.Add(new DataPoint(result.VolumeExpenseSM3S, result.TemperatureMaterialAfterExtrusionC));
            }
            PressureVsFlowPlotModel.Series.Add(pressureVsFlowSeries);
            TemperatureVsFlowPlotModel.Series.Add(tempVsFlowSeries);

            // Инвалидация всех plot models
            RadialTemperaturePlotModel.InvalidatePlot(true);
            RadialViscosityPlotModel.InvalidatePlot(true);
            RadialVulcanizationPlotModel.InvalidatePlot(true);
            RadialTimeEqPlotModel.InvalidatePlot(true);
            PressureVsFlowPlotModel.InvalidatePlot(true);
            TemperatureVsFlowPlotModel.InvalidatePlot(true);
        }

        private void LoadData()
        {
            MessageBox.Show("Функция загрузки данных пока не реализована.", "Внимание", MessageBoxButton.OK, MessageBoxImage.Information);
        }

        private void SaveResults()
        {
            try
            {
                using (var writer = new StreamWriter("ExtrusionScrewResults.csv"))
                {
                    writer.WriteLine("Id,SectionOrder,PositionMM,PressureMPa,TemperatureC,BarrelTemperatureC,ScrewTemperatureC,ChannelHeightMM,ChannelWidthMM,ShearStressKPa,EnergyThermalMJm3,EnergyMechanicalMJm3,FlowRateDeltaSM3S,ShearDeformationAdded,ShearDeformationSummary,ShearRate,Viscosity,ResidenceTime,VulcanizationDegree");
                    foreach (var result in _allScrewResults)
                    {
                        writer.WriteLine($"{result.Id},{result.SectionOrder},{result.PositionMM},{result.PressureMPa},{result.TemperatureC},{result.BarrelTemperatureC},{result.ScrewTemperatureC},{result.ChannelHeightMM},{result.ChannelWidthMM},{result.ShearStressKPa},{result.EnergyThermalMJm3},{result.EnergyMechanicalMJm3},{result.FlowRateDeltaSM3S},{result.ShearDeformationAdded},{result.ShearDeformationSummary},{result.ShearRate},{result.Viscosity},{result.ResidenceTime},{result.VulcanizationDegree}");
                    }
                }
                using (var writer = new StreamWriter("ExtrusionHeadResults.csv"))
                {
                    writer.WriteLine("Id,VolumeExpenseSM3S,SpecificPressureNearHeadMPa,TemperatureMaterialEnteringHeadC,TemperatureMaterialAfterExtrusionC,VelocityMMin,CrossSectionAreaCM2,PartInductionPeriodScrewPercent,AdditionalPartInductionPeriodHeadPercent");
                    foreach (var result in _allHeadResults)
                    {
                        writer.WriteLine($"{result.Id},{result.VolumeExpenseSM3S},{result.SpecificPressureNearHeadMPa},{result.TemperatureMaterialEnteringHeadC},{result.TemperatureMaterialAfterExtrusionC},{result.VelocityMMin},{result.CrossSectionAreaCM2},{result.PartInductionPeriodScrewPercent},{result.AdditionalPartInductionPeriodHeadPercent}");
                    }
                }

                MessageBox.Show("Результаты сохранены в ExtrusionScrewResults.csv и ExtrusionHeadResults.csv.", "Успех", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Не удалось сохранить результаты: {ex.Message}", "Ошибка", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void SaveData()
        {
            MessageBox.Show("Функция сохранения данных пока не реализована.", "Внимание", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }
}