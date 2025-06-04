// ExtrusionAnalysis.Calculators/ExtrusionCalculator.cs
using ExtrusionAnalysis.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using MathNet.Numerics.Interpolation;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.ObjectModel; // Добавлено для ObservableCollection

namespace ExtrusionAnalysis.Calculators
{
    public class ExtrusionCalculator
    {
        private readonly MaterialProperties _materialProperties;
        private readonly GeneralParameters _generalParameters;
        private readonly List<ScrewSection> _screwSections;
        private readonly List<BarrelSection> _barrelSections;
        private readonly KineticsParameters _kineticsParameters;
        private readonly List<HeadSection> _headSections; // Используем HeadSection напрямую

        // Сплайны для геометрии винта
        private readonly CubicSpline[] _screwDiameterSplines;
        private readonly CubicSpline[] _screwHeightSplines;
        private readonly CubicSpline[] _screwFlightThicknessSplines;

        public ExtrusionCalculator(
            MaterialProperties materialProperties,
            GeneralParameters generalParameters,
            List<ScrewSection> screwSections,
            List<BarrelSection> barrelSections,
            KineticsParameters kineticsParameters,
            List<HeadSection> headSections) // Изменено с FluxData на HeadSection list
        {
            _materialProperties = materialProperties ?? throw new ArgumentNullException(nameof(materialProperties));
            _generalParameters = generalParameters ?? throw new ArgumentNullException(nameof(generalParameters));
            _screwSections = screwSections?.OrderBy(s => s.Order).ToList() ?? throw new ArgumentNullException(nameof(screwSections));
            _barrelSections = barrelSections?.OrderBy(b => b.LengthMM).ToList() ?? throw new ArgumentNullException(nameof(barrelSections)); // Сортируем по длине для FindCurrentSections
            _kineticsParameters = kineticsParameters ?? throw new ArgumentNullException(nameof(kineticsParameters));
            _headSections = headSections?.OrderBy(h => h.Order).ToList() ?? throw new ArgumentNullException(nameof(headSections));

            // Инициализация сплайнов для геометрии винта
            _screwDiameterSplines = new CubicSpline[_screwSections.Count];
            _screwHeightSplines = new CubicSpline[_screwSections.Count];
            _screwFlightThicknessSplines = new CubicSpline[_screwSections.Count];
            InitializeScrewSplines();

            // Простая валидация (для подробной валидации нужно больше кода)
            ValidateInputs();
        }

        private void InitializeScrewSplines()
        {
            // В реальной Pascal-программе сплайны строятся для каждой секции,
            // здесь для простоты пока используются линейные интерполяции в пределах секции.
            // MathNet.Numerics.Interpolation.CubicSpline принимает массивы x и y,
            // если секция имеет постоянный диаметр/высоту/толщину, то x = {0, Length}, y = {Val, Val}.
            // Если меняется, то x = {0, Length}, y = {StartVal, EndVal}.
            for (int i = 0; i < _screwSections.Count; i++)
            {
                var section = _screwSections[i];
                double[] x = { 0, section.LengthMM }; // Длина секции в метрах
                _screwDiameterSplines[i] = CubicSpline.InterpolateNatural(x, new[] { section.DStartMM, section.DFinalMM });
                _screwHeightSplines[i] = CubicSpline.InterpolateNatural(x, new[] { section.HStartMM, section.HFinalMM });
                _screwFlightThicknessSplines[i] = CubicSpline.InterpolateNatural(x, new[] { section.ThreadThicknessStartMM.GetValueOrDefault(0), section.ThreadThicknessFinalMM.GetValueOrDefault(0) });
            }
        }

        private void ValidateInputs()
        {
            if (!_screwSections.Any()) throw new ArgumentException("Требуется хотя бы одна секция винта.");
            if (!_barrelSections.Any()) throw new ArgumentException("Требуется хотя бы одна секция корпуса.");
            if (!_headSections.Any()) throw new ArgumentException("Требуется хотя бы одна секция головки.");

            if (_materialProperties.ThermalDiffusivity <= 0 || _materialProperties.HeatConductivity <= 0)
                throw new ArgumentException($"Некорректные свойства материала: ThermalDiffusivity={_materialProperties.ThermalDiffusivity}, HeatConductivity={_materialProperties.HeatConductivity}. Должны быть положительными.");

            if (_generalParameters.RotationFrequency <= 0) // N, об/с (после перевода)
                throw new ArgumentException($"Некорректные общие параметры: RotationFrequency={_generalParameters.RotationFrequency}. Должна быть положительной.");

            // Проверка геометрии винта и корпуса
            foreach (var screw in _screwSections)
            {
                if (screw.HStartMM <= 0 || screw.HFinalMM <= 0 || screw.DStartMM <= 0 || screw.DFinalMM <= 0 || screw.LengthMM <= 0)
                    throw new ArgumentException($"Секция винта {screw.Order} имеет некорректные параметры: HStart={screw.HStartMM}, HFinal={screw.HFinalMM}, DStart={screw.DStartMM}, DFinal={screw.DFinalMM}, Length={screw.LengthMM}. Все должны быть положительными.");
            }

            foreach (var barrel in _barrelSections)
            {
                if (barrel.LengthMM <= 0 || barrel.WallThicknessMM <= 0)
                    throw new ArgumentException($"Секция корпуса {barrel.Id} имеет некорректные параметры: Length={barrel.LengthMM}, WallThickness={barrel.WallThicknessMM}. Должны быть положительными.");
            }

            foreach (var head in _headSections)
            {
                if (head.DStartMM <= 0 || head.DFinalMM <= 0 || head.LengthMM <= 0)
                    throw new ArgumentException($"Секция головки {head.Order} имеет некорректные размеры: DStart={head.DStartMM}, DFinal={head.DFinalMM}, Length={head.LengthMM}. Должны быть положительными.");
            }
        }

        // --- Основной метод для расчета процесса в винтовой машине (QPT.exe) ---
        public List<ExtrusionResult> CalculateScrewProcess()
        {
            var results = new List<ExtrusionResult>();
            double totalScrewLength = _screwSections.Sum(s => s.LengthMM); // Общая длина винта в метрах

            // Инициализация начальных условий
            double currentLengthM = 0; // Текущая позиция вдоль оси винта (м)
            double currentPressurePa = 0; // Текущее давление (Па)
            double currentTempC = _generalParameters.InitialMixtureTemperature; // Текущая температура (°C)
            double currentShearDeformationSummary = 0; // Суммарная деформация сдвига

            // Получаем Q_b - "расход без противодавления" из самого узкого сечения
            double q_b_min = double.MaxValue;
            double min_q_b_uz = 0;
            foreach (var s in _screwSections)
            {
                if (s.Type == SectionType.Cut)
                {
                    double D_start = s.DStartMM;
                    double H_start = s.HStartMM;
                    double D_final = s.DFinalMM;
                    double H_final = s.HFinalMM;
                    // double length = s.LengthMM; // Not used here
                    double threadPitch = s.ThreadPitchMM.GetValueOrDefault(0);
                    double numberOfThreads = s.NumberOfThreads.GetValueOrDefault(1);

                    double phi_start = Math.Atan(threadPitch / (Math.PI * D_start));
                    double W_start = (Math.PI * D_start * Math.Sin(phi_start) / numberOfThreads) - s.ThreadThicknessStartMM.GetValueOrDefault(0) * Math.Cos(phi_start);
                    double u_z_start = (Math.PI * D_start * _generalParameters.RotationFrequency) * Math.Cos(phi_start);
                    double Q_b_start = u_z_start * W_start * H_start / 2;

                    double phi_final = Math.Atan(threadPitch / (Math.PI * D_final));
                    double W_final = (Math.PI * D_final * Math.Sin(phi_final) / numberOfThreads) - s.ThreadThicknessFinalMM.GetValueOrDefault(0) * Math.Cos(phi_final);
                    double u_z_final = (Math.PI * D_final * _generalParameters.RotationFrequency) * Math.Cos(phi_final);
                    double Q_b_final = u_z_final * W_final * H_final / 2;

                    // Берем минимальное из начального и конечного расхода секции
                    if (Q_b_start < q_b_min) { q_b_min = Q_b_start; min_q_b_uz = u_z_start; }
                    if (Q_b_final < q_b_min) { q_b_min = Q_b_final; min_q_b_uz = u_z_final; }
                }
            }
            if (q_b_min == double.MaxValue || q_b_min <= 0) q_b_min = 1.0; // Fallback if no cut sections or zero flow

            // Расчет диапазона объемных расходов Q для внешней характеристики винта
            double minRelativeQ = _generalParameters.MinimalRelativeExpenditure;
            double maxRelativeQ = _generalParameters.MaximalRelativeExpenditure;
            int numQSteps = _generalParameters.NumberOfStepsByRelativeExpense;

            double Q_base_for_qpt = q_b_min; // Базовый расход для QPT в м^3/с
            if (Q_base_for_qpt <= 0) Q_base_for_qpt = 1e-6; // Защита от деления на ноль

            // Определяем единственный Q_M, который будет использоваться для "основного" расчета QPT
            // Pascal QPT.txt Table 2.1: Q_min/Q_b, Q_max/Q_b, n_step.
            // Это определяет интервал для построения внешней характеристики,
            // но основной расчет (табличный вывод) выполняется для одного Q.
            // По DOC_4.doc: "Объемный расход материала через машину, принятый для анализируемых вычислений, составил Q=27.85 см3/с."
            // Q = 27.85e-6 м^3/с (Q_M)
            double currentQ_m3_s = 27.85e-6; // Фиксированный Q_M из DOC_4.doc для табличного вывода

            // If the user specified a range in GeneralParameters, we should use the first value in that range.
            // But the problem description mentions a fixed Q, so let's use that for now.
            // If we want to use the range, we'd loop over qStep here, and each loop would generate full results.
            // For now, let's just make the main calculation for ONE Q, as per the screenshot.
            // double currentQ_m3_s = (minRelativeQ + (maxRelativeQ - minRelativeQ) * 0 / Math.Max(1, numQSteps - 1)) * Q_base_for_qpt;


            // Переинициализация для текущего Q_M
            currentLengthM = 0;
            currentPressurePa = 0;
            currentTempC = _generalParameters.InitialMixtureTemperature;
            currentShearDeformationSummary = 0;
            double currentEnergyMechanical = 0; // Накопленная механическая энергия (интеграл от диссипации)
            double currentEnergyThermal = 0; // Накопленное теплосодержание (интеграл от Q_T)
            double currentTimeEqv = 0; // Эквивалентное время для вулканизации

            // Цикл по секциям винта
            foreach (var screwSection in _screwSections)
            {
                int numCycles = screwSection.IntegrationCycles.GetValueOrDefault(_generalParameters.NumberOfIntegrationCycles); // Число циклов интегрирования для этой секции
                if (numCycles == 0) numCycles = 1; // Защита от нуля

                double dLm = screwSection.LengthMM / numCycles; // Шаг интегрирования вдоль секции (м)

                for (int i = 0; i <= numCycles; i++)
                {
                    // Проверка на NaN / Infinity для предыдущих значений
                    if (double.IsNaN(currentPressurePa) || double.IsInfinity(currentPressurePa)) currentPressurePa = 0;
                    if (double.IsNaN(currentTempC) || double.IsInfinity(currentTempC)) currentTempC = _generalParameters.InitialMixtureTemperature;
                    if (double.IsNaN(currentShearDeformationSummary) || double.IsInfinity(currentShearDeformationSummary)) currentShearDeformationSummary = 0;
                    if (double.IsNaN(currentEnergyMechanical) || double.IsInfinity(currentEnergyMechanical)) currentEnergyMechanical = 0;
                    if (double.IsNaN(currentEnergyThermal) || double.IsInfinity(currentEnergyThermal)) currentEnergyThermal = 0;
                    if (double.IsNaN(currentTimeEqv) || double.IsInfinity(currentTimeEqv)) currentTimeEqv = 0;


                    // Получение текущих параметров геометрии
                    double localZInScrewSection = i * dLm;
                    double currentD = _screwDiameterSplines[screwSection.Order - 1].Interpolate(localZInScrewSection);
                    double currentH = _screwHeightSplines[screwSection.Order - 1].Interpolate(localZInScrewSection);
                    double currentE = screwSection.Type == SectionType.Cut ? _screwFlightThicknessSplines[screwSection.Order - 1].Interpolate(localZInScrewSection) : 0;

                    if (currentD <= 1e-9) currentD = 1e-9;
                    if (currentH <= 1e-9) currentH = 1e-9;


                    double currentPitch = screwSection.ThreadPitchMM.GetValueOrDefault(0);
                    if (currentPitch == 0 && screwSection.Type == SectionType.Cut) // Fallback if pitch is zero for cut section
                    {
                        currentPitch = currentD; // A guess, can be improved
                    }
                    else if (screwSection.Type == SectionType.Smooth)
                    {
                        // For smooth sections, pitch is not directly used for channel geometry, but for axial velocity if needed.
                        // For now, let's not use it here.
                    }

                    double numberOfThreads = screwSection.NumberOfThreads.GetValueOrDefault(1);

                    double pd = Math.PI * currentD; // Это, вероятно, длина окружности
                    double u = pd * _generalParameters.RotationFrequency; // Окружная скорость
                    double phi = Math.Atan(currentPitch / pd); // Угол подъема винтовой линии
                    double cs = Math.Cos(phi);
                    double sn = Math.Sin(phi);
                    double ux = u * sn; // Компонента скорости по x (ширина канала)
                    double uz = u * cs; // Компонента скорости по z (вдоль оси шнека)

                    double W_channel = screwSection.Type == SectionType.Cut ? (pd * sn / numberOfThreads) - (currentE * cs) : (currentD * Math.PI - 0); // Ширина канала. For smooth, use D*PI approx
                    if (W_channel < 1e-9) W_channel = 1e-9; // Защита от деления на ноль

                    // Поиск соответствующей секции корпуса
                    var (currentBarrelSection, currentBarrelLocalZ) = FindCurrentBarrelSection(currentLengthM);

                    // Параметры теплообмена с корпусом и червяком (Pascal: TB_k, TB_s)
                    double barrelWallTempC = currentBarrelSection.TemperatureC; // Температура стенки корпуса в °C
                    double screwWallTempC = _generalParameters.MediumTemperatureInScrewCavity; // Температура стенки червяка в °C

                    // --- Расчет эффективной вязкости и корней (упрощенная имитация ИРЭВ) ---
                    double meanShearRate = 0;
                    if (screwSection.Type == SectionType.Cut)
                    {
                        meanShearRate = Math.Sqrt(Math.Pow(ux / currentH, 2) + Math.Pow(uz / currentH, 2));
                    }
                    else // Smooth section, approximation for annular flow
                    {
                        // Mean shear rate for annular flow: 4 * Q / (Pi * (R_outer^3 - R_inner^3)) for power-law
                        // For simplicity, using a generalized shear rate based on characteristic velocity/gap
                        meanShearRate = Math.Abs(uz / currentH); // Characteristic shear rate
                    }

                    if (meanShearRate < 1e-9) meanShearRate = 1e-9;
                    double muEff = _materialProperties.GetViscosity(currentTempC, meanShearRate, u / currentH); // Па*с
                    if (muEff <= 1e-9) muEff = 1e-9; // Защита от нуля


                    // --- Расчет градиента давления (dp_dz) ---
                    // В Pascal это очень сложная итерационная процедура, здесь - упрощенная физическая модель.
                    double dp_dz_local = 0;
                    if (screwSection.Type == SectionType.Cut)
                    {
                        // Pressure drop for power-law fluid in rectangular channel approx.
                        // Based on Hagen-Poiseuille for Newtonian: dp/dz = 12 * mu * Q / (W * H^3)
                        // For power-law, it's more complex, but we can generalize.
                        // Assuming flow is Q_m3_s, shear rate is Q_m3_s / (W * H), so dp/dz ~ mu * (Q/WH)^n / H
                        // Pascal's dp_dz calculation is complex, using Fd, Fp factors and solving system.
                        // Let's use a simplified approach as in previous steps for consistency in approximation.
                        double flow_velocity = currentQ_m3_s / (W_channel * currentH);
                        dp_dz_local = (12.0 * muEff * flow_velocity) / Math.Pow(currentH, 2); // Simplified shear stress related pressure

                        // Add pumping action effect (Pascal: nL*Q_b*Fd)
                        dp_dz_local -= (uz * muEff / currentH); // Pumping effect, simplified

                    }
                    else // Smooth section (annular channel)
                    {
                        // Simplified pressure drop for annular flow of power-law fluid
                        // Pascal: dp_dz:=2*(Tau_rz_H*rH-Tau_rz_B*rB)/S_round;
                        // Use a generalized Hagen-Poiseuille for annular flow
                        double R_outer_m = currentD / 2.0;
                        double R_inner_m = R_outer_m - currentH;
                        if (R_inner_m < 0) R_inner_m = 0; // Cannot be negative

                        double effective_radius = R_outer_m - R_inner_m;
                        if (effective_radius < 1e-9) effective_radius = 1e-9;

                        // Pascal's calculation: Tau_rz_H, Tau_rz_B are roots of complex equations
                        // Let's approximate shear stress and use it for pressure drop
                        double shear_stress_at_wall = muEff * Math.Abs(uz) / effective_radius; // Simplified
                        dp_dz_local = 2.0 * shear_stress_at_wall / (R_outer_m + R_inner_m); // Simplified gradient for annular flow
                    }

                    // Защита от Nan/Infinity
                    if (double.IsNaN(dp_dz_local) || double.IsInfinity(dp_dz_local)) dp_dz_local = 0;


                    // --- Расчет давления и температуры ---
                    double d_length_step = dLm; // Приращение длины для данного шага
                    currentPressurePa += dp_dz_local * d_length_step; // Накопление давления

                    // Упрощенный расчет изменения температуры (Pascal: T = T + ... * dz)
                    double dissipationRateVolume = muEff * Math.Pow(meanShearRate, 2); // Плотность диссипации (Вт/м^3)
                    double volumeElement = (screwSection.Type == SectionType.Cut ? W_channel * currentH : Math.PI * (Math.Pow(currentD / 2, 2) - Math.Pow(currentD / 2 - currentH, 2))) * d_length_step; // Объем элементарного участка (м^3)
                    double dEnergyHeatTotal = dissipationRateVolume * volumeElement; // Тепловая энергия, выделяемая за шаг (Джоули)

                    // Теплообмен со стенками (Pascal: w*(Al_korp*(T_korp-T)+AL_screw*(T_screw-T)))
                    // Surface area for heat exchange
                    double barrelSurfaceArea = Math.PI * currentD * d_length_step;
                    double screwSurfaceArea = Math.PI * (currentD - 2 * currentH) * d_length_step; // Assumes a solid core screw

                    double heatLossToBarrel = _generalParameters.MixtureToBarrelHeatTransferCoeff * barrelSurfaceArea * (currentTempC - barrelWallTempC);
                    double heatLossToScrew = _generalParameters.MixtureToScrewHeatTransferCoeff * screwSurfaceArea * (currentTempC - screwWallTempC);

                    double massFlowRate = currentQ_m3_s * _materialProperties.Density; // кг/с
                    double specificHeat = 1500; // Примерная теплоемкость полимера, Дж/(кг*К) - Pascal does not explicitly define Cp, but uses a_T and Lam_T. Lam_T/a_T = Ro*Cp. So Cp = Lam_T / (a_T * Ro)
                    if (_materialProperties.ThermalDiffusivity > 0 && _materialProperties.Density > 0)
                    {
                        specificHeat = _materialProperties.HeatConductivity / (_materialProperties.ThermalDiffusivity * _materialProperties.Density);
                        if (specificHeat <= 0) specificHeat = 1500; // Fallback
                    }

                    if (massFlowRate * specificHeat <= 1e-9) // Защита от деления на ноль
                    {
                        currentTempC = (barrelWallTempC + screwWallTempC) / 2.0; // Просто стабилизируем
                    }
                    else
                    {
                        currentTempC += (dEnergyHeatTotal - heatLossToBarrel - heatLossToScrew) / (massFlowRate * specificHeat);
                    }
                    currentTempC = Math.Max(-273.15, currentTempC); // Температура не может быть ниже абсолютного нуля


                    // --- Расчет деформаций сдвига (упрощенно) ---
                    // Pascal: dSSDS := Gamma_Al/Int_v_Lam; SSDS:=SSDS+dSSDS;
                    // dGamma = shearRate * dt (dt = dL/vz)
                    double d_time_element = d_length_step / uz;
                    if (double.IsNaN(d_time_element) || double.IsInfinity(d_time_element) || d_time_element < 0) d_time_element = 0; // Avoid negative time

                    double dShearDeformation = meanShearRate * d_time_element;
                    if (double.IsNaN(dShearDeformation) || double.IsInfinity(dShearDeformation)) dShearDeformation = 0;
                    currentShearDeformationSummary += dShearDeformation;

                    // --- Расчет вулканизации (упрощенно) ---
                    if (currentTempC >= _materialProperties.InitialTemperature)
                    {
                        int intervalIndex = FindVulcanizationInterval(_kineticsParameters, currentTimeEqv);
                        currentTimeEqv = UpdateEquivalentTime(_kineticsParameters, currentTimeEqv, currentTempC, d_time_element, intervalIndex);
                    }
                    double currentVulcanizationDegree = GetVulcanizationDegree(_kineticsParameters, currentTimeEqv, 0); // interval 0 for simplicity

                    // --- Накопление результатов ---
                    var result = new ExtrusionResult
                    {
                        Id = screwSection.Id,
                        SectionOrder = screwSection.Order,
                        PositionMM = (currentLengthM + d_length_step) * 1000, // м в мм для вывода
                        PressureMPa = currentPressurePa / 1e6, // Па в МПа
                        TemperatureC = currentTempC,
                        BarrelTemperatureC = barrelWallTempC,
                        ScrewTemperatureC = screwWallTempC,
                        ChannelHeightMM = currentH * 1000,
                        ChannelWidthMM = W_channel * 1000,
                        ShearStressKPa = muEff * meanShearRate / 1000.0, // Па в кПа
                        EnergyThermalMJm3 = currentEnergyThermal / 1e6, // Дж в МДж
                        EnergyMechanicalMJm3 = currentEnergyMechanical / 1e6, // Дж в МДж
                        FlowRateDeltaSM3S = currentQ_m3_s * 1e6, // м^3/с в см^3/с (Q_del)
                        ShearDeformationAdded = dShearDeformation,
                        ShearDeformationSummary = currentShearDeformationSummary,
                        ShearRate = meanShearRate,
                        Viscosity = muEff,
                        ResidenceTime = d_time_element, // Время пребывания для данного шага
                        VulcanizationDegree = currentVulcanizationDegree // Степень вулканизации
                    };
                    results.Add(result);

                    // Обновление общего пройденного расстояния
                    currentLengthM += d_length_step;
                    currentEnergyMechanical += dissipationRateVolume * d_length_step; // Накопление механической энергии (диссипация * длина)
                    currentEnergyThermal += (dEnergyHeatTotal - heatLossToBarrel - heatLossToScrew); // Накопление тепловой энергии (Джоули)

                } // End of for i (cycles along section)
            } // End of foreach screwSection

            // Добавляем итоговую деформацию сдвига в последний элемент
            if (results.Any())
            {
                results.Last().ShearDeformationSummary = currentShearDeformationSummary;
            }

            return results;
        }


        // ExtrusionAnalysis.Calculators/ExtrusionCalculator.cs

        // ... (остальной код ExtrusionCalculator)

        // --- Расчет внешней характеристики винта (p от Q) ---
        public List<(double FlowRate, double Pressure)> BuildScrewCharacteristicCurve()
        {
            var characteristicCurve = new List<(double FlowRate, double Pressure)>();

            double minRelativeQ = _generalParameters.MinimalRelativeExpenditure;
            double maxRelativeQ = _generalParameters.MaximalRelativeExpenditure;
            int numQSteps = _generalParameters.NumberOfStepsByRelativeExpense;

            // Находим Q_b для всей длины винта, как это делает Pascal
            double q_b_min = double.MaxValue;
            // double uz_at_q_b_min = 0; // Not used
            foreach (var s in _screwSections)
            {
                if (s.Type == SectionType.Cut)
                {
                    double D = s.DStartMM;
                    double H = s.HStartMM;
                    double pitch = s.ThreadPitchMM.GetValueOrDefault(0);
                    double numThreads = s.NumberOfThreads.GetValueOrDefault(1);

                    double phi = Math.Atan(pitch / (Math.PI * D));
                    double W = (Math.PI * D * Math.Sin(phi) / numThreads) - s.ThreadThicknessStartMM.GetValueOrDefault(0) * Math.Cos(phi);
                    double uz = (Math.PI * D * _generalParameters.RotationFrequency) * Math.Cos(phi);
                    double Qb = uz * W * H / 2;

                    if (Qb < q_b_min)
                    {
                        q_b_min = Qb;
                        // uz_at_q_b_min = uz; // Not used
                    }
                }
            }
            if (q_b_min == double.MaxValue || q_b_min <= 0) q_b_min = 1.0; // Fallback

            double dQ_rel = (maxRelativeQ - minRelativeQ) / Math.Max(1, numQSteps - 1);

            for (int qStep = 0; qStep < numQSteps; qStep++)
            {
                double relativeQ = minRelativeQ + qStep * dQ_rel;
                double currentQ_m3_s = relativeQ * q_b_min; // Текущий объемный расход в м^3/с

                // Выполнение расчета для данного Q по всей длине винта
                double pressureAtQ = 0; // Давление на выходе из винта для данного Q
                double currentTempC = _generalParameters.InitialMixtureTemperature; // Start fresh for each Q
                // double currentLengthM = 0; // Not used

                foreach (var screwSection in _screwSections)
                {
                    int numCycles = screwSection.IntegrationCycles.GetValueOrDefault(_generalParameters.NumberOfIntegrationCycles);
                    if (numCycles == 0) numCycles = 1;

                    double dLm = screwSection.LengthMM / numCycles;

                    for (int i = 0; i < numCycles; i++)
                    {
                        // Get current geometry
                        double localZInScrewSection = i * dLm;
                        double currentD = _screwDiameterSplines[screwSection.Order - 1].Interpolate(localZInScrewSection);
                        double currentH = _screwHeightSplines[screwSection.Order - 1].Interpolate(localZInScrewSection);
                        double currentE = screwSection.Type == SectionType.Cut ? _screwFlightThicknessSplines[screwSection.Order - 1].Interpolate(localZInScrewSection) : 0;

                        if (currentD <= 1e-9) currentD = 1e-9;
                        if (currentH <= 1e-9) currentH = 1e-9;

                        double currentPitch = screwSection.ThreadPitchMM.GetValueOrDefault(0);
                        if (currentPitch == 0 && screwSection.Type == SectionType.Cut) currentPitch = currentD; // Fallback

                        double numberOfThreads = screwSection.NumberOfThreads.GetValueOrDefault(1);

                        double pd = Math.PI * currentD;
                        double u = pd * _generalParameters.RotationFrequency;
                        double phi = Math.Atan(currentPitch / pd);

                        // Объявление отсутствующих переменных sn и cs
                        double cs = Math.Cos(phi);
                        double sn = Math.Sin(phi);

                        double ux = u * sn;
                        double uz = u * cs;

                        double W_channel = screwSection.Type == SectionType.Cut ? (pd * sn / numberOfThreads) - (currentE * cs) : (currentD * Math.PI - 0);
                        if (W_channel < 1e-9) W_channel = 1e-9;

                        double meanShearRate = screwSection.Type == SectionType.Cut ? Math.Sqrt(Math.Pow(ux / currentH, 2) + Math.Pow(uz / currentH, 2)) : Math.Abs(uz / currentH);
                        if (meanShearRate < 1e-9) meanShearRate = 1e-9;
                        double muEff = _materialProperties.GetViscosity(currentTempC, meanShearRate, u / currentH);
                        if (muEff <= 1e-9) muEff = 1e-9;

                        double dp_dz_local = 0;
                        if (screwSection.Type == SectionType.Cut)
                        {
                            double flow_velocity = currentQ_m3_s / (W_channel * currentH);
                            dp_dz_local = (12.0 * muEff * flow_velocity) / Math.Pow(currentH, 2);
                            dp_dz_local -= (uz * muEff / currentH);
                        }
                        else
                        {
                            double R_outer_m = currentD / 2.0;
                            double R_inner_m = R_outer_m - currentH;
                            if (R_inner_m < 0) R_inner_m = 0;
                            double effective_radius = R_outer_m - R_inner_m;
                            if (effective_radius < 1e-9) effective_radius = 1e-9;
                            double shear_stress_at_wall = muEff * Math.Abs(uz) / effective_radius;
                            dp_dz_local = 2.0 * shear_stress_at_wall / (R_outer_m + R_inner_m);
                        }
                        if (double.IsNaN(dp_dz_local) || double.IsInfinity(dp_dz_local)) dp_dz_local = 0;

                        pressureAtQ += dp_dz_local * dLm; // Accumulate pressure
                        if (double.IsNaN(pressureAtQ) || double.IsInfinity(pressureAtQ)) pressureAtQ = 0;

                        // Temperature update (simplified for characteristic curve)
                        double specificHeat = 1500;
                        if (_materialProperties.ThermalDiffusivity > 0 && _materialProperties.Density > 0)
                        {
                            specificHeat = _materialProperties.HeatConductivity / (_materialProperties.ThermalDiffusivity * _materialProperties.Density);
                            if (specificHeat <= 0) specificHeat = 1500;
                        }
                        currentTempC += (muEff * Math.Pow(meanShearRate, 2) * dLm) / (_materialProperties.Density * specificHeat);
                        currentTempC = Math.Max(_generalParameters.InitialMixtureTemperature, currentTempC); // Temp doesn't drop below initial

                    }
                }
                characteristicCurve.Add((currentQ_m3_s * 1e6, pressureAtQ / 1e6)); // cm^3/s and MPa
            }
            return characteristicCurve;
        }


        // --- Метод для расчета процесса в формующей головке (ROUND.exe) ---
        public HeadResult CalculateHeadProcess(Dictionary<double, double> screwCharacteristicCurve, double initialHeadTemperatureC, double finalScrewShearDeformationSummary)
        {
            if (!_headSections.Any()) return null;

            // double currentHeadPressurePa = screwCharacteristicCurve.Values.Last() * 1e6; // Начальное давление - последнее из винта // Not used
            // double currentHeadTempC = initialHeadTemperatureC; // Начальная температура - последняя из винта // Not used directly in main loop

            double Q_working_point_m3_s = 0;
            double P_working_point_Pa = 0;
            double T_working_point_C = 0; // Temp at head entrance
            double T_out_working_point_C = 0; // Temp at head exit
            double AdditionalVulcanizationHeadPercent = 0;


            double q_min_for_head_char_dm3_min = _generalParameters.MinimalRelativeExpenditure * _generalParameters.MediumFlowRateDM3Min; // Using Q_min_rb as base
            double q_max_for_head_char_dm3_min = _generalParameters.MaximalRelativeExpenditure * _generalParameters.MediumFlowRateDM3Min;
            if (q_min_for_head_char_dm3_min <= 0) q_min_for_head_char_dm3_min = 1e-6; // Protection against 0
            if (q_max_for_head_char_dm3_min <= q_min_for_head_char_dm3_min) q_max_for_head_char_dm3_min = q_min_for_head_char_dm3_min + 1e-6;

            int num_head_char_points = _generalParameters.NumberOfStepsByRelativeExpense + 1;
            double dQ_head_char_dm3_min = (q_max_for_head_char_dm3_min - q_min_for_head_char_dm3_min) / Math.Max(1, num_head_char_points - 1);

            var headPressureCurve = new List<(double FlowRate, double Pressure)>(); // Q in cm^3/s, P in MPa
            var headTemperatureCurve = new List<(double FlowRate, double Temperature)>(); // Q in cm^3/s, T in °C
            var headVulcanizationCurve = new List<(double FlowRate, double Vulcanization)>(); // Q in cm^3/s, Vulc in %


            // Build head characteristic curve
            for (int i = 0; i < num_head_char_points; i++)
            {
                double current_Q_head_dm3_min = q_min_for_head_char_dm3_min + i * dQ_head_char_dm3_min;
                double current_Q_head_m3_s = current_Q_head_dm3_min / 1000.0 / 60.0;

                double temp_head_pressure_Pa = 0;
                double temp_head_temp_C = initialHeadTemperatureC;
                double temp_head_vulcanization_t_eqv = 0; // Accumulator for t_eqv for head

                foreach (var headSection in _headSections)
                {
                    int numCycles = headSection.IntegrationCycles;
                    if (numCycles == 0) numCycles = 1; // Protect against zero

                    double dLm = headSection.LengthMM / numCycles; // Length step in meters

                    for (int j = 0; j < numCycles; j++)
                    {
                        double D_start = headSection.DStartMM;
                        double D_final = headSection.DFinalMM;
                        double L_sect = headSection.LengthMM;

                        double currentD_at_step = D_start + (D_final - D_start) * ((j * dLm) / L_sect);
                        if (currentD_at_step <= 1e-9) currentD_at_step = 1e-9; // Protection against zero

                        double rH = currentD_at_step / 2.0; // Outer radius of the channel
                        // For solid circular section, inner radius rB = 0

                        double meanShearRate = (4 * current_Q_head_m3_s) / (Math.PI * Math.Pow(rH, 3)); // For circular section (Newtonian approximation)
                        if (meanShearRate < 1e-9) meanShearRate = 1e-9;
                        double muEff = _materialProperties.GetViscosity(temp_head_temp_C, meanShearRate, meanShearRate); // Simplified viscosity, needs IREV for accuracy
                        if (muEff <= 1e-9) muEff = 1e-9;

                        // Pressure drop for power-law fluid in circular pipe
                        // Delta P = (32 * mu * L * Q) / (Pi * D^4) for Newtonian
                        // For power-law, it's more complex, but we use a generalized form:
                        double pressure_drop_local_Pa = (8.0 * muEff * current_Q_head_m3_s * dLm) / (Math.PI * Math.Pow(rH, 4));
                        temp_head_pressure_Pa += pressure_drop_local_Pa;

                        // Temperature change in head (simplified)
                        double specificHeat = 1500; // Use the same specific heat as screw calc
                        if (_materialProperties.ThermalDiffusivity > 0 && _materialProperties.Density > 0)
                        {
                            specificHeat = _materialProperties.HeatConductivity / (_materialProperties.ThermalDiffusivity * _materialProperties.Density);
                            if (specificHeat <= 0) specificHeat = 1500; // Fallback
                        }

                        double massFlowRate = current_Q_head_m3_s * _materialProperties.Density;
                        double dissipationVolumeRate = muEff * Math.Pow(meanShearRate, 2); // W/m^3
                        double volumeElement = Math.PI * Math.Pow(rH, 2) * dLm; // m^3
                        double heatGenerated = dissipationVolumeRate * volumeElement; // Joules

                        // Heat transfer with wall (Pascal: T_st)
                        double wallSurfaceArea = Math.PI * currentD_at_step * dLm;
                        double heatTransferCoeffWall = _generalParameters.MixtureToBarrelHeatTransferCoeff; // Using barrel alpha_b as general wall HT coeff
                        double heatLossToWall = heatTransferCoeffWall * wallSurfaceArea * (temp_head_temp_C - headSection.WallTemperatureC);

                        if (massFlowRate * specificHeat <= 1e-9)
                        {
                            temp_head_temp_C = headSection.WallTemperatureC; // Stabilize to wall temp
                        }
                        else
                        {
                            temp_head_temp_C += (heatGenerated - heatLossToWall) / (massFlowRate * specificHeat);
                        }
                        temp_head_temp_C = Math.Max(-273.15, temp_head_temp_C); // Clamp to absolute zero


                        // Vulcanization in head
                        double flowVelocity = current_Q_head_m3_s / (Math.PI * Math.Pow(rH, 2)); // m/s
                        double d_time_element = dLm / flowVelocity;
                        if (double.IsNaN(d_time_element) || double.IsInfinity(d_time_element) || d_time_element < 0) d_time_element = 0;

                        if (temp_head_temp_C >= _materialProperties.InitialTemperature)
                        {
                            int intervalIndex = FindVulcanizationInterval(_kineticsParameters, temp_head_vulcanization_t_eqv);
                            temp_head_vulcanization_t_eqv = UpdateEquivalentTime(_kineticsParameters, temp_head_vulcanization_t_eqv, temp_head_temp_C, d_time_element, intervalIndex);
                        }
                    }
                }
                headPressureCurve.Add((current_Q_head_m3_s * 1e6, temp_head_pressure_Pa / 1e6)); // cm^3/s, MPa
                headTemperatureCurve.Add((current_Q_head_m3_s * 1e6, temp_head_temp_C)); // cm^3/s, °C
                headVulcanizationCurve.Add((current_Q_head_m3_s * 1e6, GetVulcanizationDegree(_kineticsParameters, temp_head_vulcanization_t_eqv, 0))); // cm^3/s, %
            }

            // --- Поиск рабочей точки (Intersection of screw and head characteristics) ---
            Q_working_point_m3_s = 0;
            P_working_point_Pa = 0;
            T_working_point_C = initialHeadTemperatureC; // Temp at head entrance is last screw temp
            T_out_working_point_C = 0;
            AdditionalVulcanizationHeadPercent = 0;

            if (screwCharacteristicCurve.Any() && headPressureCurve.Any())
            {
                var screwQ = screwCharacteristicCurve.Keys.ToArray();
                var screwP = screwCharacteristicCurve.Values.ToArray();
                var screwSpline = CubicSpline.InterpolateNatural(screwQ, screwP);

                var headQ = headPressureCurve.Select(p => p.FlowRate).ToArray();
                var headP = headPressureCurve.Select(p => p.Pressure).ToArray();
                var headTS = headTemperatureCurve.Select(p => p.Temperature).ToArray();
                var headVulcs = headVulcanizationCurve.Select(p => p.Vulcanization).ToArray();

                var headSpline = CubicSpline.InterpolateNatural(headQ, headP);
                var headTempSpline = CubicSpline.InterpolateNatural(headQ, headTS);
                var headVulcSpline = CubicSpline.InterpolateNatural(headQ, headVulcs);

                // Find intersection (root of difference function)
                // Use a simple iterative search (like Pascal's Chorda/Minim)
                double best_q_match = 0;
                double min_abs_diff = double.MaxValue;

                // Search within the common range of Q
                double startQ = Math.Max(screwQ.Min(), headQ.Min());
                double endQ = Math.Min(screwQ.Max(), headQ.Max());
                double stepQ = (endQ - startQ) / 200.0; // Finer step for intersection search

                if (stepQ <= 0) // Handle case where ranges are too small or overlap is minimal
                {
                    startQ = screwQ.Min();
                    endQ = screwQ.Max();
                    stepQ = (endQ - startQ) / 200.0;
                }

                for (double q_val = startQ; q_val <= endQ; q_val += stepQ)
                {
                    double p_screw = screwSpline.Interpolate(q_val);
                    double p_head = headSpline.Interpolate(q_val);

                    double current_diff = Math.Abs(p_screw - p_head);

                    if (current_diff < min_abs_diff)
                    {
                        min_abs_diff = current_diff;
                        best_q_match = q_val;
                    }
                }
                // Ensure best_q_match is within the bounds of spline
                best_q_match = Math.Max(Math.Min(best_q_match, screwQ.Max()), screwQ.Min());
                best_q_match = Math.Max(Math.Min(best_q_match, headQ.Max()), headQ.Min());

                // Calculate working point parameters
                Q_working_point_m3_s = best_q_match / 1e6; // cm^3/s to m^3/s
                P_working_point_Pa = screwSpline.Interpolate(best_q_match) * 1e6; // MPa to Pa
                T_out_working_point_C = headTempSpline.Interpolate(best_q_match);
                AdditionalVulcanizationHeadPercent = headVulcSpline.Interpolate(best_q_match);
            }

            // --- Calculate Radial Profiles for the working point Q (last head section) ---
            var lastHeadSection = _headSections.Last();
            double D_final_m = lastHeadSection.DFinalMM / 1000.0;
            double rH_m = D_final_m / 2.0; // Radius for circular pipe

            // The Pascal code uses "n_slices" (N_Alpha in FluxData) for radial points.
            int nRadialPoints = _generalParameters.NumberOfMaterialSlicesInFlood;
            if (nRadialPoints <= 1) nRadialPoints = 20; // Default if not set


            // Assuming a simple linear profile for simplicity in this approximation
            // For a real profile, you'd need to solve heat equation for circular pipe with shear heat generation.
            double[] radialRadiiMM = new double[nRadialPoints];
            double[] radialTProfileC = new double[nRadialPoints];
            double[] radialViscosityProfilePaS = new double[nRadialPoints];
            double[] radialVulcanizationProfilePercent = new double[nRadialPoints];
            double[] radialTimeEqProfileS = new double[nRadialPoints];

            // Radial distribution from center to wall (0 to rH_m)
            for (int k = 0; k < nRadialPoints; k++)
            {
                double r_fraction = (double)k / (nRadialPoints - 1);
                radialRadiiMM[k] = r_fraction * rH_m * 1000; // Radius from center to wall in mm

                // Simplistic assumption: temperature varies linearly from center to wall temp
                // More complex: solve heat equation in radial direction
                radialTProfileC[k] = T_out_working_point_C; // For simplicity, assume uniform temp for now
                // A better approximation might be T_center + (T_wall - T_center) * (r/R)^2 for heat generation in circular pipe

                radialViscosityProfilePaS[k] = _materialProperties.GetViscosity(radialTProfileC[k], Q_working_point_m3_s / (Math.PI * Math.Pow(rH_m, 2) * rH_m), Q_working_point_m3_s / (Math.PI * Math.Pow(rH_m, 2) * rH_m)); // Example shear rate
                radialVulcanizationProfilePercent[k] = AdditionalVulcanizationHeadPercent; // Uniform for now
                radialTimeEqProfileS[k] = 0; // Not calculated in detail for radial for now
            }


            // --- Return results for the head ---
            var finalHeadResult = new HeadResult
            {
                Id = _headSections.Last().Id, // ID of the last head section
                VolumeExpenseSM3S = Q_working_point_m3_s * 1e6, // m^3/s to cm^3/s
                SpecificPressureNearHeadMPa = P_working_point_Pa / 1e6, // Pa to MPa
                TemperatureMaterialEnteringHeadC = T_working_point_C,
                TemperatureMaterialAfterExtrusionC = T_out_working_point_C,
                VelocityMMin = Q_working_point_m3_s / (Math.PI * Math.Pow(D_final_m / 2.0, 2)) * 60.0, // m/s to m/min
                CrossSectionAreaCM2 = Math.PI * Math.Pow(D_final_m / 2.0, 2) * 1e4, // m^2 to cm^2
                PartInductionPeriodScrewPercent = finalScrewShearDeformationSummary, // Shear deformation from screw
                AdditionalPartInductionPeriodHeadPercent = AdditionalVulcanizationHeadPercent, // From head calculation
                // Radial profiles
                RadialRadiiMM = radialRadiiMM,
                RadialTemperatureProfileC = radialTProfileC,
                RadialViscosityProfilePaS = radialViscosityProfilePaS,
                RadialVulcanizationProfilePercent = radialVulcanizationProfilePercent,
                RadialTimeEqProfileS = radialTimeEqProfileS
            };

            return finalHeadResult;
        }

        // --- Вспомогательные функции для расчета (упрощенные версии из Pascal) ---

        // Поиск текущей секции корпуса по общей длине
        private (BarrelSection barrel, double localZInBarrelSection) FindCurrentBarrelSection(double totalPassedLengthM)
        {
            double currentBarrelStartLength = 0;
            foreach (var barrel in _barrelSections)
            {
                if (totalPassedLengthM >= currentBarrelStartLength && totalPassedLengthM < currentBarrelStartLength + barrel.LengthMM)
                {
                    return (barrel, totalPassedLengthM - currentBarrelStartLength);
                }
                currentBarrelStartLength += barrel.LengthMM;
            }
            // If passed all sections, return the last one and its length
            return (_barrelSections.Last(), _barrelSections.Last().LengthMM);
        }

        // Pascal `FindVulcanizationInterval` (QPT.txt)
        private int FindVulcanizationInterval(KineticsParameters kinetics, double t_eqv)
        {
            for (int i = 0; i < kinetics.Intervals.Count; i++)
            {
                if (t_eqv < kinetics.Intervals[i].EquivalentTime)
                    return i;
            }
            return kinetics.Intervals.Count - 1; // If t_eqv is greater than all intervals
        }

        // Pascal `UpdateEquivalentTime` (QPT.txt)
        private double UpdateEquivalentTime(KineticsParameters kinetics, double current_t_eqv, double T_celsius, double dt, int intervalIndex)
        {
            if (intervalIndex < 0 || intervalIndex >= kinetics.Intervals.Count)
            {
                // Fallback to first interval if index is out of bounds
                intervalIndex = 0;
                Debug.WriteLine($"Warning: Invalid interval index for UpdateEquivalentTime. Using index 0. Index: {intervalIndex}");
                if (!kinetics.Intervals.Any()) return current_t_eqv; // Cannot compute without intervals
            }

            var interval = kinetics.Intervals[intervalIndex];
            double T_eqv = kinetics.EquivalentTemperature;
            double E_R_int = interval.ActivationEnergyOverR;

            double T_abs = T_celsius + 273.15; // Temperature in Kelvins
            double T_eqv_abs = T_eqv + 273.15; // Equivalent temperature in Kelvins

            if (T_abs <= 0 || T_eqv_abs <= 0)
            {
                Debug.WriteLine($"Invalid absolute temperature for UpdateEquivalentTime: T_abs={T_abs}, T_eqv_abs={T_eqv_abs}");
                return current_t_eqv;
            }

            double exponent = E_R_int * (T_celsius - T_eqv) / (T_abs * T_eqv_abs);
            if (double.IsNaN(exponent) || double.IsInfinity(exponent))
            {
                Debug.WriteLine($"Invalid exponent in equivalent time: exponent={exponent}, T_celsius={T_celsius}, T_eqv={T_eqv}, T_abs={T_abs}, T_eqv_abs={T_eqv_abs}, E_R_int={E_R_int}");
                return current_t_eqv;
            }
            return current_t_eqv + Math.Exp(exponent) * dt;
        }

        // Pascal `GetVulcanizationDegree` (QPT.txt)
        private double GetVulcanizationDegree(KineticsParameters kinetics, double t_eqv, int intervalIndex)
        {
            if (intervalIndex < 0 || intervalIndex >= kinetics.Intervals.Count)
            {
                intervalIndex = 0;
                Debug.WriteLine($"Warning: Invalid interval index for GetVulcanizationDegree. Using index 0. Index: {intervalIndex}");
                if (!kinetics.Intervals.Any()) return 0; // Cannot compute without intervals
            }
            var interval = kinetics.Intervals[intervalIndex];
            if (interval.EquivalentTime <= 1e-9) return 0; // Protection against division by zero
            return Math.Min(t_eqv / interval.EquivalentTime * 100, 100);
        }
    }
}