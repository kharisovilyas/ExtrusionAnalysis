using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using ExtrusionAnalysis.ViewModel;
using ExtrusionAnalysis.ViewModels;

namespace ExtrusionAnalysis.Models
{
    public class MaterialProperties : ObservableObject
    {
        private double _flowIndex; // Pascal: n (индекс течения)
        private double _characteristicTemperature; // Pascal: T0 (характерная температура переработки, °C)
        private double _consistencyCoefficient; // Pascal: Mu0 (коэффициент консистенции при T=T0, кПа с^n)
        private double _temperatureViscosityInfluence; // Pascal: b (степень влияния температуры на вязкость, 1/K)
        private double _density; // Pascal: Ro (плотность, кг/м^3)
        private double _thermalDiffusivity; // Pascal: a (коэффициент температуропроводности, м^2/с, ×10^8)
        private double _heatConductivity; // Pascal: Lam (коэффициент теплопроводности, Вт/(м К))
        private double _equivalentTemperature; // Pascal: T_eqv (температура эквивалентного режима превращения, °C)
        private double _inductionTime; // Pascal: t_in_eq (индукционное время превращения при T=Teqv, с)
        private double _optimalTime; // Pascal: t_op_eq (время критического превращения при T= Teqv, с)
        private double _initialTemperature; // Pascal: T_ini (температура начала необратимого превращения, °C)

        public double FlowIndex // n
        {
            get => _flowIndex;
            set => SetProperty(ref _flowIndex, value);
        }

        public double CharacteristicTemperature // T0
        {
            get => _characteristicTemperature;
            set => SetProperty(ref _characteristicTemperature, value);
        }

        public double ConsistencyCoefficient // Mu0 (в кПа*с^n, в расчетах будет в Па*с^n)
        {
            get => _consistencyCoefficient;
            set => SetProperty(ref _consistencyCoefficient, value);
        }

        public double TemperatureViscosityInfluence // b
        {
            get => _temperatureViscosityInfluence;
            set => SetProperty(ref _temperatureViscosityInfluence, value);
        }

        public double Density // Ro
        {
            get => _density;
            set => SetProperty(ref _density, value);
        }

        public double ThermalDiffusivity // a (в м^2/с * 10^8, в расчетах будет в м^2/с)
        {
            get => _thermalDiffusivity;
            set => SetProperty(ref _thermalDiffusivity, value);
        }

        public double HeatConductivity // Lam
        {
            get => _heatConductivity;
            set => SetProperty(ref _heatConductivity, value);
        }

        public double EquivalentTemperature // T_eqv
        {
            get => _equivalentTemperature;
            set => SetProperty(ref _equivalentTemperature, value);
        }

        public double InductionTime // t_in_eq
        {
            get => _inductionTime;
            set => SetProperty(ref _inductionTime, value);
        }

        public double OptimalTime // t_op_eq
        {
            get => _optimalTime;
            set => SetProperty(ref _optimalTime, value);
        }

        public double InitialTemperature // T_ini
        {
            get => _initialTemperature;
            set => SetProperty(ref _initialTemperature, value);
        }

        // Метод GetViscosity из Pascal Mu*exp((m-1)*Ln(Mu_eff[i]))/MuEf_Fix
        // Mu - это ConsistencyCoefficient, m - это FlowIndex
        // Mu_eff[i] - это sqrt(sqr(Gamma_XY[i])+sqr(Gamma_YZ[i])) - полная скорость сдвига
        // MuEf_Fix - это Mu*exp((m-1)*Ln(u/H)) - опорная вязкость
        // Для точного соответствия нам нужно будет передавать полную скорость сдвига и опорную скорость сдвига
        public double GetViscosity(double T_celsius, double shearRate, double referenceShearRate)
        {
            // Переводим T_celsius в T0 в формуле
            double T_eff = T_celsius - CharacteristicTemperature;

            // ConsistencyCoefficient (Mu0) приходит в кПа*с^n, переводим в Па*с^n
            double mu0_Pa = ConsistencyCoefficient * 1000.0;

            // shearRate должна быть абсолютной скоростью сдвига
            // referenceShearRate - это та величина, на которую нормируется shearRate
            // Math.Pow(shearRate / referenceShearRate, FlowIndex - 1) - это часть для неньютоновской жидкости
            // Если shearRate очень маленькая, может быть проблема с Log.
            if (shearRate < 1e-9) shearRate = 1e-9;
            if (referenceShearRate < 1e-9) referenceShearRate = 1e-9;

            return mu0_Pa * Math.Exp(-TemperatureViscosityInfluence * T_eff) * Math.Pow(shearRate / referenceShearRate, FlowIndex - 1);
        }
    }
}