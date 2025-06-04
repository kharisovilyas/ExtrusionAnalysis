using ExtrusionAnalysis.ViewModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ExtrusionAnalysis.Models
{
    public class ExtrusionResult : ObservableObject
    {
        // Pascal QPT.exe Output Columns
        private string _id; // Секция ID для фильтрации (например, S1, S2, B1 и т.д.)
        private int _sectionOrder; // Pascal Z: Order of the section
        private double _positionMM; // Pascal Z: Position (mm)
        private double _pressureMPa; // Pascal p: Specific pressure (MPa)
        private double _temperatureC; // Pascal T: Temperature of polymer material (°C)
        private double _barrelTemperatureC; // Pascal T_in: Temperature of barrel (°C) - assuming this is T_korp
        private double _screwTemperatureC; // Pascal T_out: Temperature of screw (°C) - assuming this is TB_k
        private double _channelHeightMM; // Pascal H: Geometry of screw channel (mm)
        private double _channelWidthMM; // Pascal W: Geometry of screw channel (mm)
        private double _shearStressKPa; // Pascal Tau: Shear stress (kPa)
        private double _energyThermalMJm3; // Pascal Ener_T: Density of energy (thermal) (MJ/m^3)
        private double _energyMechanicalMJm3; // Pascal Ener_M: Density of energy (mechanical) (MJ/m^3)
        private double _flowRateDeltaSM3S; // Pascal Q_del: Expenditure by radial slots (sm^3/sec)
        private double _shearDeformationAdded; // Pascal Shear deformation: added one
        private double _shearDeformationSummary; // Pascal Shear deformation: summary one
        public double _shearRate;
        public double ShearRate // Z, мм
        {
            get => _shearRate;
            set => SetProperty(ref _shearRate, value);
        }
        public double _viscosity;
        public double Viscosity // Z, мм
        {
            get => _viscosity;
            set => SetProperty(ref _viscosity, value);
        }
        public double _shearStressZ;
        public double ShearStressZ // Z, мм
        {
            get => _shearStressZ;
            set => SetProperty(ref _shearStressZ, value);
        }
        public double _shearStressX;
        public double ShearStressX // Z, мм
        {
            get => _shearStressX;
            set => SetProperty(ref _shearStressX, value);
        }
        public double _residenceTime;
        public double ResidenceTime // Z, мм
        {
            get => _residenceTime;
            set => SetProperty(ref _residenceTime, value);
        }
        public double _vulcanizationDegree;
        public double VulcanizationDegree // Z, мм
        {
            get => _vulcanizationDegree;
            set => SetProperty(ref _vulcanizationDegree, value);
        }

        public string Id
        {
            get => _id;
            set => SetProperty(ref _id, value);
        }

        public int SectionOrder // Z, номер секции
        {
            get => _sectionOrder;
            set => SetProperty(ref _sectionOrder, value);
        }

        public double PositionMM // Z, мм
        {
            get => _positionMM;
            set => SetProperty(ref _positionMM, value);
        }

        public double PressureMPa // p, МПа
        {
            get => _pressureMPa;
            set => SetProperty(ref _pressureMPa, value);
        }

        public double TemperatureC // T, °C
        {
            get => _temperatureC;
            set => SetProperty(ref _temperatureC, value);
        }

        public double BarrelTemperatureC // T_in (T_korp)
        {
            get => _barrelTemperatureC;
            set => SetProperty(ref _barrelTemperatureC, value);
        }

        public double ScrewTemperatureC // T_out (TB_k)
        {
            get => _screwTemperatureC;
            set => SetProperty(ref _screwTemperatureC, value);
        }

        public double ChannelHeightMM // H, мм
        {
            get => _channelHeightMM;
            set => SetProperty(ref _channelHeightMM, value);
        }

        public double ChannelWidthMM // W, мм
        {
            get => _channelWidthMM;
            set => SetProperty(ref _channelWidthMM, value);
        }

        public double ShearStressKPa // Tau, кПа
        {
            get => _shearStressKPa;
            set => SetProperty(ref _shearStressKPa, value);
        }

        public double EnergyThermalMJm3 // Ener_T, МДж/м^3
        {
            get => _energyThermalMJm3;
            set => SetProperty(ref _energyThermalMJm3, value);
        }

        public double EnergyMechanicalMJm3 // Ener_M, МДж/м^3
        {
            get => _energyMechanicalMJm3;
            set => SetProperty(ref _energyMechanicalMJm3, value);
        }

        public double FlowRateDeltaSM3S // Q_del, см^3/сек
        {
            get => _flowRateDeltaSM3S;
            set => SetProperty(ref _flowRateDeltaSM3S, value);
        }
        public double ShearDeformationAdded // added one
        {
            get => _shearDeformationAdded;
            set => SetProperty(ref _shearDeformationAdded, value);
        }
        public double ShearDeformationSummary // summary one
        {
            get => _shearDeformationSummary;
            set => SetProperty(ref _shearDeformationSummary, value);
        }
    }
}
