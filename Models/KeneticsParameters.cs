using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using ExtrusionAnalysis.ViewModel;

namespace ExtrusionAnalysis.Models
{
    public class KineticsParameters : ObservableObject
    {
        private int _numberOfIntervals;
        public double _equivalentTemperature;
        public double _inductionTime;
        public double _optimalTime;
        public double _initialTemperature;
        private ObservableCollection<CureKineticsInterval> _intervals;

        public int NumberOfIntervals
        {
            get => _numberOfIntervals;
            set => SetProperty(ref _numberOfIntervals, value);
        }

        public ObservableCollection<CureKineticsInterval> Intervals
        {
            get => _intervals;
            set => SetProperty(ref _intervals, value);
        }

        public double EquivalentTemperature
        {
            get => _equivalentTemperature;
            set => SetProperty(ref _equivalentTemperature, value);
        }

        public double InductionTime
        {
            get => _inductionTime;
            set => SetProperty(ref _inductionTime, value);
        }
        public double OptimalTime
        {
            get => _optimalTime;
            set => SetProperty(ref _optimalTime, value);
        }
        public double InitialTemperature
        {
            get => _initialTemperature;
            set => SetProperty(ref _initialTemperature, value);
        }

        public KineticsParameters()
        {
            Intervals = new ObservableCollection<CureKineticsInterval>();
        }

        public void AddInterval(double teqv, double er)
        {
            Intervals.Add(new CureKineticsInterval
            {
                Order = Intervals.Count + 1,
                EquivalentTime = teqv,
                ActivationEnergyOverR = er
            });
            NumberOfIntervals = Intervals.Count;
        }

        public class CureKineticsInterval : ObservableObject
        {
            private int _order;
            private double _equivalentTime;
            private double _activationEnergyOverR;

            public int Order
            {
                get => _order;
                set => SetProperty(ref _order, value);
            }
            public double EquivalentTime
            {
                get => _equivalentTime;
                set => SetProperty(ref _equivalentTime, value);
            }
            public double ActivationEnergyOverR
            {
                get => _activationEnergyOverR;
                set => SetProperty(ref _activationEnergyOverR, value);
            }
        }
    }
}