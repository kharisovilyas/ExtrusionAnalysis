using ExtrusionAnalysis.ViewModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ExtrusionAnalysis.Models
{
    public class FluxData : ObservableObject
    {
        private int _n; // Pascal: N (число сопряженных материальных слоев в потоке)
        private int _iR; // Pascal: iR (число регулярных шагов по r/R для построения v(r))
        private double _aH; // Pascal: AH (коэффициент неравномерности шага по "alpha")
        private double _aB; // Pascal: AB (коэффициент неравномерности шага по "alpha")
        private double _kH; // Pascal: kH (коэффициент неравномерности шага по r/R)
        private double _kB; // Pascal: kB (коэффициент неравномерности шага по r/R)

        public int N // N_Alpha в скриншотах
        {
            get => _n;
            set => SetProperty(ref _n, value);
        }

        public int IR // N_r/R в скриншотах
        {
            get => _iR;
            set => SetProperty(ref _iR, value);
        }

        public double AH // k1_Alph в скриншотах
        {
            get => _aH;
            set => SetProperty(ref _aH, value);
        }

        public double AB // k2_Alph в скриншотах
        {
            get => _aB;
            set => SetProperty(ref _aB, value);
        }

        public double KH // k1_r/R в скриншотах
        {
            get => _kH;
            set => SetProperty(ref _kH, value);
        }

        public double KB // k2_r/R в скриншотах
        {
            get => _kB;
            set => SetProperty(ref _kB, value);
        }
    }
}