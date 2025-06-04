using System.Windows;
using ExtrusionAnalysis.Models;
using ExtrusionAnalysis.ViewModels;

namespace ExtrusionAnalysis.Views
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DataContext = new MainViewModel();
        }
    }
}