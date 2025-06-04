using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace ExtrusionAnalysis.ViewModel
{
    public class RelayCommand : ICommand
    {
        // Делегат для метода, который будет выполнен при вызове команды
        private readonly Action _execute;
        // Делегат для метода, определяющего, может ли команда быть выполнена
        private readonly Func<bool> _canExecute;

        // Конструктор класса RelayCommand
        public RelayCommand(Action execute, Func<bool> canExecute = null)
        {
            // Проверяем, что метод для выполнения команды не равен null, иначе выбрасываем исключение ArgumentNullException
            _execute = execute ?? throw new ArgumentNullException(nameof(execute));
            // Присваиваем делегату _canExecute метод, определяющий, может ли команда быть выполнена
            _canExecute = canExecute;
        }

        // Событие, сигнализирующее об изменении возможности выполнения команды
        public event EventHandler CanExecuteChanged
        {
            // Подписываемся на событие RequerySuggested, чтобы оповещать о изменении состояния команды
            add { CommandManager.RequerySuggested += value; }
            // Отписываемся от события RequerySuggested
            remove { CommandManager.RequerySuggested -= value; }
        }

        // Метод, определяющий, может ли команда быть выполнена
        public bool CanExecute(object parameter) => _canExecute?.Invoke() ?? true;

        // Метод, выполняющий команду
        public void Execute(object parameter)
        {
            // Проверяем, что метод для выполнения команды не равен null, и вызываем его
            if (_execute != null)
            {
                _execute();
            }
        }
    }
}
