using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace ExtrusionAnalysis.ViewModel
{
    public class ObservableObject : INotifyPropertyChanged
    {
        // Событие, оповещающее об изменении свойств объекта
        public event PropertyChangedEventHandler PropertyChanged;

        // Метод для вызова события PropertyChanged при изменении свойства
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            // Проверяем, если есть подписчики на событие
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        // Метод для установки значения свойства объекта
        protected bool SetProperty<T>(ref T storage, T value, [CallerMemberName] string propertyName = null)
        {
            // Проверяем, если новое значение равно старому
            if (Equals(storage, value))
                return false; // Возвращаем false, чтобы не вызывать событие PropertyChanged

            // Сохраняем новое значение в хранилище
            storage = value;

            // Вызываем событие PropertyChanged, оповещая об изменении свойства
            OnPropertyChanged(propertyName);

            // Возвращаем true, чтобы указать, что свойство изменилось
            return true;
        }
    }

}
