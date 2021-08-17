using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace EyeRobotControlApp
{
    /// <summary>
    /// Interaction logic for InputCOM.xaml
    /// </summary>
    public partial class InputCOM : Window
    {
        string comName;
        bool ejectCondition;

        public InputCOM()
        {
            InitializeComponent();
            ejectCondition = false;
        }

        public string Get_COM() { return comName; }
        public bool Get_Eject() { return ejectCondition; }

        private void Enter_Click(object sender, RoutedEventArgs e)
        {
            comName = comPortBox.GetLineText(0);
            this.Close();
        }

        private void EjectApp_Click(object sender, RoutedEventArgs e)
        {
            ejectCondition = true;
            this.Close();
        }
    }
}
