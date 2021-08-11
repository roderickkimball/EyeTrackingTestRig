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
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace EyeRobotControlApp
{
    /// <summary>
    /// Interaction logic for NeckSteppersManual.xaml
    /// </summary>
    public partial class NeckControl : Page
    {
        private SerialComm serialComm;

        public NeckControl(SerialComm serialComm)
        {
            this.serialComm = serialComm;
            InitializeComponent();
            //neckPosition.Text = this.serialComm.Get_NeckPosition();
        }

        private void NeckCalibrationButton_Click(object sender, RoutedEventArgs e)
        {
            NeckCalibration neckCal = new NeckCalibration(serialComm);
            serialComm.ChangeState(SerialComm.StateMachine.NeckCalibrate);
            neckCal.ShowDialog();
        }

        public void DisplayPosition()
        {
            neckPosition.Text = serialComm.Get_NeckPosition();
        }

        private void SendAngles_Click(object sender, RoutedEventArgs e)
        {
            bool tryYaw = float.TryParse(yawInput.GetLineText(0), out float yaw);
            bool tryPhiR = float.TryParse(phiRInput.GetLineText(0), out float amplitude);
            bool tryPhiS = float.TryParse(phiSInput.GetLineText(0), out float angle);

            // TODO: add another check that values are within valid range
            if (tryYaw & tryPhiR & tryPhiS)
            {
                serialComm.Send_NeckToPosition(amplitude, angle, yaw);

                neckPosition.Text = serialComm.Get_NeckPosition();
            }
            else MessageBox.Show("Those inputs weren't regocnizable!\nYou need to give one angle per box.");
        }
    }
}