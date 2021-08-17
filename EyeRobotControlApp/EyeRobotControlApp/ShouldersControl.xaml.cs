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
    /// Interaction logic for HomeShoulders.xaml
    /// </summary>
    public partial class ShouldersControl : Page
    {
        private readonly SerialComm serialComm;

        public ShouldersControl(SerialComm comm)
        {
            InitializeComponent();
            serialComm = comm;

            //displShoulderPos.Text = serialComm.Get_ShoulderPosition();
            doneText.Text = "DONE";

            ManualCommandsVisibility(Visibility.Visible);
            shoulderStepperManualButton.Visibility = Visibility.Hidden;
            waitText.Visibility = Visibility.Hidden;
            wait_bar.Visibility = Visibility.Hidden;
            doneText.Visibility = Visibility.Hidden;
        }

        public void DisplayPosition()
        {
            displShoulderPos.Text = serialComm.Get_ShoulderPosition();
        }

        private void WaitForSteppers()
        {
            wait_bar.Visibility = Visibility.Visible;
            waitText.Visibility = Visibility.Visible;
            ManualCommandsVisibility(Visibility.Hidden);

            wait_bar.Value = 0;

            Task.Run(() =>
            {
                for (int i = 0; i <= 100; i++)
                {
                    System.Threading.Thread.Sleep(200);
                    //System.Threading.Thread.Sleep(10);

                    Dispatcher.Invoke(() =>
                    {
                        if (serialComm.In_MenuMode()) i = 100;
                        wait_bar.Value = i;
                        if (i == 100)
                        {
                            doneText.Visibility = Visibility.Visible;
                            shoulderStepperManualButton.Visibility = Visibility.Visible;
                        }
                    });
                }
            });
        }

        private void ManualCommandsVisibility(Visibility visibility)
        {
            homeShoulderSteppersButton.Visibility = visibility;
            moveShoulersTo_Layout.Visibility = visibility;
            moveShoulderButton.Visibility = visibility;
            displShoulderPos.Visibility = visibility;
            metersPSA.Visibility = visibility;
            FOR_PSA.Visibility = visibility;
        }

        private void ShoulderStepperManualButton_Click(object sender, RoutedEventArgs e)
        {
            waitText.Visibility = Visibility.Hidden;
            wait_bar.Visibility = Visibility.Hidden;
            doneText.Visibility = Visibility.Hidden;
            shoulderStepperManualButton.Visibility = Visibility.Hidden;
            ManualCommandsVisibility(Visibility.Visible);

            serialComm.ChangeState(SerialComm.StateMachine.RobotRun); 
        }

        private void MoveShoulderButton_Click(object sender, RoutedEventArgs e)
        {
            bool tryBack = float.TryParse(disBackInput.GetLineText(0), out float back);
            bool tryRight = float.TryParse(disRightInput.GetLineText(0), out float right);
            bool tryUp = float.TryParse(disUpInput.GetLineText(0), out float up);
            
            if (tryBack & tryRight & tryUp)
            {
                if ((back < 0) & (right < 0) & (up < 0))
                    MessageBox.Show("Shoulder positions cannot be negative!");
                else serialComm.Send_ShoulderToPosition(right, back, up);
            }
            else MessageBox.Show("Error in reading positions!");
            
            displShoulderPos.Text = serialComm.Get_ShoulderPosition();
        }

        private void HomeSteppers_Click(object sender, RoutedEventArgs e)
        {
            serialComm.ChangeState(SerialComm.StateMachine.ShoulderCalibrate);
            this.WaitForSteppers();
        }
    }
}