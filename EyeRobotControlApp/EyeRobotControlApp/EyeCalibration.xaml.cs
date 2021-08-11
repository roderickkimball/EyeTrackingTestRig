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
using System.IO.Ports;

namespace EyeRobotControlApp
{
    /// <summary>
    /// Interaction logic for EyeCal.xaml
    /// </summary>
    public partial class EyeCalibration : Window
    {
        private SerialComm serialComm;

        public EyeCalibration(SerialComm comm)
        {
            InitializeComponent();
            serialComm = comm;
            up_button.IsEnabled = false;
            down_button.IsEnabled = false;
            right_button.IsEnabled = false;
            left_button.IsEnabled = false;
        }

        private void B_Click(object sender, RoutedEventArgs e)
        {
            serialComm.ChangeState(SerialComm.StateMachine.RobotRun);
            this.Close();
        }
        private void Up_Click(object sender, RoutedEventArgs e)
        {
            serialComm.Send_CalUp();
        }
        private void Down_Click(object sender, RoutedEventArgs e)
        {
            serialComm.Send_CalDown();
        }
        private void Left_Click(object sender, RoutedEventArgs e)
        {
            serialComm.Send_CalDown();
        }
        private void Right_Click(object sender, RoutedEventArgs e)
        {
            serialComm.Send_CalUp();
        }

        private void LeftVerticalServo_Click(object sender, RoutedEventArgs e)
        {
            this.up_button.IsEnabled = true;
            this.down_button.IsEnabled = true;
            this.left_button.IsEnabled = false;
            this.right_button.IsEnabled = false;

            serialComm.Send("2"); /*go to LV servo*/
            PressMotorButton(left_vertical_servo);
        }

        private void RightVerticalServo_Click(object sender, RoutedEventArgs e)
        {
            this.up_button.IsEnabled = true;
            this.down_button.IsEnabled = true;
            this.left_button.IsEnabled = false;
            this.right_button.IsEnabled = false;

            serialComm.Send("1"); /*go to RV servo*/
            PressMotorButton(right_vertical_servo);
        }

        private void LeftHorizontalServo_Click(object sender, RoutedEventArgs e)
        {
            this.up_button.IsEnabled = false;
            this.down_button.IsEnabled = false;
            this.left_button.IsEnabled = true;
            this.right_button.IsEnabled = true;

            serialComm.Send("4"); /*go to LH servo*/
            PressMotorButton(left_horizontal_servo);
        }

        private void RightHorizontalServo_Click(object sender, RoutedEventArgs e)
        {
            this.up_button.IsEnabled = false;
            this.down_button.IsEnabled = false;
            this.left_button.IsEnabled = true;
            this.right_button.IsEnabled = true;

            serialComm.Send("3"); /*go to RH servo*/
            PressMotorButton(right_horizontal_servo);
        }

        private void PressMotorButton(Button motor)
        {
            left_horizontal_servo.IsEnabled = true;
            left_vertical_servo.IsEnabled = true;
            right_vertical_servo.IsEnabled = true;
            right_horizontal_servo.IsEnabled = true;

            motor.IsEnabled = false;
        }
    }
}