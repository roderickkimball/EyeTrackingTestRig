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
    /// Interaction logic for EyeServoManual.xaml
    /// </summary>
    public partial class EyeControl : Page
    {
        public string DisplGazePos
        {
            get { return (string)GetValue(DisplGazePosProperty); }
            set { SetValue(DisplGazePosProperty, value); }
        }
        public static readonly DependencyProperty DisplGazePosProperty =
            DependencyProperty.Register("DisplGazePos", typeof(string), typeof(EyeControl), new PropertyMetadata(string.Empty));

        private readonly SerialComm serialComm;
        private float lastX;
        private float lastZ;

        public EyeControl(SerialComm comm)
        {
            InitializeComponent();
            DataContext = this;

            serialComm = comm;

            eyeServoCalButton.Content = " Eye Servo\nCalibration";
            gButton.Content += "\n";

            lastX = (float)0.0;
            lastZ = (float)0.0;
        }

        private void GazeButton_Click(object sender, RoutedEventArgs e)
        {
            bool tryX = float.TryParse(gazeToTextBoxX.GetLineText(0), out float posX);
            bool tryZ = float.TryParse(gazeToTextBoxZ.GetLineText(0), out float posZ);
            if (tryX & tryZ)
            {
                string newpos = ((float)posX).ToString() + "," + ((float)posZ).ToString();
                serialComm.Send_GazePoint(newpos);
                lastX = posX; lastZ = posZ;
                DisplGazePos = serialComm.Get_GazePosition();
            }
            else
            {
                MessageBox.Show("Cannot recognize X or Z inputs!");
            }

        }

        private void PosButton_Click(object sender, RoutedEventArgs e)
        {
            DisplGazePos = serialComm.Get_GazePosition();
        }

        private void CenterButton_Click(object sender, RoutedEventArgs e)
        {
            serialComm.Send_GazePoint("0,0");
            DisplGazePos = serialComm.Get_GazePosition();
        }

        private void ModeButton_Click(object sender, RoutedEventArgs e)
        {
            //DisplMode = serialComm.Get_Mode();
        }

        private void TypeButton_Click(object sender, RoutedEventArgs e)
        {
            string phrase = letterTextBox.Text.ToString().ToUpperInvariant();
            MessageBox.Show(phrase);

            foreach (char toType in phrase)
            { // note: won't display position during loop without Messagebox, even if delayed
                serialComm.Type_Char(toType);
                this.DisplGazePos = serialComm.Get_GazePosition();
                //time for tobii to recognize the chosen key
                MessageBox.Show(toType.ToString());
            }
        }

        private void DebugButton_Click(object sender, RoutedEventArgs e)
        {
            serialComm.Send(debugText.Text.ToString());
        }

        private void EyeServoCal_Click(object sender, RoutedEventArgs e)
        {
            EyeCalibration eyeCal = new EyeCalibration(serialComm);
            serialComm.ChangeState(SerialComm.StateMachine.EyeCalibrate);
            eyeCal.ShowDialog();
        }
    }


}
