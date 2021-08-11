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
using System.Runtime.InteropServices;

namespace EyeRobotControlApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public string DisplMode
        {
            get { return (string)GetValue(DisplModeProperty); }
            set { SetValue(DisplModeProperty, value); }
        }
        public static readonly DependencyProperty DisplModeProperty =
            DependencyProperty.Register("DisplMode", typeof(string), typeof(MainWindow), new PropertyMetadata(string.Empty));

        private readonly SerialComm serialComm;
        private readonly InputCOM inputCOM;

        private readonly EyeControl eyeControl;
        private readonly NeckControl neckControl;
        private readonly ShouldersControl shouldersControl;

        public MainWindow()
        {
            InitializeComponent();
            //DataContext = this;
            DisplayPage.Background = new ImageBrush(SetImage(Properties.Resources.WSU_Gleason));
            
            bool eject;
            bool try_again;
            do
            {
                inputCOM = new InputCOM();
                inputCOM.ShowDialog();

                string comName = inputCOM.Get_COM();
                eject = inputCOM.Get_Eject();
                try_again = !eject;
                try
                {
                    serialComm = new SerialComm(comName, 115200);
                    //serialComm = new SerialComm(comName, 9600); // test Ardunio

                    serialComm.ChangeState(SerialComm.StateMachine.MenuMode);
                    
                    if (serialComm.IsOpen()) //was using serialComm.In_MenuMode() as handshake
                    {
                        MessageBox.Show(serialComm.GetPortName() + " is open!");
                        try_again = false;
                    }
                    else
                    {
                        //throw new Exception("valid port, but not the robot");
                        MessageBox.Show("Valid COM port,\nbut cannot connect to robot");
                        serialComm.Close();
                    }
                }
                catch
                {
                    MessageBox.Show("Incorrect COM port");
                }
            }
            while (try_again);
            if (eject) { this.Close(); }

            eyeControl = new EyeControl(serialComm);
            neckControl = new NeckControl(serialComm);
            shouldersControl = new ShouldersControl(serialComm);


            serialComm.ChangeState(SerialComm.StateMachine.RobotRun);
        }

        private void EyeModeButton_Click(object sender, RoutedEventArgs e)
        {
            //DisplMode = serialComm.Get_Mode();
            DisplayPage.Content = eyeControl;
            ShowClick(eyeModeButton);
        }

        private void ShoulderModeButton_Click(object sender, RoutedEventArgs e)
        {
            //serialComm.ChangeState(SerialComm.StateMachine.RobotRun);
            DisplayPage.Content = shouldersControl;
            shouldersControl.DisplayPosition();
            ShowClick(shoudlerModeButton);
        }

        private void NeckSteppersManualButton_Click(object sender, RoutedEventArgs e)
        {
            //serialComm.ChangeState(SerialComm.StateMachine.RobotRun);
            DisplayPage.Content = neckControl;
            neckControl.DisplayPosition();
            ShowClick(neckModeButton);
        }

        private void ShowClick(Button button_clicked)
        {
            eyeModeButton.IsEnabled = true;
            neckModeButton.IsEnabled = true;
            shoudlerModeButton.IsEnabled = true;

            button_clicked.IsEnabled = false;
        }

        private ImageSource SetImage(System.Drawing.Bitmap picture)
        {
            BitmapSource bSource = BitmapToBitmapSource(picture);
            return bSource;
        }

        public static BitmapSource BitmapToBitmapSource(System.Drawing.Bitmap bitmap)
        {
            var bitmapData = bitmap.LockBits(
                new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height),
                System.Drawing.Imaging.ImageLockMode.ReadOnly, bitmap.PixelFormat);

            var bitmapSource = BitmapSource.Create(
                bitmapData.Width, bitmapData.Height,
                bitmap.HorizontalResolution, bitmap.VerticalResolution,
                PixelFormats.Bgr24, null,
                bitmapData.Scan0, bitmapData.Stride * bitmapData.Height, bitmapData.Stride);

            bitmap.UnlockBits(bitmapData);

            return bitmapSource;
        }

        private void Test_Click(object sender, RoutedEventArgs e)
        {
            serialComm.In_MenuMode();
        }
    }
}