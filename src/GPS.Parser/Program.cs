using GHIElectronics.TinyCLR.Devices.Gpio;
using GHIElectronics.TinyCLR.Devices.Spi;
using GHIElectronics.TinyCLR.Devices.Uart;
using GHIElectronics.TinyCLR.Drivers.Sitronix.ST7735;
using GHIElectronics.TinyCLR.Pins;
using GPS.Parser.Properties;
using System;
using System.Collections;
using System.Diagnostics;
using System.Drawing;
using System.Text;
using System.Threading;

namespace GPS.Parser
{
    class Program
    {
        private static ST7735Controller st7735;
        private const int SCREEN_WIDTH = 160;
        private const int SCREEN_HEIGHT = 128;
        static Timer timer;
        static Random rnd;
        static double lat, lon;
        static GpsPoint LastPosition = null;
        private static void Main()
        {
            var spi = SpiController.FromName(SC20100.SpiBus.Spi3);
            var gpio = GpioController.GetDefault();

            st7735 = new ST7735Controller(
                spi.GetDevice(ST7735Controller.GetConnectionSettings
                (SpiChipSelectType.Gpio, gpio.OpenPin(SC20100.GpioPin.PD10))), //CS pin.
                gpio.OpenPin(SC20100.GpioPin.PC4), //RS pin.
                gpio.OpenPin(SC20100.GpioPin.PE15) //RESET pin.
            );

            var backlight = gpio.OpenPin(SC20100.GpioPin.PE5);
            backlight.SetDriveMode(GpioPinDriveMode.Output);
            backlight.Write(GpioPinValue.High);

            st7735.SetDataAccessControl(true, true, false, false); //Rotate the screen.
            st7735.SetDrawWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
            st7735.Enable();

            // Create flush event
            Graphics.OnFlushEvent += Graphics_OnFlushEvent;

            // Create bitmap buffer
            var screen = Graphics.FromImage(new Bitmap(SCREEN_WIDTH, SCREEN_HEIGHT));

            //var image = Resources.GetBitmap(Resources.BitmapResources.
            //    smallJpegBackground);

            var font = Resources.GetFont(Resources.FontResources.small);

            rnd = new Random();

            timer = new Timer((object o) =>
            {
                screen.Clear();

                screen.FillEllipse(new SolidBrush(System.Drawing.Color.FromArgb
                    (255, 255, 0, 0)), 0, 0, 80, 64);

                screen.FillEllipse(new SolidBrush(System.Drawing.Color.FromArgb
                    (255, 0, 0, 255)), 80, 0, 80, 64);

                screen.FillEllipse(new SolidBrush(System.Drawing.Color.FromArgb
                    (128, 0, 255, 0)), 40, 0, 80, 64);
                /*
                //screen.DrawImage(image, 56, 50);

                screen.DrawRectangle(new Pen(Color.Yellow), 10, 80, 40, 25);
                screen.DrawEllipse(new Pen(Color.Purple), 60, 80, 40, 25);
                screen.FillRectangle(new SolidBrush(Color.Teal), 110, 80, 40, 25);

                screen.DrawLine(new Pen(Color.White), 10, 127, 150, 127);
                screen.SetPixel(80, 92, Color.White);
                */
                if (LastPosition != null)
                {
                    screen.DrawString("[GPS PARSER]", font, new SolidBrush(Color.Green), 10, 80);
                    screen.DrawString($"lat = {string.Format("{0:n4}", LastPosition.Latitude)}, lon = {string.Format("{0:n4}", LastPosition.Longitude)}", font, new SolidBrush(Color.Green), 10, 100);
                }
                screen.Flush();
            }, null, 1000, 1000);

            StartGPS();
            Thread.Sleep(Timeout.Infinite);
        }

        private static void Graphics_OnFlushEvent(Graphics sender, byte[] data, int x, int y, int width, int height, int originalWidth)
        {
            st7735.DrawBuffer(data);
        }

        public static void StartGPS()
        {
            //using NEO 6M GPS MODULE
            UartController serialPort = UartController.FromName(SC20100.UartPort.Uart1);
            var uartSetting = new UartSetting()
            {
                BaudRate = 9600,
                DataBits = 8,
                Parity = UartParity.None,
                StopBits = UartStopBitCount.One,
                Handshaking = UartHandshake.None,
            };
            serialPort.SetActiveSettings(uartSetting);
            serialPort.Enable();

            Reader gpsShield = new Reader(serialPort, 100, 0.0);
            gpsShield.GpsData += GpsShield_GpsData;
            gpsShield.Start();
        }

        static void WriteLine(string Message, bool Status = false)
        {
            Debug.WriteLine(Message);
        }

        private static void GpsShield_GpsData(GpsPoint gpsPoint)
        {
            Debug.WriteLine("time: " + gpsPoint.Timestamp + "Lat/Lng: " + gpsPoint.Latitude + "/" + gpsPoint.Longitude);
            //Lcd.Clear();
            WriteLine("  GPS MONITOR ", false);
            WriteLine("              ", false);
            WriteLine("Time: " + gpsPoint.Timestamp + " UTC     ");
            //string latStr = gpsPoint.Latitude.ToString("F4");
            //string lonStr = gpsPoint.Longitude.ToString("F4");           
            LastPosition = gpsPoint;
            WriteLine("Lat: " + lat + "     ");
            WriteLine("Lng: " + lon + "     ");
        }
    }

}
