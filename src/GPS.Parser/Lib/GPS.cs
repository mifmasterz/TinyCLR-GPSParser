﻿using GHIElectronics.TinyCLR.Devices.Uart;
using System;
using System.Collections;
using System.Diagnostics;
using System.Text;
using System.Threading;

namespace GPS.Parser
{

    public class GprmcParser
    {
        // Parse the GPRMC line
        //
        public static GpsPoint Parse(string line)
        {
            // $GPRMC,040302.663,A,3939.7,N,10506.6,W,0.27,358.86,200804,,*1A
            //Debug.WriteLine("GpsPoint Parse");                                                                                                                                                                                                                           
            if (!IsCheckSumGood(line))
            {
                return null;
            }
            //Debug.WriteLine(line);
            try
            {
                string[] parts = line.Split(',');
                //Debug.WriteLine(parts.Length.ToString());
                if (parts.Length != 13)  // This GPS has extra field
                {
                    return null;
                }
                //Debug.WriteLine(parts[2]);
                if (parts[2] != "A")
                {
                    return null;
                }
                //Debug.WriteLine(parts[9]);
                string date = parts[9]; // UTC Date DDMMYY
                if (date.Length != 6)
                {
                    return null;
                }
                int year = 2000 + int.Parse(date.Substring(4, 2));
                int month = int.Parse(date.Substring(2, 2));
                int day = int.Parse(date.Substring(0, 2));
                string time = parts[1]; // HHMMSS.XXX
                if (time.Length != 9)
                {
                    return null;
                }
                int hour = int.Parse(time.Substring(0, 2));
                int minute = int.Parse(time.Substring(2, 2));
                int second = int.Parse(time.Substring(4, 2));
                int milliseconds = int.Parse(time.Substring(7, 2));
                DateTime utcTime = new DateTime(year, month, day, hour, minute, second, milliseconds);

                string lat = parts[3];  // HHMM.MMMM
                if (lat.Length != 10)
                {
                    return null;
                }
                double latHours = double.Parse(lat.Substring(0, 2));
                double latMins = double.Parse(lat.Substring(2));
                double latitude = latHours + latMins / 60.0;
                if (parts[4] == "S")       // N or S
                {
                    latitude = -latitude;
                }

                string lng = parts[5];  // HHHMM.MMMMM
                if (lng.Length != 11)
                {
                    return null;
                }
                double lngHours = double.Parse(lng.Substring(0, 3));
                double lngMins = double.Parse(lng.Substring(3));
                double longitude = lngHours + lngMins / 60.0;
                if (parts[6] == "W")
                {
                    longitude = -longitude;
                }

                double speed = double.Parse(parts[7]);
                double bearing = double.Parse(parts[8]);

                // Should probably validate check sum

                GpsPoint gpsPoint = new GpsPoint
                {
                    BearingInDegrees = bearing,
                    Latitude = latitude,
                    Longitude = longitude,
                    SpeedInKnots = speed,
                    Timestamp = utcTime
                };
                return gpsPoint;

            }
            catch (Exception)
            {
                // One of our parses failed...ignore.
                Debug.WriteLine("parse exception");
            }
            return null;
        }

        private static bool IsCheckSumGood(string sentence)
        {
            int index1 = sentence.IndexOf("$");
            int index2 = sentence.LastIndexOf("*");

            if (index1 != 0 || index2 != sentence.Length - 3)
            {
                return false;
            }

            string checkSumString = sentence.Substring(index2 + 1, 2);
            int checkSum1 = Convert.ToInt32(checkSumString, 16);

            string valToCheck = sentence.Substring(index1 + 1, index2 - 1);
            char c = valToCheck[0];
            for (int i = 1; i < valToCheck.Length; i++)
            {
                c ^= valToCheck[i];
            }

            return checkSum1 == c;
        }
    }

    public static class GeoDistanceCalculator
    {
        private const double _earthRadiusInMiles = 3956.0;
        private const double _earthRadiusInKilometers = 6367.0;
        public static double DistanceInMiles(double lat1, double lng1, double lat2, double lng2)
        {
            return Distance(lat1, lng1, lat2, lng2, _earthRadiusInMiles);
        }
        public static double DistanceInKilometers(double lat1, double lng1, double lat2, double lng2)
        {
            return Distance(lat1, lng1, lat2, lng2, _earthRadiusInKilometers);
        }
        private static double Distance(double lat1, double lng1, double lat2, double lng2, double radius)
        {
            // Implements the Haversine formulat http://en.wikipedia.org/wiki/Haversine_formula
            //
            var lat = NumericExtensions.ToRadians(lat2 - lat1);
            var lng = NumericExtensions.ToRadians(lng2 - lng1);
            var sinLat = System.Math.Sin(0.5 * lat);
            var sinLng = System.Math.Sin(0.5 * lng);
            var cosLat1 = System.Math.Cos(NumericExtensions.ToRadians(lat1));
            var cosLat2 = System.Math.Cos(NumericExtensions.ToRadians(lat2));
            var h1 = sinLat * sinLat + cosLat1 * cosLat2 * sinLng * sinLng;
            var h2 = System.Math.Sqrt(h1);
            var h3 = 2 * System.Math.Asin(System.Math.Min(1, h2));
            return radius * h3;
        }
    }

    public static class NumericExtensions
    {
        public static double ToRadians(this double val)
        {
            return (System.Math.PI / 180) * val;
        }
    }

    public class Reader
    {
        private readonly object _lock = new object();
        private readonly UartController _serialPort;
        private readonly int _timeOut;
        private readonly double _minDistanceBetweenPoints;
        private bool _isStarted;
        private Thread _processor;

        public delegate void LineProcessor(string line);

        public delegate void GpsDataProcessor(GpsPoint gpsPoint);

        public event LineProcessor RawLine;
        public event GpsDataProcessor GpsData;

        public bool IsStarted { get { return _isStarted; } }

        public Reader(UartController serialPort)
            : this(serialPort, 100, 0.0)
        {

        }
        public Reader(UartController serialPort, int timeOutBetweenReadsInMilliseconds, double minDistanceInMilesBetweenPoints)
        {
            _serialPort = serialPort;
            _timeOut = timeOutBetweenReadsInMilliseconds;
            _minDistanceBetweenPoints = minDistanceInMilesBetweenPoints;
        }

        public bool Start()
        {
            lock (_lock)
            {
                if (_isStarted)
                {
                    return false;
                }
                _isStarted = true;
                _processor = new Thread(ThreadProc);
                _processor.Start();
            }
            return true;
        }

        public bool Stop()
        {
            lock (_lock)
            {
                if (!_isStarted)
                {
                    return false;
                }
                _isStarted = false;
                if (!_processor.Join(5000))
                {
                    _processor.Abort();
                }
                return true;
            }
        }

        private void ThreadProc()
        {
            Debug.WriteLine("GPS thread started...");
            //if (!_serialPort.IsOpen)
            //{
            //    _serialPort.Open();
            //}
            while (_isStarted)
            {
                int bytesToRead = _serialPort.BytesToRead;
                if (bytesToRead > 0)
                {
                    byte[] buffer = new byte[bytesToRead];
                    var byteRec = _serialPort.Read(buffer, 0, buffer.Length);
                    try
                    {
                        string temp = Encoding.UTF8.GetString(buffer, 0, byteRec);
                        Debug.WriteLine(temp);
                        ProcessBytes(temp);
                    }
                    catch (Exception ex)
                    {
                        // only process lines we can parse.
                        Debug.WriteLine(ex.ToString());
                    }
                }

                Thread.Sleep(_timeOut);
            }
            Debug.WriteLine("GPS thread stopped...");
        }

        private string _data = string.Empty;
        private GpsPoint _lastPoint;
        private DateTime _lastDateTime = DateTime.Now;

        private void ProcessBytes(string temp)
        {
            while (temp.IndexOf('\n') != -1)
            {
                string[] parts = temp.Split('\n');
                _data += parts[0];
                _data = _data.Trim();
                if (_data != string.Empty)
                {
                    if (_data.IndexOf("$GPRMC") == 0)
                    {
                        Debug.WriteLine("GOT $GPRMC LINE");
                        if (GpsData != null)
                        {
                            GpsPoint gpsPoint = GprmcParser.Parse(_data);
                            if (gpsPoint != null)
                            {
                                bool isOk = true;
                                if (_lastPoint != null)
                                {
                                    double distance = GeoDistanceCalculator.DistanceInMiles(gpsPoint.Latitude, gpsPoint.Longitude,
                                                                                   _lastPoint.Latitude, _lastPoint.Longitude);
                                    double distInFeet = distance * 5280;
                                    Debug.WriteLine("distance = " + distance + " mi (" + distInFeet + " feet)");
                                    if (distance < _minDistanceBetweenPoints)
                                    {
                                        // Too close to the last point....don't raise the event
                                        isOk = false;
                                    }
                                    DateTime now = DateTime.Now;
                                    TimeSpan diffseconds = (now - _lastDateTime);
                                    if (diffseconds.Seconds > 60)
                                    {
                                        // A minute has gone by, so update
                                        isOk = true;
                                        _lastDateTime = now;
                                    }
                                }
                                _lastPoint = gpsPoint;

                                // Raise the event
                                if (isOk)
                                {
                                    GpsData(gpsPoint);
                                }
                            }
                        }
                    }
                    if (RawLine != null)
                    {
                        RawLine(_data);
                    }
                }
                temp = parts[1];
                _data = string.Empty;
            }
            _data += temp;
        }

    }
    public class GpsPoint
    {
        public DateTime Timestamp { get; set; }
        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double SpeedInKnots { get; set; }
        public double BearingInDegrees { get; set; }
    }
}
