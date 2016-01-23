class Program
{
    //把第二个参数设置为2.5表示更高的灵敏度，但坏处是增大了抖动
    static AHRS.MadgwickAHRS AHRS = new AHRS.MadgwickAHRS(1f / 256f, 2.5f);
    //根据具体情况，将串口设置为正确的编号
    private static SerialPort port = new SerialPort("COM7", 9600, Parity.None, 8, StopBits.One);

    private static Form_3Dcuboid form_3DcuboidA = new Form_3Dcuboid();

    private static void port_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        // Show all the incoming data in the port's buffer
        string line = port.ReadLine();
        string[] fields = line.Split(',');
        if (fields.Length == 9)
        {
            AHRS.Update(deg2rad(float.Parse(fields[0])), deg2rad(float.Parse(fields[1])), deg2rad(float.Parse(fields[2])),
                deg2rad(float.Parse(fields[3])), deg2rad(float.Parse(fields[4])), deg2rad(float.Parse(fields[5])));
        }
        form_3DcuboidA.RotationMatrix = ConvertToRotationMatrix(AHRS.Quaternion);
    }
    
    static void Main(string[] args)
    {
        Console.WriteLine(Assembly.GetExecutingAssembly().GetName().Name + " " + Assembly.GetExecutingAssembly().GetName().Version.Major.ToString() + "." + Assembly.GetExecutingAssembly().GetName().Version.Minor.ToString());
        try
        {
            // Show 3D cuboid forms
            Console.WriteLine("Showing 3D Cuboid forms...");
            form_3DcuboidA.Text += "IMU orientation estimation demo";
            BackgroundWorker backgroundWorkerA = new BackgroundWorker();
            backgroundWorkerA.DoWork += new DoWorkEventHandler(delegate { form_3DcuboidA.ShowDialog(); });
            backgroundWorkerA.RunWorkerAsync();

            port.DataReceived += new SerialDataReceivedEventHandler(port_DataReceived);
            port.Open();
        }
        catch (Exception ex)
        {
            Console.WriteLine("Error: " + ex.Message);
        }
        Application.Run();
    }

    static void xIMUserial_CalInertialAndMagneticDataReceived_updateIMU(object sender, x_IMU_API.CalInertialAndMagneticData e)
    {
        AHRS.Update(deg2rad(e.Gyroscope[0]), deg2rad(e.Gyroscope[1]), deg2rad(e.Gyroscope[2]), e.Accelerometer[0], e.Accelerometer[1], e.Accelerometer[2]);
    }

    static void xIMUserial_CalInertialAndMagneticDataReceived_updateAHRS(object sender, x_IMU_API.CalInertialAndMagneticData e)
    {
        AHRS.Update(deg2rad(e.Gyroscope[0]), deg2rad(e.Gyroscope[1]), deg2rad(e.Gyroscope[2]), e.Accelerometer[0], e.Accelerometer[1], e.Accelerometer[2], e.Magnetometer[0], e.Magnetometer[1], e.Magnetometer[2]);
    }

    private static float deg2rad(float degrees)
    {
        return (float)(Math.PI / 180) * degrees;
    }

    private static float[] ConvertToRotationMatrix(float[] Quaternion)
    {
        float R11 = 2 * Quaternion[0] * Quaternion[0] - 1 + 2 * Quaternion[1] * Quaternion[1];
        float R12 = 2 * (Quaternion[1] * Quaternion[2] + Quaternion[0] * Quaternion[3]);
        float R13 = 2 * (Quaternion[1] * Quaternion[3] - Quaternion[0] * Quaternion[2]);
        float R21 = 2 * (Quaternion[1] * Quaternion[2] - Quaternion[0] * Quaternion[3]);
        float R22 = 2 * Quaternion[0] * Quaternion[0] - 1 + 2 * Quaternion[2] * Quaternion[2];
        float R23 = 2 * (Quaternion[2] * Quaternion[3] + Quaternion[0] * Quaternion[1]);
        float R31 = 2 * (Quaternion[1] * Quaternion[3] + Quaternion[0] * Quaternion[2]);
        float R32 = 2 * (Quaternion[2] * Quaternion[3] - Quaternion[0] * Quaternion[1]);
        float R33 = 2 * Quaternion[0] * Quaternion[0] - 1 + 2 * Quaternion[3] * Quaternion[3];
        return new float[] { R11, R12, R13,
                             R21, R22, R23,
                             R31, R32, R33 };
    }

    private static float[] ConvertToEulerAngles(float[] Quaternion)
    {
        float phi = (float)Math.Atan2(2 * (Quaternion[2] * Quaternion[3] - Quaternion[0] * Quaternion[1]), 2 * Quaternion[0] * Quaternion[0] - 1 + 2 * Quaternion[3] * Quaternion[3]);
        float theta = (float)-Math.Atan((2.0 * (Quaternion[1] * Quaternion[3] + Quaternion[0] * Quaternion[2])) / Math.Sqrt(1.0 - Math.Pow((2.0 * Quaternion[1] * Quaternion[3] + 2.0 * Quaternion[0] * Quaternion[2]), 2.0)));
        float psi = (float)Math.Atan2(2 * (Quaternion[1] * Quaternion[2] - Quaternion[0] * Quaternion[3]), 2 * Quaternion[0] * Quaternion[0] - 1 + 2 * Quaternion[1] * Quaternion[1]);
        return new float[] { Rad2Deg(phi), Rad2Deg(theta), Rad2Deg(psi) };
    }

    private static float Rad2Deg(float radians)
    {
        return 57.2957795130823f * radians;
    }
}
