using UnityEngine;
using System.Collections;
//using ManagedROSWrapper;
using System.Runtime;
using System.Runtime.InteropServices;
using UnityEngine.UI;
  

public class diffdrive : MonoBehaviour
{
    static string path = @"C:\Users\Gregory Brill\Source\Repos\ROSIndigo\ROSIndigoDLL\Debug\";

    #region interop definitions



    [DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool SetDllDirectory(string lpPathName);

    [DllImport("DiffDrive.dll", EntryPoint = "ROSInit", CallingConvention = CallingConvention.Cdecl)]
    public static extern void ROSInit(string nodeName);

    [DllImport("DiffDrive.dll", EntryPoint = "InitStandAlone", CallingConvention = CallingConvention.Cdecl)]
    public static extern void InitStandAlone(int szPoseBuff, int callRosInit);

    [DllImport("DiffDrive.dll", EntryPoint = "DirectCommand", CallingConvention = CallingConvention.Cdecl)]
    public static extern void DirectCommand(double speed, double theta);


    [DllImport("DiffDrive.dll", EntryPoint = "SpinStandAloneOnce", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SpinStandAloneOnce();

    [DllImport("DiffDrive.dll", EntryPoint = "SpinStandAloneOnce", CallingConvention = CallingConvention.Cdecl)]
    public static extern void getTimeStamp(ref System.UInt32 sec, ref System.UInt32 nsec);

    [DllImport("DiffDrive.dll", EntryPoint = "GetPosition", CallingConvention = CallingConvention.Cdecl)]
    public static extern void GetPosition(ref double x, ref double y, ref double z,
        ref double qx, ref double qy, ref double qz, ref double qw, ref double quangle);

    [DllImport("DiffDrive.dll", EntryPoint = "GetPosition2D", CallingConvention = CallingConvention.Cdecl)]
    public static extern void GetPosition2D(ref double x, ref double y, ref double quangle);

    [DllImport("DiffDrive.dll", EntryPoint = "SetPosition2D", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetPosition2D(double x, double y,  double quangle);



    #endregion

    public Slider slideSpeed;
    public Slider slideTheta;

    //ManagedROSWrapper.Class1 c=new ManagedROSWrapper.Class1();
    public void GetPosition()
    {
        double x, theta;
        double posX = 0, posY = 0, posZ = 0;
        double qX = 0, qY = 0, qZ = 0, qW=0, qAngle = 0;

        x = System.Convert.ToDouble(slideSpeed.value);
        theta = System.Convert.ToDouble(slideTheta.value);

        //diff.DirectCommand(x, theta);
        //diff.SpinStandAloneOnce();
        //diff.GetPosition(ref posX, ref posY, ref posZ, ref qX, ref qY,ref qZ,ref qAngle);
        DirectCommand(x, theta);
        SpinStandAloneOnce();
        GetPosition(ref posX, ref posY, ref posZ, ref qX, ref qY, ref qZ, ref qW, ref qAngle);

      Debug.Log(string.Format("\t---->x:{0} y:{1} z:{2} angle:{3}", posX, posY, posZ, qAngle));


    }

    public void InitDiffDrive()
    {
        Debug.Log("Initializing Diff Drive DLL");

        SetDllDirectory(path);
        InitStandAlone(10, 1);
    //    SetPosition2D(transform.position.x, transform.position.z, transform.rotation.y);
        
    }
     
    // Use this for initialization
    void Start()
    {

    } 

    // Update is called once per frame
    void Update()
    {

    }
} 
