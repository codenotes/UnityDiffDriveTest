using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using Messages;


#pragma warning disable 0219
#pragma warning disable 0168



public class RobotControl : MonoBehaviour {

	// Use this for initialization
public float rotationSpeed = 30.0f;
public float speed = 10.0f;
public float RayLength = 10.0f;
public float ScanAngle = 270f;
public int ScanPoints = 1080;
public float Step = .25f;
public float ScanSpeed = .025f;

//public Slider slider;
//public Slider LeftForceSlider;
//public Slider RightForceSlider;

public Slider slideSpeed;
public Slider slideTheta;

public UnityEngine.UI.Text txVel;



public float force 
{ get;
    
 set; 


}


 public Vector3 RotateX(Vector3 v, float angle)
 {
     float sin = Mathf.Sin(angle);
     float cos = Mathf.Cos(angle);

     float ty = v.y;
     float tz = v.z;
     v.y = (cos * ty) - (sin * tz);
     v.z = (cos * tz) + (sin * ty);
     return v;

 }

 public Vector3 RotateY(Vector3 v, float angle)
 {
     float sin = Mathf.Sin(angle);
     float cos = Mathf.Cos(angle);

     float tx = v.x;
     float tz = v.z;
     v.x = (cos * tx) + (sin * tz);
     v.z = (cos * tz) - (sin * tx);
     return v;

 }

 public  Vector3 RotateZ( Vector3 v, float angle)
 {
     float sin = Mathf.Sin(angle);
     float cos = Mathf.Cos(angle);

     float tx = v.x;
     float ty = v.y;
     v.x = (cos * tx) - (sin * ty);
     v.y = (cos * ty) + (sin * tx);
     return v;

 }


 void Awake()
 {
     bScanStart = false;
     

   


  //   motor = GetComponent<CharacterMotor>();
 }

void FixedUpdate()
 {

     //float yaw, x, y, z, w = 0;

     //x = transform.rotation.x;
     //y = transform.rotation.y;
     //z = transform.rotation.z;
     //w = transform.rotation.w;

     //yaw = Mathf.Asin(2f * x * y + 2f * z * w);

     //Debug.Log(string.Format("{0} {1} {2} {3}", transform.rotation.Yaw(), transform.eulerAngles.x, transform.eulerAngles.y, transform.eulerAngles.z));



//  //  txVel.text = rigidbody.angularVelocity.ToString(); //this works, taking it out for performance reasons.
//     var lefty = GameObject.Find("lefty");
//     var left_forward = lefty.transform.TransformDirection(Vector3.forward);


////     lefty.rigidbody.AddForce(left_forward * LeftForceSlider.value , ForceMode.Impulse);

//     var righty = GameObject.Find("righty");
//     var right_forward = righty.transform.TransformDirection(Vector3.forward);


//     Debug.DrawRay(lefty.transform.position, left_forward * LeftForceSlider.value, Color.yellow);
//     Debug.DrawRay(righty.transform.position, right_forward * RightForceSlider.value, Color.red);


//     var resv_add = (left_forward * LeftForceSlider.value) + (right_forward * RightForceSlider.value);
//  //   var resv_mult = (left_forward * LeftForceSlider.value). * (right_forward * RightForceSlider.value);
     
    
//     Debug.DrawRay(transform.position, resv_add * 10, Color.green);
//    // Debug.DrawRay(transform.position, resv_mult * 10, Color.blue);

//    // rigidbody.AddForce(resv, ForceMode.Force);

//  //   righty.rigidbody.AddForce(right_forward * RightForceSlider.value, ForceMode.Impulse);
     
    

 }
 
 void Update()
 {
     Vector3 directionVector = new Vector3(0, Input.GetAxis("Vertical"));
 
     transform.Rotate(Vector3.up, Input.GetAxis("Horizontal") * rotationSpeed * Time.deltaTime);
 

       //if (Input.GetKey(KeyCode.T))
       //{
       //    RightForceSlider.value--;
       //}

       //if (Input.GetKey(KeyCode.Y))
       //{
       //    RightForceSlider.value++;
       //}


       //if (Input.GetKey(KeyCode.G))
       //{
       //    LeftForceSlider.value--;

       //}

       //if (Input.GetKey(KeyCode.H))
       //{
       //    LeftForceSlider.value++; 

       //}




     if (Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S)){ 
         transform.Translate(0, 0, -1* speed * Time.deltaTime);
     }
 
     if (Input.GetKey (KeyCode.UpArrow) || Input.GetKey(KeyCode.W)) {
         transform.Translate (0, 0, 1 * speed * Time.deltaTime);
     }
 
     if (Input.GetKey(KeyCode.RightArrow)|| Input.GetKey(KeyCode.D)) {
         transform.Rotate(0, 1, 0);
     }
 
     if (Input.GetKey (KeyCode.LeftArrow)|| Input.GetKey(KeyCode.A)) {
         transform.Rotate(0, -1, 0);
     }


     if (Input.GetKey(KeyCode.X)) //scan it
     {
         
      


         var forward = transform.TransformDirection(Vector3.forward);

         Vector3 rbound = RotateY(forward, ScanAngle * Mathf.Deg2Rad/2f);
         Vector3 lbound = RotateY(forward, -ScanAngle * Mathf.Deg2Rad/2f);
         
         
        
         
         Debug.DrawRay(transform.position, forward * RayLength, Color.green);
         
         
         Debug.DrawRay(transform.position, rbound * RayLength, Color.red);
         Debug.DrawRay(transform.position, lbound * RayLength, Color.red);

         if (bScanStart) return;

         bScanStart = true;

         //StartCoroutine("Scan");
         Scan2();


     }


#if false     
     //not sure what below is for
     //if (directionVector != Vector3.zero) {
     //    // Get the length of the directon vector and then normalize it
     //    // Dividing by the length is cheaper than normalizing when we already have the length anyway
     //    float directionLength = directionVector.magnitude;
     //    directionVector = directionVector / directionLength;
         
     //    // Make sure the length is no bigger than 1
     //    directionLength = Mathf.Min(1, directionLength);
         
     //    // Make the input vector more sensitive towards the extremes and less sensitive in the middle
     //    // This makes it easier to control slowz speeds when using analog sticks
     //    directionLength = directionLength * directionLength;
         
     //    // Multiply the normalized direction vector by the modified length
     //    directionVector = directionVector * directionLength;
     //}
     
     // Apply the direction to the CharacterMotor
  //   motor.inputMoveDirection = transform.rotation * directionVector;
//     motor.inputJump = Input.GetButton("Jump");
#endif

 }


 IEnumerator Scan()
 {
     var forward = transform.TransformDirection(Vector3.forward);
     Vector3 lbound = RotateY(forward, -ScanAngle * Mathf.Deg2Rad/2f);
     
     var start = -ScanAngle/2f ;
     //var start = -45f;
     

     for (int i = 0; i < ScanPoints;i++ )
     {

         

         Vector3 rcast = RotateY(forward, (start )* Mathf.Deg2Rad  );

         Debug.DrawRay(transform.position, rcast* RayLength, Color.yellow);

         start += Step;
       //  Debug.Log(start);



         //Color c = renderer.material.color;
         //c.a = f;
         //renderer.material.color = c;
         yield return null; // new WaitForSeconds(ScanSpeed); //null;
     }
     bScanStart = false;

 }


void PubScan(Messages.sensor_msgs.LaserScan lscan)
 {

 }



 void Scan2()
 {
     var forward = transform.TransformDirection(Vector3.forward);
     Vector3 lbound = RotateY(forward, -ScanAngle * Mathf.Deg2Rad / 2f);

     var start = -ScanAngle / 2f;
     var scan=new Messages.sensor_msgs.LaserScan();

     

     //var start = -45f;


     for (int i = 0; i < ScanPoints; i++)
     {



         Vector3 rcast = RotateY(forward, (start) * Mathf.Deg2Rad);

         Debug.DrawRay(transform.position, rcast * RayLength, Color.yellow);

         start += Step;
     //    Debug.Log(start);



         //Color c = renderer.material.color;
         //c.a = f;
         //renderer.material.color = c;
        
     }


         //ros::Time scan_time = ros::Time::now();
     var tm=new Messages.std_msgs.Time();


     System.UInt32 sec = 0;
     System.UInt32 nsec = 0;


     diffdrive.getTimeStamp(ref sec, ref nsec);
     
     scan.header.stamp.data.nsec = nsec;
     scan.header.stamp.data.sec = sec;


    //populate the LaserScan message
    
   // scan.header.stamp = scan_time;
     scan.header.frame_id = new Messages.std_msgs.String("laser_frame");
    scan.angle_min = -ScanAngle * Mathf.Deg2Rad;
    scan.angle_max = ScanAngle * Mathf.Deg2Rad;
    scan.angle_increment = 3.14f / (float)ScanPoints;  //was num_readings
  //  scan.time_increment = (1 / laser_frequency) / (num_readings); //time in seconds between measurements
    scan.time_increment = ScanSpeed / (float)ScanPoints;
    scan.range_min = 0.0f;
    scan.range_max = 30.0f;

    PubScan(scan);


     bScanStart = false;

 }


 public bool bScanStart { get; set; }

 
public void slideForce()
 {

    // force = slider.value;


 }


public void TestButtonClick()
{
    //Debug.Log("Apply force");
    //Debug.Log(force);

    var forward = transform.TransformDirection(Vector3.forward);

   // rigidbody.AddForce(forward * force, ForceMode.VelocityChange);
    var lefty=GameObject.Find("lefty");
    var left_forward = lefty.transform.TransformDirection(Vector3.forward);
    lefty.rigidbody.AddForce(left_forward * force, ForceMode.VelocityChange);

    var righty= GameObject.Find("righty");
    var right_forward = righty.transform.TransformDirection(Vector3.forward);
    righty.rigidbody.AddForce(right_forward * force, ForceMode.VelocityChange);
    

    


    //experimental
 //   var v = new Vector3( 0, -force, 0f);
   // rigidbody.AddTorque(v, ForceMode.VelocityChange);

    

}



}
















/*

Hokuyu data
 * Angular resolution

Step angle : approx. 0.36°(360°/1,024 steps)
Scanning time
270deg fov
25ms/scan
1080 steps at .25 degrees per step
 * 
 * 
 * 
 * * 
 * 
 * 
 * 
 
 * 
 * 
 * 

 UTM30LXEW
 
 
 
 
 Header header
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds]
 * 
 * 
 * if 100ms and 
float32 scan_time        # time between scans [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units]
 * 
 * 
 
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 */