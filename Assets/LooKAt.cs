using UnityEngine;
using System.Collections;

public class LooKAt : MonoBehaviour {
    public Transform target;
	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
        transform.rotation = Quaternion.LookRotation(transform.position - target.position);
	}
}
