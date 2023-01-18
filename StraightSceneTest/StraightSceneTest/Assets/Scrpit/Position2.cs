using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Position2 : MonoBehaviour
{
    public Vector3 position2;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        position2 = this.transform.localPosition;
    }
}
