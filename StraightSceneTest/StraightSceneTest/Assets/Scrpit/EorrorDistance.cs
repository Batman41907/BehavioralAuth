using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EorrorDistance : MonoBehaviour
{
    public static bool IsEnterTrigger = false;
   // public List<GameObject> IndicateRoad = new List<GameObject>();

    private void OnTriggerExit(Collider collision)
    {
        if (collision.tag == "PERSON")
        {

            IsEnterTrigger = false;
            //for (int i = 0; i < IndicateRoad.Count; i++)
            //{
            //    IndicateRoad[i].GetComponent<Renderer>().material.color = Color.red;
            //}
            // Debug.Log("人物超出范围");
        }


    }
    private void OnTriggerEnter(Collider collision)
    {
        if (collision.tag == "PERSON")
        {
            IsEnterTrigger = false;
            //for (int i = 0; i < IndicateRoad.Count; i++)
            //{
            //    IndicateRoad[i].GetComponent<Renderer>().material.color = Color.red;
            //}
            // Debug.Log("人物进入范围");
        }


    }
    private void OnTriggerStay(Collider collision)
    {
        if (collision.tag == "PERSON")
        {
            IsEnterTrigger = true;

            //for (int i = 0; i < IndicateRoad.Count; i++)
            //{
            //    IndicateRoad[i].GetComponent<Renderer>().material.color = Color.blue;
            //}
            //Debug.Log("人物呆在范围");
        }

    }

}
