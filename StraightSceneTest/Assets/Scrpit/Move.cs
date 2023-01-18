using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Move : MonoBehaviour
{
    private float m_speed = 5f;
    private float log_up;
    private float log_initx = 0f;
    private float log_perx = 1f;
    private float pow_up;
    private float pow_initx = 0f;
    private float pow_perx = 1f;
    public Text SpeedTip;

    void Update()
    {
        RoleMove();
        SpeedTip.text = m_speed.ToString();
    }

    public void RoleMove()
    {
        if (Input.GetKey(KeyCode.Alpha1))
        {

            m_speed = m_speed * 1.001f;
        }
        else if (Input.GetKey(KeyCode.Alpha2))
        {
            pow_initx += pow_perx;
            pow_up = Mathf.Pow(pow_initx,0.2f);
            m_speed = 5 + pow_up;
        }
        else if (Input.GetKey(KeyCode.Alpha3))
        {
            log_initx += log_perx;
            log_up = Mathf.Log(log_initx);
            m_speed =5+ log_up;

        }
        else
        {
            m_speed = 5f;
            log_initx = 0f;
            pow_initx = 0f;
        }

        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
        {
            this.transform.Translate(Vector3.forward * m_speed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.RightArrow))
        {
            this.transform.Translate(Vector3.right * m_speed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
        {
            this.transform.Translate(Vector3.back * m_speed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.LeftArrow))
        {
            this.transform.Translate(Vector3.left * m_speed * Time.deltaTime);
        }

    }
}
