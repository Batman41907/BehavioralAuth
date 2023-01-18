using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraView : MonoBehaviour
{
    public float m_sensitivityX = 10f;
    public float m_sensitivityY = 10f;

    // 垂直方向镜头范围
    public float m_minimumY = -45f;
    public float m_maximumY = 45f;

    float m_rotationY = 0f;


    // Use this for initialization
    void Start()
    {
        // 防止刚体影响镜头旋转
        if (GetComponent<Rigidbody>())
        {
            GetComponent<Rigidbody>().freezeRotation = true;
        }
    }

    void Update()
    {
        RotateAngle();
    }

    public void RotateAngle()
    {
        float m_rotationX = transform.localEulerAngles.y + Input.GetAxis("Mouse X") * m_sensitivityX;//水平偏移角度
        m_rotationY += Input.GetAxis("Mouse Y") * m_sensitivityY;//垂直偏移角度
        m_rotationY = Mathf.Clamp(m_rotationY, m_minimumY, m_maximumY);//返回在既定范围内的值
        transform.localEulerAngles = new Vector3(-m_rotationY, m_rotationX, 0);//绕x轴的是垂直偏移量，绕y轴的是水平偏移量
    }
}
