using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RecordTime : MonoBehaviour
{
    protected long startTime;
    protected long endTime;
    private int hour;
    private int minute;
    private int second;
    private int year;
    private int month;
    private int day;

    // Start is called before the first frame update
    void Start()
    {
         
        Debug.Log("当前时间："+ CurrenctTime());
        startTime = System.DateTime.Now.Ticks;
        StartCoroutine(RecordDataPoint(0.04f));

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //TaskTime1();
    }


    protected virtual long TaskTime1() //完成任务的时间
    {
        endTime = System.DateTime.Now.Ticks;
        long currentMillis = (endTime - startTime) / 10000;
        //Debug.Log(currentMillis);
        return currentMillis;
    }

    IEnumerator RecordDataPoint(float steptime)
    {
        while (true)
        {
            Debug.Log("当前时间：" + CurrenctTime()+"任务完成时间"+TaskTime1());
            yield return new WaitForSeconds(steptime);
        }
        //开始任务后，每0.04s进行一次采样。当用户完成一次任务，再把所有信息写入。
    }

    protected string CurrenctTime()
    {
        hour = DateTime.Now.Hour;
        minute = DateTime.Now.Minute;
        second = DateTime.Now.Second;
        var systemTime= string.Format("{0:D2}:{1:D2}:{2:D2} ", hour, minute, second);
        return systemTime;
    }
}
