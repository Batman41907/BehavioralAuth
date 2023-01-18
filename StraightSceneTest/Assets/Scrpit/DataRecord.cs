using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using OfficeOpenXml;

public class DataRecord : MonoBehaviour
{
    // Start is called before the first frame update
    protected void SomeTest()
    {
        string FilePath = "D:/Test.xlsx";

        //获取excel的信息
        FileInfo fileInfo = new FileInfo(FilePath);
        //通过excel文件信息打开excel表格
        using (ExcelPackage excelPackage = new ExcelPackage(fileInfo))
        {
            ExcelWorksheet worksheet;
            if (!fileInfo.Exists)
            {
                //取得Excel文件中的第一张表
                worksheet = excelPackage.Workbook.Worksheets.Add("Sheet1");
                excelPackage.Save();
            }
            else
            {
                worksheet = excelPackage.Workbook.Worksheets[1];
            }

            //取得某个单元格的值

            // string s= worksheet.Cells[1, 1].Value.ToString();
            // Debug.Log(s);
            //写入数据
            worksheet.Cells[1, 1].Value = "麦";
            //保存数据
            excelPackage.Save();
            //做一些操作
        }//关闭Excel

    }
    public void Button()
    {
        SomeTest();
    }
}
