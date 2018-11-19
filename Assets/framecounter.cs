using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
public class framecounter : MonoBehaviour {
    /// <summary>
    /// 화면에 보여줄 text object
    /// </summary>
    public Text text;
    /// <summary>
    /// 200프레임동안의 FPS의 합
    /// </summary>
    private float fpsSum = 0.0f;
    /// <summary>
    /// 0~200 프레임의 인덱스[0~200)
    /// </summary>
    private int frameNum = 0;
    /// <summary>
    /// 200프레임 집합의 개수[0~10)
    /// </summary>
    private int frameStep = 0;
    /// <summary>
    /// 현재의 프레임 (0~) 
    /// </summary>
    private int globalFrame;
    //
    private float globalSum;
    public int startframe; 
    /// <summary>
    /// 모든 deltaTime의 합
    /// </summary>
    private float BigSum = 0;
    /// <summary>
    /// frame 계산을 화면에 보여주는 플래그
    /// </summary>
    public bool showFrame = true;

    void Start () {
        InitText();
        globalFrame = 0;
        globalSum = 0.0f;
	}
    void InitText()
    {
        if (showFrame)
        {
            if (text != null)
            text.text =  "\n";
        }
    }
    // Update is called once per frame
    void Update () {
        frameNum++;
        globalFrame++;
        if(globalFrame>startframe)
            globalSum += Time.deltaTime;
        fpsSum += 1.0f / (Time.deltaTime);
        //print(globalFrame+"/"+Time.deltaTime + "/"+1.0f/(globalSum/(globalFrame-startframe)));
        
        if (frameNum > 200)
        {
            if (frameStep < 10)
            {
                if (text != null) { 
                    if (showFrame)
                        text.text += "frame = " + fpsSum / 200.0f + "\n";
                    BigSum += fpsSum / 200.0f;
                }
                frameNum = 0;
                fpsSum = 0;
                
            }
            frameStep++;
            if (frameStep == 10)
            {
                text.text += "average= " + BigSum/10.0f+ "\n";
            }
            
        }
    }
}
