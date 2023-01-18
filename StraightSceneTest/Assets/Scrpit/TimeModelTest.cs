namespace VRTK
{
    using UnityEngine;
    using System.Collections.Generic;
    using UnityEngine.UI;
    using System.IO;
    using OfficeOpenXml;
    using System;
    using System.Collections;

    [AddComponentMenu("VRTK/Scripts/Locomotion/TimeModelTest")]
    public class TimeModelTest : MonoBehaviour
    {
        public GameObject picture;
        public enum ControlOptions
        {
            /// <summary>
            /// Track both headset and controllers for movement calculations.
            /// </summary>
            HeadsetAndControllers,
            /// <summary>
            /// Track only the controllers for movement calculations.
            /// </summary>
            ControllersOnly,
            /// <summary>
            /// Track only headset for movement caluclations.
            /// </summary>
            HeadsetOnly,
        }

        public enum DirectionalMethod
        {
            /// <summary>
            /// Will always move in the direction they are currently looking.
            /// </summary>
            Gaze,
            /// <summary>
            /// Will move in the direction that the controllers are pointing (averaged).
            /// </summary>
            ControllerRotation,
            /// <summary>
            /// Will move in the direction they were first looking when they engaged Move In Place.
            /// </summary>
            DumbDecoupling,
            /// <summary>
            /// Will move in the direction they are looking only if their headset point the same direction as their controllers.
            /// </summary>
            SmartDecoupling,
            /// <summary>
            /// Will move in the direction that the controller with the engage button pressed is pointing.
            /// </summary>
            EngageControllerRotationOnly,
            /// <summary>
            /// Will move in the direction that the left controller is pointing.
            /// </summary>
            LeftControllerRotationOnly,
            /// <summary>
            /// Will move in the direction that the right controller is pointing.
            /// </summary>
            RightControllerRotationOnly
        }

        [Header("Control Settings")]

        [Tooltip("If this is checked then the left controller engage button will be enabled to move the play area.")]
        public bool leftController = true;
        [Tooltip("If this is checked then the right controller engage button will be enabled to move the play area.")]
        public bool rightController = true;
        [Tooltip("The button to press to activate the movement.")]
        public VRTK_ControllerEvents.ButtonAlias engageButton = VRTK_ControllerEvents.ButtonAlias.ButtonOnePress; //任务开始按钮
        public VRTK_ControllerEvents.ButtonAlias endButton = VRTK_ControllerEvents.ButtonAlias.TouchpadPress;     //任务结束按钮
        [Tooltip("The device to determine the movement paramters from.")]
        public ControlOptions controlOptions = ControlOptions.HeadsetAndControllers;
        [Tooltip("The method in which to determine the direction of forward movement.")]
        public DirectionalMethod directionMethod = DirectionalMethod.Gaze;

        [Header("Speed Settings")]

        [Tooltip("The speed in which to move the play area.")]
        public float speedScale = 100000;
        [Tooltip("The maximun speed in game units. (If 0 or less, max speed is uncapped)")]
        public float maxSpeed = 4;
        [Tooltip("The speed in which the play area slows down to a complete stop when the engage button is released. This deceleration effect can ease any motion sickness that may be suffered.")]
        public float deceleration = 0.1f;
        [Tooltip("The speed in which the play area slows down to a complete stop when falling is occuring.")]
        public float fallingDeceleration = 0.01f;

        [Header("Advanced Settings")]

        [Tooltip("The degree threshold that all tracked objects (controllers, headset) must be within to change direction when using the Smart Decoupling Direction Method.")]
        public float smartDecoupleThreshold = 30f;
        // The cap before we stop adding the delta to the movement list. This will help regulate speed.
        [Tooltip("The maximum amount of movement required to register in the virtual world.  Decreasing this will increase acceleration, and vice versa.")]
        public float sensitivity = 0.02f;

        [Header("Custom Settings")]

        [Tooltip("An optional Body Physics script to check for potential collisions in the moving direction. If any potential collision is found then the move will not take place. This can help reduce collision tunnelling.")]
        public VRTK_BodyPhysics bodyPhysics;

        protected Transform playArea;
        protected Transform PreviousXZ;
        protected GameObject controllerLeftHand;
        protected GameObject controllerRightHand;

        protected VRTK_ControllerReference engagedController;
        protected Transform Headset;

        protected Vector3 PreHeadsetV3;
        protected Vector3 NowHeadsetV3;
        protected bool leftSubscribed;
        protected bool rightSubscribed;
        protected bool previousLeftControllerState;
        protected bool previousRightControllerState;
        protected VRTK_ControllerEvents.ButtonAlias previousEngageButton; //存储上一个EngageButton
        protected VRTK_ControllerEvents.ButtonAlias previousEndButton; //存储上一个结束按钮
        protected bool currentlyFalling;

        protected int averagePeriod;
        protected List<Transform> trackedObjects = new List<Transform>();
        protected Dictionary<Transform, List<float>> movementList = new Dictionary<Transform, List<float>>();
        protected Dictionary<Transform, float> previousYPositions = new Dictionary<Transform, float>();
        protected Dictionary<Transform, Vector3> previousControllPositions = new Dictionary<Transform, Vector3>();
        protected List<Vector3> controllerVector3 = new List<Vector3>();//记录两个控制器之间的向量
        protected List<Vector3> trackerVector3 = new List<Vector3>();//记录两个tracker之间的向量
        protected Vector3 initialGaze;

        // protected float currentMovement;
        protected float currentAngle;
        protected float currentSpeed;
        protected float currentMovement;
        protected Vector3 currentDirection;
        protected Vector3 previousDirection;
        protected float distanceWindow;
        protected Vector3 trackerPos1;
        protected Vector3 trackerPos2;
        protected bool movementEngaged;
        protected List<string> bodyPart = new List<string> { "head", "hand", "torso", "knees", "hand", "torso", "knees", "head", "torso", "knees", "head", "hand" };
        protected List<int> randomNumList = new List<int>(); //存储已经随机过的数
        protected int randomNum = 0;
        public Text tips;
        public Text testData;
        public GameObject TipCanvas;
        protected int currentMappingMethod;
        protected string currentTarget;
        protected int recordNum = 0;
        protected int recordPoint = 0;
        protected int InvaildNum = 0;
        private float SumX = 0;
        private float SumErrorDis = 0;
        private float SumRightDis = 0;
        protected long startTime;
        protected long endTime;
        protected bool IsDoneTask = true;
        protected bool IsNeedRedoTask = false;
        protected bool IsVaild = true;
        protected ExcelPackage excelPackage;
        protected ExcelWorksheet worksheet;
        protected string MappingDisplay;

        protected List<float> movementAngle = new List<float>(); //角度列表
        protected List<float> bodyMotion = new List<float>(); //肢体交互列表
        protected List<float> timeCost = new List<float>(); //记录时间
        protected List<float> mappingSpeed = new List<float>(); //速度列表
        protected List<float> virtualMovement = new List<float>(); //移动列表

     //   protected List<float> window = new List<float>(); //移动列表

        Coroutine mCoroutine = null;


        //Vector3 preTrackerV3;
        // Vector3 preControllerV3;

        public GameObject tracker1;
        public GameObject tracker2;
        public int UsingBody;
        public string Participant = "";
        public string BlockNum = "";
        protected Vector3 neckV3;
        protected float PreTrackerYPos = 0.1f;
        protected bool IsPressReset = false;
        public GameObject Arrow;
        private float DeadZone = 0.1f;
        [Range(0f, 1f)][SerializeField] private float _leaningDeadzone;
        protected Dictionary<Transform, Vector3> previousAngleV3 = new Dictionary<Transform, Vector3>();
        protected Dictionary<Transform, List<float>> angleList = new Dictionary<Transform, List<float>>();




        /// <summary>
        /// Set the control options and modify the trackables to match.
        /// </summary>
        /// <param name="givenControlOptions">The control options to set the current control options to.</param>
        public virtual void SetControlOptions(ControlOptions givenControlOptions) //选择使用的设备
        {
            controlOptions = givenControlOptions;
            trackedObjects.Clear();

            if (controllerLeftHand != null && controllerRightHand != null && (controlOptions == ControlOptions.HeadsetAndControllers || controlOptions == ControlOptions.ControllersOnly))
            {
                VRTK_SharedMethods.AddListValue(trackedObjects, VRTK_DeviceFinder.GetActualController(controllerLeftHand).transform, true);
                VRTK_SharedMethods.AddListValue(trackedObjects, VRTK_DeviceFinder.GetActualController(controllerRightHand).transform, true);

            }

            if (Headset != null && (controlOptions == ControlOptions.HeadsetAndControllers || controlOptions == ControlOptions.HeadsetOnly))
            {
                VRTK_SharedMethods.AddListValue(trackedObjects, Headset.transform, true);

            }

        }

        public virtual Vector3 GetMovementDirection() //获得移动方向
        {
            return currentDirection;
        }


        protected virtual void Awake()
        {
            VRTK_SDKManager.AttemptAddBehaviourToToggleOnLoadedSetupChange(this);
        }

        protected virtual void Start()
        {
            //DataRecordInit();

        }

        protected virtual void OnEnable()// 初始化设备
        {
            trackedObjects.Clear();
            movementList.Clear();
            previousYPositions.Clear();//之前的y方向的坐标
            angleList.Clear();
            previousAngleV3.Clear();
            initialGaze = Vector3.zero;
            currentDirection = Vector3.zero;
            previousDirection = Vector3.zero;
            distanceWindow = 0.1f; //初始化距离尺度
            averagePeriod = 60;
            currentSpeed = 0f;
            movementEngaged = false;
            previousEngageButton = engageButton;
            previousEndButton = endButton;


            bodyPhysics = (bodyPhysics != null ? bodyPhysics : FindObjectOfType<VRTK_BodyPhysics>());
            controllerLeftHand = VRTK_DeviceFinder.GetControllerLeftHand();
            controllerRightHand = VRTK_DeviceFinder.GetControllerRightHand();

            SetControllerListeners(controllerLeftHand, leftController, ref leftSubscribed);
            SetControllerListeners(controllerRightHand, rightController, ref rightSubscribed);

            Headset = VRTK_DeviceFinder.HeadsetTransform();//获得在VE中头戴式设备的世界坐标
            PreHeadsetV3 = Headset.position; //存储之前更新的头戴式设备位置
            neckV3 = new Vector3(PreHeadsetV3.x, PreHeadsetV3.y - 0.3f, PreHeadsetV3.z);//计算脖子的位置

            SetControlOptions(controlOptions);

            playArea = VRTK_DeviceFinder.PlayAreaTransform();//使用的空间

            VRTK_SharedMethods.AddDictionaryValue(previousControllPositions, controllerLeftHand.transform, trackedObjects[0].transform.localPosition, true);//记录左控制器的位置
            VRTK_SharedMethods.AddDictionaryValue(previousControllPositions, controllerRightHand.transform, trackedObjects[1].transform.localPosition, true);//记录右控制器的位置
            VRTK_SharedMethods.AddListValue(controllerVector3, trackedObjects[0].transform.position - trackedObjects[1].transform.position, true);//记录控制器之间的向量

            // Vector3 preControllerV3 = trackedObjects[1].transform.position - trackedObjects[0].transform.position;

            // Vector3 preTrackerV3 = SteamVR_Controller.Input(5).transform.pos - SteamVR_Controller.Input(6).transform.pos;
            //Vector3 preTrackerV3 = tracker1.transform.localPosition - tracker2.transform.localPosition;
            Vector3 V3X = new Vector3(1, 0, 0);
            Vector3 V3Y = new Vector3(0, 1, 0);

            // Debug.Log("T1位置" + tracker1.transform.localPosition + "T2位置" + tracker2.transform.localPosition + "现在的位置" + V3X);
            VRTK_SharedMethods.AddListValue(trackerVector3, V3X, true);//记录tracker之间的向量
            VRTK_SharedMethods.AddListValue(trackerVector3, V3Y, true);//记录tracker之间的向量
                                                                       //   VRTK_SharedMethods.AddListValue(trackerVector3, V3YT, true);//记录tracker高度的向量
                                                                       //Debug.Log(V3YT);

            // Initialize the lists.
            for (int i = 0; i < trackedObjects.Count; i++) //手部左右控制器
            {
                Transform trackedObj = trackedObjects[i];
                VRTK_SharedMethods.AddDictionaryValue(movementList, trackedObj, new List<float>(), true);//存储坐标点的位置
                VRTK_SharedMethods.AddDictionaryValue(previousYPositions, trackedObj, trackedObj.transform.localPosition.y, true);//存储之前tracker的y坐标和当前y坐标
                                                                                                                                  // Debug.Log("this" + trackedObj.transform.localPosition);
                VRTK_SharedMethods.AddDictionaryValue(angleList, trackedObj, new List<float>(), true);//存储产生的角
                VRTK_SharedMethods.AddDictionaryValue(previousAngleV3, trackedObj, trackedObj.transform.forward, true);//存储之前tracker的y坐标和当前y坐标

            }
        }

        protected virtual void OnDisable()//不激活的话
        {
            SetControllerListeners(controllerLeftHand, leftController, ref leftSubscribed, true);
            SetControllerListeners(controllerRightHand, rightController, ref rightSubscribed, true);

            controllerLeftHand = null;
            controllerRightHand = null;
            Headset = null;
            playArea = null;
        }

        protected virtual void OnDestroy()//销毁
        {
            VRTK_SDKManager.AttemptRemoveBehaviourToToggleOnLoadedSetupChange(this);
        }

        protected virtual void Update()
        {
            CheckControllerState(controllerLeftHand, leftController, ref leftSubscribed, ref previousLeftControllerState); //检查触发器状态，是否开启
            CheckControllerState(controllerRightHand, rightController, ref rightSubscribed, ref previousRightControllerState);
            previousEngageButton = engageButton;
            previousEndButton = endButton;
            // TipCanvas.transform.forward = currentDirection;
            // TipCanvas.transform.position = new Vector3(playArea.position.x, playArea.position.y + 1.8f, playArea.position.z + 2); //提示面板的位置
            //testData.text = "手柄1位置" + trackedObjects[0].transform.localPosition + "手柄2" + trackedObjects[1].transform.localPosition;

            // Debug.Log("Input5的位置：" + SteamVR_Controller.Input(5).transform.pos + "Input6的位置：" + SteamVR_Controller.Input(6).transform.pos);
            // Debug.Log(Headset.transform.localEulerAngles.x);
        }

        protected virtual void FixedUpdate()//注意
        {
            HandleFalling();

            if (MovementActivated() && !currentlyFalling) //①当点击手柄，激活按钮开始运动
            {
                NowHeadsetV3 = Headset.transform.localEulerAngles;//更新后头戴式设备的位置
                float motion = 0f;                                //当前使用的部位
                float angle = 0f;
                CurrentBodyMotion(UsingBody, out angle, out motion);

                SumX = SumX + motion;
                previousDirection = currentDirection;
                currentDirection = SetDirection();
                currentAngle = angle;
                currentMovement = motion;
            }
            else
            {
                currentMovement = 0f;
                currentAngle = 0f;
                currentDirection = Vector3.zero;
                previousDirection = Vector3.zero;
            }

            VRTK_SharedMethods.AddDictionaryValue(previousControllPositions, controllerLeftHand.transform, trackedObjects[0].transform.localPosition, true);
            VRTK_SharedMethods.AddDictionaryValue(previousControllPositions, controllerRightHand.transform, trackedObjects[1].transform.localPosition, true);
            SetDeltaTransformData(); //更新数据
            currentSpeed = MoveViewPoint(currentDirection, currentMovement); //④得到运动速度
            PreHeadsetV3 = NowHeadsetV3;
            CheckMovement();
        }

        protected void CheckMovement()
        {
            Vector3 playAreaPosition = playArea.position;
            Vector3 startPosition = new Vector3(0, 0, 0);
            float movement = Vector3.Distance(startPosition, playAreaPosition);
            if (movement >= distanceWindow)
            {
                movementAngle.Add(currentAngle);                 //记录运动角度
                bodyMotion.Add(currentMovement);     //记录物理运动
                mappingSpeed.Add(currentSpeed);      //记录运动速度
                var timeForDataPoint = TaskTime1();  //记录任务开始到当前状态花费的时间
                timeCost.Add(timeForDataPoint);
                virtualMovement.Add(movement);       //记录当前的运动间隔
               // window.Add(distanceWindow);
                distanceWindow = distanceWindow + 0.1f;
            }
        }

        IEnumerator RecordDataPoint(float steptime)
        {
            while (true)
            {
                bodyMotion.Add(currentMovement);
                mappingSpeed.Add(currentSpeed);
                var timeForDataPoint = TaskTime1(); //每个数据点之间的时间间隔
                timeCost.Add(timeForDataPoint);
                yield return new WaitForSeconds(steptime);
            }
            //开始任务后，每0.04s进行一次采样。当用户完成一次任务，再把所有信息写入。
        }

        protected void StopRecord() //停止记录和清空列表
        {
            StopCoroutine(RecordDataPoint(0.04f)); //停止携程
        }

        protected void ClearList() //停止记录和清空列表
        {
            movementAngle.Clear();
            bodyMotion.Clear();
            mappingSpeed.Clear();
            timeCost.Clear();
            virtualMovement.Clear();
          //  window.Clear();
            distanceWindow = 0.1f;
        }


        /// <summary>
        /// 随机组合
        /// </summary>
        /// <param name="randomNum"></param>
        /// <returns></returns>

        protected void CurrentBodyMotion(int randomNum, out float angle, out float motion) //当前使用的肢体部位
        {
            float returnAngle = 0f;
            float returnDistance = 0f;

            switch (randomNum)
            {
                case 1: CalculateHeadAngle(out returnAngle, out returnDistance); break; //case1头部(角度变化)
                                                                                        // case 2: returnDistance = CalculateArmAngle(); break;//case2手部(角度变化)
                case 2: CalculateTorsoAngle(out returnAngle, out returnDistance); break;//case3躯干(角度变化)
                case 3: CalculateAngleUpdate(out returnAngle, out returnDistance); break;//case4膝盖(角度变化)
                //case 5: returnDistance = CalculateFeetDistance(); break;//case5脚后跟(角度变化)
                default: break;
            }

            angle = returnAngle;
            motion = returnDistance;
        }


        protected virtual bool CheckRandom(int num)//随机距离和映射函数的组合
        {
            bool t = false;
            for (int i = 0; i < randomNumList.Count; i++)
            {

                if (randomNumList[i] == num)
                {

                    t = true;
                    break;
                }
            }
            return t;
        }
        protected virtual int RandomDistanceAndMapping()//随机距离和映射函数的组合(正式实验)
        {
            int num = UnityEngine.Random.Range(1, 16);//5种距离和3种映射关系的组合为15

            if (randomNumList.Count == 15) //检查是否已经随机过
            {
                randomNumList.Clear();
            }
            while (CheckRandom(num))
            {
                num = UnityEngine.Random.Range(1,16);
                CheckRandom(num);
            }

            VRTK_SharedMethods.AddListValue(randomNumList, num);
            //  num = 14;
            return num;
        }

        protected virtual int TestRandomDistanceAndMapping()//随机距离和映射函数的组合(给用户测试的一种距离,8种函数)
        {
            int num = UnityEngine.Random.Range(5, 9);//3种距离和6种映射关系的组合为18

            if (randomNumList.Count == 4)
            {
                randomNumList.Clear();
            }
            while (CheckRandom(num))
            {
                num = UnityEngine.Random.Range(5, 9);
                CheckRandom(num);
            }

            VRTK_SharedMethods.AddListValue(randomNumList, num);
            // num = 3;
            return num;
        }

        protected virtual int MappingBasedRandom(int randomNum)//根据随机数判断使用的映射函数（除以5）
        {
            int mappingBasedRandom = -1;

            mappingBasedRandom = randomNum % 3;  //使用的是除数不是余数
            if (mappingBasedRandom == 0) //一共有1，2，3，4四种函数，当刚好可以被除尽时，就是使用函数4
            {
                mappingBasedRandom = 3;
            }

            //mappingBasedRandom = 1;

            return mappingBasedRandom;
        }

        protected virtual string TargetBasedRandom(int randomNum)//根据随机数判断使用的目标距离(1-4为A点，5-8为B点，9-12为C点，13-16为D点，17-20为E点)
        {

            string targetRandom = "";

            if (randomNum > 16)
            {
                targetRandom = "E";
            }
            else if (randomNum > 12)
            {
                targetRandom = "D";
            }
            else if (randomNum > 8)
            {
                targetRandom = "C";
            }
            else if (randomNum > 4)
            {
                targetRandom = "B";
            }
            else
            {
                targetRandom = "A";
            }
            return targetRandom;
        }

        protected virtual void RandomEvent()
        {
            int a = 0;
            float ArrowZ = 0;
            if (Participant.Equals("test"))
            {
                a = TestRandomDistanceAndMapping();
            }
            else
            {
                a = RandomDistanceAndMapping();   //正式实验随机组合
            }

            MappingDisplay = "";
            currentTarget = TargetBasedRandom(a); //随机出目标点
            currentMappingMethod = MappingBasedRandom(a);

            switch (currentMappingMethod)
            {
                case 1: MappingDisplay = "线性函数(k=1)"; break;
                //  case 2: MappingDisplay = "线性1.8"; break;
                //  case 2: MappingDisplay = "2次幂函数"; break;
                case 2: MappingDisplay = "线性函数(k=2)"; break;
                // case 3: MappingDisplay = "分段线性函数"; break;
                case 3: MappingDisplay = "线性函数(k=3)"; break;
               // case 4: MappingDisplay = "分段函数(常数+幂函数)"; break;
                // case 6: MappingDisplay = "线性函数+幂函数"; break;
                default: break;
            }

            switch (currentTarget)
            {
                case "A": ArrowZ = 3; break;
                case "B": ArrowZ = 6; break;
                case "C": ArrowZ = 9; break;
                case "D": ArrowZ = 12; break;
                case "E": ArrowZ = 15; break;
                default: break;
            }

            ActiveCanvas();
            tips.text = "使用“" + MappingDisplay + "”到指示点";
            Invoke("CloseCanvas", 2f);
            Arrow.transform.position = new Vector3(0, 2, ArrowZ); //将指示箭头移动到目标位置

        }

        private void ActiveCanvas()
        {
            TipCanvas.SetActive(true);
        }

        private void CloseCanvas()
        {
            TipCanvas.SetActive(false);
        }
        /// <summary>
        /// 随机组合
        /// </summary>


        /// <summary>
        /// 计算肢体产生的位移
        /// </summary>
        /// <returns></returns>
        protected virtual float CalculateDistance()//计算手部运动得距离
        {
            float curdis = 0f;
            curdis = (trackedObjects[0].transform.localPosition - trackedObjects[1].transform.localPosition).magnitude;
            if (curdis < 0.4)//假设两个控制器之间的距离小于0.4的距离，默认不移动，当大于0.4把超出的距离做函数映射
            {
                curdis = 0;
            }
            else
            {
                curdis = curdis - 0.4f;
            }
            return curdis;

        }

        protected virtual float CalculateHeadDistanceUpgrade()//计算头部发生的位移
        {
            Vector3 preTrackerV3 = trackerVector3[1];//指向y轴的向量
            Vector3 nowTrackerV3 = Headset.transform.localPosition - tracker1.transform.localPosition;
            //需要将向量投影到xy平面
            preTrackerV3 = preTrackerV3 - Vector3.Project(preTrackerV3, Vector3.forward);
            nowTrackerV3 = nowTrackerV3 - Vector3.Project(nowTrackerV3, Vector3.forward);
            float angle = Vector3.Angle(preTrackerV3, nowTrackerV3);
            angle = angle / 180 * Mathf.PI;//转化为角度
            angle = Mathf.Sin(angle);
            float dir = (Vector3.Dot(Vector3.forward, Vector3.Cross(nowTrackerV3, preTrackerV3)) < 0 ? -1 : 1);
            angle *= dir;
            Debug.Log("I5" + Headset.transform.localPosition + "I6" + tracker1.transform.localPosition + "之前的向量：" + preTrackerV3 + "现在的向量：" + nowTrackerV3 + "计算得到的角度" + angle);
            if (angle < 0.1)//假设两个控制器之间的距离小于0.4的距离，默认不移动，当大于0.4把超出的距离做函数映射
            {
                angle = 0;
            }
            else
            {
                angle = angle - 0.1f;
            }
            return angle;
        }

        protected void CalculateHeadAngle(out float angle, out float motion)
        {
            float retHead = 0f;
            float HeadPos = Headset.transform.localEulerAngles.x;
            if (HeadPos > 270)
            {
                retHead = 0;
            }
            else
            {
                retHead = HeadPos;
            }

            if (retHead < 0)//假设两个控制器之间的距离小于0.4的距离，默认不移动，当大于0.4把超出的距离做函数映射
            {
                retHead = 0;
            }

            angle = retHead;

            retHead = retHead / 20f;

            if (retHead > 1)
            {
                retHead = 1;
            }

            motion = retHead;
        }

        protected virtual float CalculateDistanceUpgrade()//计算其他部位发生的位移
        {
            float currentDistance = (SteamVR_Controller.Input(5).transform.pos - SteamVR_Controller.Input(6).transform.pos).magnitude;
            if (currentDistance < 0.4)//假设两个控制器之间的距离小于0.4的距离，默认不移动，当大于0.4把超出的距离做函数映射
            {
                currentDistance = 0;
            }
            else
            {
                currentDistance = currentDistance - 0.4f;
            }
            return currentDistance;
        }

        protected virtual float CalculateDistanceUpgrade1(Vector3 PosA, Vector3 PosB)
        {
            return (PosA - PosB).magnitude;
        }

        protected virtual float CalculateAngle() //计算运动的角度
        {
            Vector3 preControllerV3 = controllerVector3[0];
            Vector3 nowControllerV3 = trackedObjects[0].transform.localPosition - trackedObjects[1].transform.localPosition;
            VRTK_SharedMethods.AddListValue(controllerVector3, nowControllerV3, true);
            //使向量处于同一个平面，这里平面为XY,Vector3.Project计算向量在指定轴上的投影，向量本身减去此投影向量就为在平面上的向量
            preControllerV3 = preControllerV3 - Vector3.Project(preControllerV3, Vector3.up);
            nowControllerV3 = nowControllerV3 - Vector3.Project(nowControllerV3, Vector3.up);
            float angle = Vector3.Angle(preControllerV3, nowControllerV3);
            angle = angle / 180 * Mathf.PI;//转化为角度
            angle = Mathf.Sin(angle);
            if (angle < 0.1)//假设两个控制器之间的距离小于0.4的距离，默认不移动，当大于0.4把超出的距离做函数映射
            {
                angle = 0;
            }
            else
            {
                angle = angle - 0.1f;
            }
            ////计算方向,Vector3.Cross 叉乘返回为同时垂直于两个参数向量的向量，方向可朝上也可朝下，由两向量夹角的方向决定。
            ////Vector3.Dot 点乘意义为两参数向量方向完全相同返回1，完全相反返回 - 1,垂直返回0。当两向量角度减小，将得到更大的值。
            //float dir = (Vector3.Dot(Vector3.up, Vector3.Cross(preControllerV3, nowControllerV3)) < 0 ? -1 : 1);
            //angle *= dir;
            //Debug.Log(angle);
            //angle = angle / 180 * Mathf.PI;//转化为角度
            ////Debug.Log("p"+angle);
            //angle = Mathf.Sin(angle);
            ////Debug.Log("I5" + trackedObjects[0].transform.localPosition + "I6" + trackedObjects[1].transform.localPosition + "之前的向量：" + preControllerV3 + "现在的向量：" + nowControllerV3 + "计算得到的角度" + angle);
            //angle = Mathf.Abs(angle);//取绝对值

            Debug.Log("手柄角度" + angle);



            return angle;
        }

        protected void CalculateAngleUpdate(out float angle, out float motion)//使用膝盖交互
        {
            Vector3 preTrackerV3 = trackerVector3[0];
            //Vector3 nowTrackerV3 = SteamVR_Controller.Input(5).transform.pos - SteamVR_Controller.Input(6).transform.pos; //把更新后的向量存进列表
            Vector3 nowTrackerV3 = tracker2.transform.localPosition - tracker1.transform.localPosition;

            //VRTK_SharedMethods.AddListValue(trackerVector3, nowTrackerV3, true);
            // 使向量处于同一个平面，向量需要保持在xz的平面,计算向量在指定轴上的投影，向量本身减去此投影向量就为在平面上的向量
            preTrackerV3 = preTrackerV3 - Vector3.Project(preTrackerV3, Vector3.up);
            nowTrackerV3 = nowTrackerV3 - Vector3.Project(nowTrackerV3, Vector3.up);


            float angle1 = Vector3.Angle(preTrackerV3, nowTrackerV3);
            //angle = angle / 180 * Mathf.PI;//转化为角度
            //angle = Mathf.Sin(angle);
            float dir = (Vector3.Dot(Vector3.up, Vector3.Cross(nowTrackerV3, preTrackerV3)) < 0 ? -1 : 1);
            angle1 *= dir;

            if (angle1 < 0)//假设两个控制器之间的距离小于0.4的距离，默认不移动，当大于0.4把超出的距离做函数映射
            {
                angle1 = 0;
            }

            angle = angle1;

            angle1 = angle1 / 20f;

            if (angle1 > 1)
            {
                angle1 = 1;
            }

            motion = angle1;
        }

        //protected virtual float CalculateTorsoAngle()//计算躯干角度
        //{

        //    Vector3 preTrackerV3 = trackerVector3[1];//指向y轴的向量
        //    Vector3 nowTrackerV3 = Headset.transform.localPosition - tracker1.transform.localPosition;
        //    //需要将向量投影到yz平面
        //    preTrackerV3 = preTrackerV3 - Vector3.Project(preTrackerV3, Vector3.right);
        //    nowTrackerV3 = nowTrackerV3 - Vector3.Project(nowTrackerV3, Vector3.right);
        //    float angle = Vector3.Angle(preTrackerV3, nowTrackerV3);
        //    angle = angle / 180 * Mathf.PI;//转化为角度
        //    angle = Mathf.Sin(angle);
        //    float dir = (Vector3.Dot(Vector3.right, Vector3.Cross(preTrackerV3, nowTrackerV3)) < 0 ? -1 : 1);
        //    angle *= dir;
        //    Debug.Log("I5" + Headset.transform.localPosition + "I6" + tracker1.transform.localPosition + "之前的向量：" + preTrackerV3 + "现在的向量：" + nowTrackerV3 + "计算得到的角度" + angle);
        //    if (angle < 0.1)//假设两个控制器之间的距离小于0.4的距离，默认不移动，当大于0.4把超出的距离做函数映射
        //    {
        //        angle = 0;
        //    }
        //    else
        //    {
        //        angle = angle - 0.1f;
        //    }
        //    //float _outputAngle = GetInput(angle);
        //    return angle; 
        //}

        protected void CalculateTorsoAngle(out float angle, out float motion)//计算躯干角度
        {

            Vector3 preTrackerV3 = trackerVector3[1];//tracker位置
            Vector3 nowTrackerV3 = Headset.transform.localPosition - tracker1.transform.localPosition; //相减得到变化后的向量，需要将向量投影到yz平面
            preTrackerV3 = preTrackerV3 - Vector3.Project(preTrackerV3, Vector3.right);
            nowTrackerV3 = nowTrackerV3 - Vector3.Project(nowTrackerV3, Vector3.right);
            float angle1 = Vector3.Angle(preTrackerV3, nowTrackerV3);
            float dir = (Vector3.Dot(Vector3.right, Vector3.Cross(preTrackerV3, nowTrackerV3)) < 0 ? -1 : 1);
            angle1 *= dir;
            if (angle1 < 0)
            {
                angle1 = 0;
            }

            angle = angle1;

            float displacement = angle1 / 20f; //规范化过程

            if (displacement > 1)
            {
                displacement = 1;
            }
            motion = displacement;
        }

        protected virtual float CalculateFeetDistance()//计算脚距离地面的位置
        {
            float retYDis = 0f;
            retYDis = tracker2.transform.localPosition.y - PreTrackerYPos;
            if (retYDis < 0.05)
            {
                retYDis = 0;
            }
            Debug.Log("高度" + retYDis + "t1" + tracker2.transform.localPosition + "t2" + PreTrackerYPos);
            return retYDis;
        }

        protected virtual float CalculateListAverage() //手部Swing
        {
            float listAverage = 0;

            for (int i = 0; i < trackedObjects.Count; i++)
            {
                Transform trackedObj = trackedObjects[i];
                // Get the amount of Y movement that's occured since the last update.获取自上次更新以来发生的Y移动量。
                float previousYPosition = VRTK_SharedMethods.GetDictionaryValue(previousYPositions, trackedObj);
                float deltaYPostion = Mathf.Abs(previousYPosition - trackedObj.transform.localPosition.y);

                // Convenience code.
                List<float> trackedObjList = VRTK_SharedMethods.GetDictionaryValue(movementList, trackedObj, new List<float>(), true);

                // Cap off the speed.
                if (deltaYPostion > sensitivity) //如果y的位置超过预设值，则把预设值存进列表
                {
                    VRTK_SharedMethods.AddListValue(trackedObjList, sensitivity);
                }
                else//否则直接把y的位置存进列表
                {
                    VRTK_SharedMethods.AddListValue(trackedObjList, deltaYPostion);
                }

                if (trackedObjList.Count > averagePeriod)
                {
                    trackedObjList.RemoveAt(0);
                }


                if (IsPressReset)
                {
                    trackedObjList.Clear();
                }
                // Average out the current tracker's list.
                float sum = 0;
                // Debug.Log("trackedObjList.Countn      " + trackedObjList.Count);
                for (int j = 0; j < trackedObjList.Count; j++)
                {
                    float diffrences = trackedObjList[j];
                    sum += diffrences;
                }
                float avg = sum / averagePeriod;
                avg = avg * 60;
                // float avg = sum;
                // Add the average to the the list average.
                listAverage += avg;
                Debug.Log("listAverage" + listAverage);
            }
            if (listAverage < 0.1)
            {
                listAverage = 0;
            }
            return listAverage;
        }

        protected virtual float CalculateArmAngle()//手臂角度
        {
            float listAverage = 0;

            for (int i = 0; i < trackedObjects.Count; i++)
            {
                Transform trackedObj = trackedObjects[i];
                Vector3 preAngleV3 = VRTK_SharedMethods.GetDictionaryValue(previousAngleV3, trackedObj);
                float deltaAngle = Vector3.Angle(preAngleV3, trackedObj.transform.forward);
                //deltaAngle = Normalization(deltaAngle);

                List<float> trackedObjList = VRTK_SharedMethods.GetDictionaryValue(angleList, trackedObj, new List<float>(), true);


                if (deltaAngle > 1)
                {
                    VRTK_SharedMethods.AddListValue(trackedObjList, 1);
                }
                else
                {
                    VRTK_SharedMethods.AddListValue(trackedObjList, deltaAngle);
                }

                if (trackedObjList.Count > averagePeriod)
                {
                    trackedObjList.RemoveAt(0);
                }

                if (IsPressReset)
                {
                    trackedObjList.Clear();
                }
                float sum = 0;
                for (int j = 0; j < trackedObjList.Count; j++)
                {
                    float diffrences = trackedObjList[j];
                    sum += diffrences;
                }
                float avg = sum / averagePeriod;
                listAverage += avg;
                //Debug.Log("listAverage" + listAverage);
            }
            if (listAverage < 0.1)
            {
                listAverage = 0;
            }
            return listAverage;
        }

        protected virtual void SetDeltaTransformData() //更新Swing的数据
        {
            for (int i = 0; i < trackedObjects.Count; i++)
            {
                Transform trackedObj = trackedObjects[i];
                // Get delta postions and rotations
                VRTK_SharedMethods.AddDictionaryValue(previousYPositions, trackedObj, trackedObj.transform.localPosition.y, true);
                VRTK_SharedMethods.AddDictionaryValue(previousAngleV3, trackedObj, trackedObj.transform.forward, true);

            }
        }

        protected float Normalization(float angle)
        {
           //loat xDir = 0f;
            if (angle > 180)
            {
                angle = angle - 360;
            }
            angle = angle / 20;
            if (angle < 0 && angle > -DeadZone || angle > 0 && angle < DeadZone)
            {
                angle = 0;
            }
            Vector2 _input = new Vector2(0, angle);
            if (_input.sqrMagnitude > 1)
            {
                _input.Normalize();
            }
            return _input.y;
        }

        protected float GetInput(float angle) //把映射角度规范化到0,1然后进行映射
        {
            float horizontal = 0; // no strafing
            if (angle > 90)
            {
                angle -= 360;
            }

            angle = angle / 20;
            Debug.Log("angle" + angle);
            if (angle < _leaningDeadzone)
            {
                angle = 0;
            }
            Vector2 _input = new Vector2(horizontal, angle);

            // normalize input if it exceeds 1 in combined length:
            if (_input.sqrMagnitude > 1)
            {
                _input.Normalize();
            }
            Debug.Log(_input.y);
            return _input.y;
        }

        /// <summary>
        /// 计算肢体产生的位移
        /// </summary>
        /// <returns></returns>


        /// <summary>
        /// 映射函数
        /// </summary>
        protected virtual float MappingMethods(int a, float movement)
        {
            float y = 0;
            float x = movement;
            switch (a)
            {
                case 1: y =  x; break; //线性函数
                //case 2: y = 1.8f * x; break; //斜率为1.8的线性函数
                //case 2:
                //    {
                //        y = Mathf.Pow((x / 100), 2) * 100 * 100; //2次幂函数
                //        //Debug.Log("L2"+y);
                //    }; break;
                case 2:
                    {
                        y = 2*x;
                        // y = 1.4f * Mathf.Pow(x, 3); //3次幂函数
                        //Debug.Log("L3" + y);
                    }; break;
                //case 3:
                //    {                                                    //分段线性函数
                //        float s = 0f;
                //        float M = 2f;
                //        float R = 0f;

                //        if (currentTarget == "A")
                //        {
                //            s = (playArea.position - new Vector3(0, 0, 5)).magnitude;
                //            /* M = 2.19f*/
                //            ;
                //            R = 1.83f;
                //        }
                //        else if (currentTarget == "B")
                //        {
                //            s = (playArea.position - new Vector3(0, 0, 10)).magnitude;
                //            //M = 4.60f;
                //            R = 3.5f;
                //        }
                //        else
                //        {
                //            s = (playArea.position - new Vector3(0, 0, 15)).magnitude;
                //            //M = 9.17f;
                //            R = 5.71f;
                //        }

                //        if (s < 0.5)
                //        {
                //            y = x;
                //        }
                //        else if (s < R)
                //        {
                //            y = (1 + (M - 1) * ((s - 0.3f) / (R - 0.3f))) * x;
                //        }
                //        else
                //        {
                //            y = M * x;
                //        }
                //    }; break;
                case 3:
                    {
                        y = 3 * x;
                        //Debug.Log("speed" + x / Time.fixedDeltaTime);
                        //float yt = Mathf.Min(x / Time.fixedDeltaTime, 0.5f) / 0.5f;

                        ////分段函数（常数函数+最小值函数）
                        //if (yt < 0.05)
                        //{
                        //    y = 0;
                        //}
                        //else
                        //{
                        //    y = 3.5f * yt;
                        //}

                    }; break;
                //case 4:
                //    {                                               //分段函数（常数函数+幂函数）
                //        if (x < 0.05)
                //        {
                //            y = 0;
                //        }
                //        else
                //        {
                //            y = Mathf.Pow((x - 0.05f) / 0.95f, 2);
                //        }
                //    }; break;
                //    case 6:
                //        {                                               //分段函数（线性函数+幂函数）
                //            if (x < 0.43)
                //            {
                //                y = x;
                //            }
                //            else
                //            {

                //                y = x + Mathf.Pow(x - 0.43f, 2) / 6;
                //        //            }
                //}; break;
                default: break;
            }
            // Debug.Log("函数"+a+":"+ y);
            return y;
        }
        /// <summary>
        /// 映射函数
        /// </summary>
        /// 

        /// <summary>
        /// 控制运动与方向
        /// </summary>

        protected virtual bool MovementActivated()
        {
            return (movementEngaged || engageButton == VRTK_ControllerEvents.ButtonAlias.Undefined); //主要看movementEngaged
        }

        protected virtual void CheckControllerState(GameObject controller, bool controllerState, ref bool subscribedState, ref bool previousState)
        {
            if (controllerState != previousState || engageButton != previousEngageButton || endButton != previousEndButton)
            {
                SetControllerListeners(controller, controllerState, ref subscribedState);
            }
            previousState = controllerState;
        }


        protected virtual Vector3 HeadsetPosition()
        {
            return (Headset != null ? new Vector3(Headset.forward.x, 0, Headset.forward.z) : Vector3.zero);

        }

        protected virtual Vector3 SetDirection()
        {
            switch (directionMethod)
            {
                case DirectionalMethod.SmartDecoupling:
                case DirectionalMethod.DumbDecoupling:
                    return CalculateCouplingDirection();
                case DirectionalMethod.ControllerRotation:
                    return CalculateControllerRotationDirection(DetermineAverageControllerRotation() * Vector3.forward);
                case DirectionalMethod.LeftControllerRotationOnly:
                    return CalculateControllerRotationDirection((controllerLeftHand != null ? controllerLeftHand.transform.rotation : Quaternion.identity) * Vector3.forward);
                case DirectionalMethod.RightControllerRotationOnly:
                    return CalculateControllerRotationDirection((controllerRightHand != null ? controllerRightHand.transform.rotation : Quaternion.identity) * Vector3.forward);
                case DirectionalMethod.EngageControllerRotationOnly:
                    return CalculateControllerRotationDirection((engagedController != null ? engagedController.scriptAlias.transform.rotation : Quaternion.identity) * Vector3.forward);
                case DirectionalMethod.Gaze:
                    return HeadsetPosition();
            }

            return Vector2.zero;
        }

        protected virtual Vector3 CalculateCouplingDirection()
        {
            // If we haven't set an inital gaze yet, set it now.
            // If we're doing dumb decoupling, this is what we'll be sticking with.
            if (initialGaze == Vector3.zero)
            {
                initialGaze = HeadsetPosition();
            }

            // If we're doing smart decoupling, check to see if we want to reset our distance.
            if (directionMethod == DirectionalMethod.SmartDecoupling)
            {
                bool closeEnough = true;
                float curXDir = (Headset != null ? Headset.rotation.eulerAngles.y : 0f);
                if (curXDir <= smartDecoupleThreshold)
                {
                    curXDir += 360;
                }

                closeEnough = closeEnough && (Mathf.Abs(curXDir - controllerLeftHand.transform.rotation.eulerAngles.y) <= smartDecoupleThreshold);
                closeEnough = closeEnough && (Mathf.Abs(curXDir - controllerRightHand.transform.rotation.eulerAngles.y) <= smartDecoupleThreshold);

                // If the controllers and the headset are pointing the same direction (within the threshold) reset the direction the player's moving.
                if (closeEnough)
                {
                    initialGaze = HeadsetPosition();
                }
            }
            return initialGaze;
        }

        protected virtual Vector3 CalculateControllerRotationDirection(Vector3 calculatedControllerDirection)
        {
            return (Vector3.Angle(previousDirection, calculatedControllerDirection) <= 90f ? calculatedControllerDirection : previousDirection);
        }


        //protected virtual void MoveViewPoint(Vector3 moveDirection, float moveDistance)//控制视点移动
        //{
        //    float distance = moveDistance;
        //    Vector3 movement = moveDirection * MappingMethods(currentMappingMethod, distance);
        //    if (playArea != null)
        //    {
        //        Vector3 finalPosition = new Vector3(movement.x + playArea.position.x, playArea.position.y, movement.z + playArea.position.z);
        //        if (CanMove(bodyPhysics, playArea.position, finalPosition))
        //        {
        //            playArea.position = finalPosition;
        //        }
        //    }
        //}
        protected virtual float MoveViewPoint(Vector3 moveDirection, float motion)//控制视点移动
        {
            float mappingSpeed = MappingMethods(currentMappingMethod, motion);
            Vector3 movement = moveDirection * mappingSpeed * Time.fixedDeltaTime;//速度乘以时间等于位移
            //Debug.Log("speed"+ mappingSpeed+"movement" + movement);
            if (playArea != null)
            {
                Vector3 finalPosition = new Vector3(movement.x + playArea.position.x, playArea.position.y, movement.z + playArea.position.z);
                // Debug.Log(finalPosition);
                if (CanMove(bodyPhysics, playArea.position, finalPosition))
                {
                    playArea.position = finalPosition;
                }
            }
            CheckViewPoint(mappingSpeed);//每次发生移动后检查PlayArea是否还在路径上

            //}
            return mappingSpeed;
        }

        protected virtual void CheckViewPoint(float mappingDistance) //检查PlayArea在不在路径上
        {
            if (EorrorDistance.IsEnterTrigger == true)
            {

                SumRightDis = SumRightDis + mappingDistance;
                //for (int i = 0; i < IndicateRoad.Count; i++)
                //{
                //    IndicateRoad[i].GetComponent<Renderer>().material.color = Color.blue;
                //}
            }
            else
            {
                SumErrorDis = SumErrorDis + mappingDistance;
                //for (int i = 0; i < IndicateRoad.Count; i++)
                //{
                //    IndicateRoad[i].GetComponent<Renderer>().material.color = Color.red;
                //}
                //   Debug.Log("错误距离" + SumErrorDis);
            }
        }

        protected virtual void ResetViewPoint() //重置视点
        {
            playArea.position = new Vector3(0, 0, 0);
        }

        protected virtual bool CanMove(VRTK_BodyPhysics givenBodyPhysics, Vector3 currentPosition, Vector3 proposedPosition)
        {
            if (givenBodyPhysics == null)
            {
                return true;
            }

            Vector3 proposedDirection = (proposedPosition - currentPosition).normalized;
            float distance = Vector3.Distance(currentPosition, proposedPosition);
            return !givenBodyPhysics.SweepCollision(proposedDirection, distance);
        }

        protected virtual void HandleFalling()
        {
            if (bodyPhysics != null && bodyPhysics.IsFalling())
            {
                currentlyFalling = true;
            }

            if (bodyPhysics != null && !bodyPhysics.IsFalling() && currentlyFalling)
            {
                currentlyFalling = false;
                currentSpeed = 0f;
            }
        }

        /// <summary>
        /// 控制运动与方向
        /// </summary>


        /// <summary>
        /// 数据存储
        /// </summary>
        protected virtual string BodyCodeName() //肢体代号
        {
            string body = "";
            switch (UsingBody)
            {
                case 1: body = "head"; break;
                case 2: body = "hand"; break;
                case 3: body = "torso"; break;
                case 4: body = "knees"; break;
                default: break;
            }
            return body;
        }

        protected virtual string FunCodeName()//函数代号
        {
            string fun = "";
            switch (currentMappingMethod)
            {
                case 1: fun = "L(k=1)"; break;
                case 2: fun = "L(k=2)"; break;
                case 3: fun = "L(k=3)"; break;
             //   case 4: fun = "CP"; break;
                //case 5: fun = "LLL"; break;
                //case 6: fun = "CM"; break;
                //case 7: fun = "CP"; break;
                //case 8: fun = "LP"; break;
                default: break;
            }
            return fun;
        }

        protected virtual float TargetCodeName() //目标代号
        {
            float target = 0;
            switch (currentTarget)
            {
                case "A": target = 3; break;
                case "B": target = 6; break;
                case "C": target = 9; break;
                case "D": target = 12; break;
                case "E": target = 15; break;
                default: break;
            }
            return target;
        }



        protected virtual float OffsetTarget() //任务结束时，距离目标点的位置
        {
            float offsetDistance = 0;
            Vector3 playAreaPostion = playArea.position;
            if (currentTarget == "A")
            {
                offsetDistance = (playAreaPostion - new Vector3(0, 0, 3)).magnitude;
            }
            else if (currentTarget == "B")
            {
                offsetDistance = (playAreaPostion - new Vector3(0, 0, 6)).magnitude;
            }
            else if (currentTarget == "C")
            {
                offsetDistance = (playAreaPostion - new Vector3(0, 0, 9)).magnitude;
            }
            else if (currentTarget == "D")
            {
                offsetDistance = (playAreaPostion - new Vector3(0, 0, 12)).magnitude;
            }
            else if (currentTarget == "E")
            {
                offsetDistance = (playAreaPostion - new Vector3(0, 0, 15)).magnitude;
            }
            return Mathf.Abs(offsetDistance);
        }

        protected virtual long TaskTime1() //完成任务的时间
        {
            endTime = System.DateTime.Now.Ticks;
            long currentMillis = (endTime - startTime) / 10000;
            //string tasktime = currentMillis.ToString();
            // Debug.Log(tasktime);
            return currentMillis;
        }

        protected virtual void DataRecordInit()//excel初始化
        {
            string FilePath = Application.streamingAssetsPath + "/DataRecord.xlsx";

            //获取excel的信息
            FileInfo fileInfo = new FileInfo(FilePath);
            //通过excel文件信息打开excel表格
            using (excelPackage = new ExcelPackage(fileInfo))
            {
                int sheetNum = excelPackage.Workbook.Worksheets.Count;
                if (excelPackage.Workbook.Worksheets[sheetNum].Cells[2, 1].Value != null)
                {
                    sheetNum++;
                    worksheet = excelPackage.Workbook.Worksheets.Add("Sheet" + sheetNum);
                    worksheet.Cells[1, 1].Value = "Participant";
                    worksheet.Cells[1, 2].Value = "Block";
                    worksheet.Cells[1, 3].Value = "Index";
                    worksheet.Cells[1, 4].Value = "BodyPart";
                    worksheet.Cells[1, 5].Value = "FunctionID";
                    worksheet.Cells[1, 6].Value = "Distance(m)";
                    worksheet.Cells[1, 7].Value = "TaskTime(ms)";
                    worksheet.Cells[1, 8].Value = "TargetOffsetDistance(m)";
                    excelPackage.Save();
                }
                else
                {
                    worksheet = excelPackage.Workbook.Worksheets[sheetNum];
                }
            }

        }


        protected virtual void DataRecord() //数据记录
        {
            string FilePath = Application.streamingAssetsPath + "/DataRecordMovementToSpeed.xlsx";

            //获取excel的信息
            FileInfo fileInfo = new FileInfo(FilePath);
            //通过excel文件信息打开excel表格
            using (excelPackage = new ExcelPackage(fileInfo))
            {

                worksheet = excelPackage.Workbook.Worksheets[excelPackage.Workbook.Worksheets.Count];
                //if (Participant.CompareTo("test") == 1)
                //{

                //}
                int endRow = worksheet.Dimension.End.Row + 1;
                //ebug.Log(endRow);
                //存储一个测试的所有数据
                worksheet.Cells[endRow, 1].Value = Participant;
                worksheet.Cells[endRow, 2].Value = BlockNum;
                worksheet.Cells[endRow, 3].Value = recordNum;
                worksheet.Cells[endRow, 4].Value = BodyCodeName();
                worksheet.Cells[endRow, 5].Value = FunCodeName();
                worksheet.Cells[endRow, 6].Value = TargetCodeName();
                worksheet.Cells[endRow, 7].Value = "None";
                worksheet.Cells[endRow, 8].Value = "None";
                worksheet.Cells[endRow, 9].Value = TaskTime1();
                worksheet.Cells[endRow, 10].Value = OffsetTarget();
                worksheet.Cells[endRow, 11].Value = SumX;
                worksheet.Cells[endRow, 12].Value = SumRightDis;
                worksheet.Cells[endRow, 13].Value = SumErrorDis;
                worksheet.Cells[endRow, 14].Value = SumRightDis / (SumRightDis + SumErrorDis);
                worksheet.Cells[endRow, 15].Value = (SumRightDis + SumErrorDis) / TargetCodeName();
                //检查这组数据是否合理

                string taskTime = worksheet.Cells[endRow, 9].Value.ToString();
                int taskTime1 = int.Parse(taskTime);
                string taskOffset = worksheet.Cells[endRow, 10].Value.ToString();
                float taskOffset1 = float.Parse(taskOffset);
                //   Debug.Log("Time"+taskTime1+"Offset"+ taskOffset1);
                if (taskOffset1 > 0.5)
                {
                    ActiveCanvas();
                    tips.text = "该次任务数据无效，请认真重新完成任务。\n按开始trigger重新开始任务。";
                    worksheet.Cells[endRow, 16].Value = "Invalid!";
                    InvaildNum++;
                    IsNeedRedoTask = true;//需要重做任务
                }
                else
                {
                    worksheet.Cells[endRow, 16].Value = InvaildNum;
                    InvaildNum = 0;
                    ActiveCanvas();
                    tips.text = "任务完成，数据已被记录。\n按开始按钮开启下一个任务。";
                    IsNeedRedoTask = false;//需要重做任务
                    IsDoneTask = true;
                }
                //保存数据
                excelPackage.Save();
            }//关闭Excel
        }

        protected virtual void TrialRedoCheck() //重做条件
        {

            //如果用户按下手柄的时，与目标点之间的距离大于0.5，记录用户无效，并开始一次新的任务。
            float targetOffset = OffsetTarget();
            if (targetOffset > 0.5) //距离大于0.5m
            {
                ActiveCanvas();
                tips.text = "没到达目标点，请规范地完成任务。\n按开始trigger重新开始任务。";
                IsNeedRedoTask = true;//需要重做任务
            }
            else
            {
                ActiveCanvas();
                tips.text = "任务完成，数据已被记录。\n按开始按钮开启下一个任务。";
                IsNeedRedoTask = false;//需要重做任务
                IsDoneTask = true;
            }
        }

        protected virtual void DataRecordMovementtoSpeed() //数据记录映射前的运动，映射后的速度之间的关系
        {
            string FilePath = Application.streamingAssetsPath + "/DataRecordMovementToSpeed.xlsx";

            //获取excel的信息
            FileInfo fileInfo = new FileInfo(FilePath);
            //通过excel文件信息打开excel表格
            using (excelPackage = new ExcelPackage(fileInfo))
            {

                worksheet = excelPackage.Workbook.Worksheets[excelPackage.Workbook.Worksheets.Count];
                //if (Participant.CompareTo("test") == 1)
                //{
                int endRow = worksheet.Dimension.End.Row;
                for (int i = 0; i < bodyMotion.Count; i++)
                {
                    endRow = worksheet.Dimension.End.Row + 1;
                    //ebug.Log(endRow);
                    //存储一个测试的所有数据
                    worksheet.Cells[endRow, 1].Value = Participant; //参与者
                    worksheet.Cells[endRow, 2].Value = BlockNum;  //测试的block
                    worksheet.Cells[endRow, 3].Value = recordNum; //实验次数
                    worksheet.Cells[endRow, 4].Value = BodyCodeName(); //使用的肢体
                    worksheet.Cells[endRow, 5].Value = FunCodeName();  //使用的函数
                    worksheet.Cells[endRow, 6].Value = TargetCodeName(); //使用的目标距离
                    worksheet.Cells[endRow, 7].Value = movementAngle[i];  //运动输入
                    worksheet.Cells[endRow, 8].Value = bodyMotion[i];  //运动输入
                    worksheet.Cells[endRow, 9].Value = mappingSpeed[i];  //速度
                    worksheet.Cells[endRow, 10].Value = timeCost[i]; //任务完成时间 
                    worksheet.Cells[endRow, 11].Value = virtualMovement[i];//距离间隔
                   // worksheet.Cells[endRow, 12].Value = window[i];//距离间隔
                }

                if (IsNeedRedoTask) //如果需要重做任务就是无效的，不需要重做任务就是有效的
                {
                    worksheet.Cells[endRow, 12].Value = "InValid"; //任务完成时间
                }
                else
                {
                    worksheet.Cells[endRow, 12].Value = "Valid"; //任务完成时间
                }

                //保存数据
                excelPackage.Save();

            }//关闭Excel

        }

        protected virtual void TestTip() //测试提示
        {
            float taskOffset1 = OffsetTarget();
            if (taskOffset1 > 0.5)
            {
                ActiveCanvas();
                tips.text = "该次任务数据无效，请认真重新完成任务。\n按开始trigger重新开始任务。";
                IsNeedRedoTask = true;//需要重做任务
            }
            else
            {
                ActiveCanvas();
                tips.text = "任务完成，数据已被记录。\n按开始按钮开启下一个任务。";
                IsNeedRedoTask = false;//需要重做任务
                IsDoneTask = true;
            }
        }
        /// <summary>
        /// 数据存储
        /// </summary>


        /// <summary>
        /// 按钮控制
        /// </summary>

        protected virtual void EngageButtonPressed(object sender, ControllerInteractionEventArgs e)
        {
            engagedController = e.controllerReference;

            if (IsNeedRedoTask)
            {

                ActiveCanvas();
                tips.text = "您正在重做任务"; //具有时间差，用户等待指示出现后才开始任务
                Invoke("ChangTip", 1f);
                SumX = 0;
                SumErrorDis = 0;
                SumRightDis = 0;
                ClearList(); //先清空列表，再开启携程记录数据
                movementEngaged = true; //当开始运动的时候
                startTime = System.DateTime.Now.Ticks; //开始记录时间点
                                                       // mCoroutine = StartCoroutine(RecordDataPoint(0.04f)); //开始携程记录用户进行一次trial期间的采样点
                IsPressReset = false;
            } //是否需要重做任务
            else
            {
                if (IsDoneTask)
                {
                    if (UsingBody == 5 && recordNum == 0)
                    {
                        PreTrackerYPos = tracker2.transform.localPosition.y;
                        Debug.Log(PreTrackerYPos);

                    }

                    IsPressReset = false;
                    RandomEvent(); //随机项目
                    recordNum++;
                    recordPoint = 0;
                    IsDoneTask = false;
                    if (Participant.Equals("test"))
                    {
                        if (recordNum > 4) //如果大于48次直接return，不允许开始
                        {
                            tips.text = "您已完成体验,可以开始正式实验。";
                            return;
                        }
                    }
                    else
                    {
                        if (recordNum > 15) //如果大于48次直接return，不允许开始 4种函数,3种距离,重复2次,现在是4种函数,5种距离,重复两次,共40
                        {
                            tips.text = "您已完成所有组合的测试。";
                            return;
                        }
                    }
                    SumX = 0;
                    SumErrorDis = 0;
                    SumRightDis = 0;
                    ClearList();
                    movementEngaged = true;
                    startTime = System.DateTime.Now.Ticks;
                    //  mCoroutine = StartCoroutine(RecordDataPoint(0.04f)); //②开始携程记录用户进行一次trial期间的采样点
                } //是否完成了一次任务
                else
                {
                    if (Participant.Equals("test"))
                    {
                        if (recordNum > 4) //如果大于48次直接return，不允许开始
                        {
                            tips.text = "您已完成体验,可以开始正式实验。";
                            return;
                        }
                    }
                    else
                    {
                        if (recordNum > 15) //如果大于48次直接return，不允许开始
                        {
                            tips.text = "您已完成所有组合的测试。";
                            return;
                        }
                    }
                    tips.text = "您还未完成当前任务，\n请勿随意切换任务。";
                    movementEngaged = true;
                    Invoke("ChangTip", 1f);
                }           //没有完成任务，只是误触，无需调用携程
            }
        }

        protected virtual void ChangTip()
        {
            tips.text = "使用“" + MappingDisplay + "”到指示点";
            Invoke("CloseCanvas", 3f);
        }


        protected virtual void EngageButtonReleased(object sender, ControllerInteractionEventArgs e)
        {
            // If the button is released, clear all the lists.
            for (int i = 0; i < trackedObjects.Count; i++)
            {
                Transform trackedObj = trackedObjects[i];
                VRTK_SharedMethods.GetDictionaryValue(movementList, trackedObj, new List<float>()).Clear();
            }
            initialGaze = Vector3.zero;

            //movementEngaged = false;
            engagedController = null;
        }

        private void EndButtonReleased(object sender, ControllerInteractionEventArgs e)
        {
            movementEngaged = false;
            engagedController = null;
        }

        private void EndButtonPressed(object sender, ControllerInteractionEventArgs e) //EndButton是圆盘键
        {
            engagedController = e.controllerReference;
            picture.SetActive(true);
            // StopRecord(); //⑤停止携程，不再记录采样点 
            //  StopCoroutine(mCoroutine);

            TrialRedoCheck(); //⑥检查用户是否到达目标点（trial是否有效）

            DataRecordMovementtoSpeed(); //⑦将列表的数据一次性写入excel

            ClearList(); //⑧清空列表

            if (Participant.Equals("test"))
            {
                TestTip();//判断是否需要重做
                ResetViewPoint();
                if (recordNum > 4)
                {
                    ActiveCanvas();
                    tips.text = "您已完成体验,可以开始正式实验。";
                }
            }
            else
            {
                if (recordNum > 15)
                {
                    ActiveCanvas();
                    tips.text = "您已完成所有组合的测试。";
                    return;
                }
                //DataRecord();//任务结束时记录数据
                ResetViewPoint();
                IsPressReset = true;
                SumX = 0;
                SumErrorDis = 0;
                SumRightDis = 0;

            }
        }

        protected int RecordPoint()
        {
            recordPoint++;
            return recordPoint;
        }

        protected virtual Quaternion DetermineAverageControllerRotation()
        {
            // Build the average rotation of the controller(s)
            Quaternion newRotation;

            // Both controllers are present
            if (controllerLeftHand != null && controllerRightHand != null)
            {
                newRotation = AverageRotation(controllerLeftHand.transform.rotation, controllerRightHand.transform.rotation);
            }
            // Left controller only
            else if (controllerLeftHand != null && controllerRightHand == null)
            {
                newRotation = controllerLeftHand.transform.rotation;
            }
            // Right controller only
            else if (controllerRightHand != null && controllerLeftHand == null)
            {
                newRotation = controllerRightHand.transform.rotation;
            }
            // No controllers!
            else
            {
                newRotation = Quaternion.identity;
            }

            return newRotation;
        }

        // Returns the average of two Quaternions
        protected virtual Quaternion AverageRotation(Quaternion rot1, Quaternion rot2)
        {
            return Quaternion.Slerp(rot1, rot2, 0.5f);
        }

        protected virtual void SetControllerListeners(GameObject controller, bool controllerState, ref bool subscribedState, bool forceDisabled = false)
        {
            if (controller != null)
            {
                bool toggleState = (forceDisabled ? false : controllerState);
                ToggleControllerListeners(controller, toggleState, ref subscribedState);
            }
        }

        protected virtual void ToggleControllerListeners(GameObject controller, bool toggle, ref bool subscribed)
        {
            VRTK_ControllerEvents controllerEvents = controller.GetComponentInChildren<VRTK_ControllerEvents>();
            if (controllerEvents != null)
            {
                //If engage button has changed, then unsubscribe the previous engage button from the events
                if (engageButton != previousEngageButton && subscribed)
                {
                    controllerEvents.UnsubscribeToButtonAliasEvent(previousEngageButton, true, EngageButtonPressed);
                    controllerEvents.UnsubscribeToButtonAliasEvent(previousEngageButton, false, EngageButtonReleased);
                    subscribed = false;
                }
                if (endButton != previousEndButton && subscribed)
                {
                    controllerEvents.UnsubscribeToButtonAliasEvent(previousEndButton, true, EndButtonPressed);
                    controllerEvents.UnsubscribeToButtonAliasEvent(previousEndButton, false, EndButtonReleased);
                    subscribed = false;
                }

                if (toggle && !subscribed)
                {
                    controllerEvents.SubscribeToButtonAliasEvent(engageButton, true, EngageButtonPressed);
                    controllerEvents.SubscribeToButtonAliasEvent(engageButton, false, EngageButtonReleased);
                    controllerEvents.SubscribeToButtonAliasEvent(endButton, true, EndButtonPressed);
                    controllerEvents.SubscribeToButtonAliasEvent(endButton, false, EndButtonReleased);
                    subscribed = true;
                }
                else if (!toggle && subscribed)
                {
                    controllerEvents.UnsubscribeToButtonAliasEvent(engageButton, true, EngageButtonPressed);
                    controllerEvents.UnsubscribeToButtonAliasEvent(engageButton, false, EngageButtonReleased);
                    controllerEvents.UnsubscribeToButtonAliasEvent(endButton, true, EndButtonPressed);
                    controllerEvents.UnsubscribeToButtonAliasEvent(endButton, false, EndButtonReleased);
                    subscribed = false;
                }
            }
        }


    }
}

