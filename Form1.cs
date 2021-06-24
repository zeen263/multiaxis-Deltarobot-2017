using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using CeDll = CEDLL.SafeNativeMethods;
using CeDef = CEDLL.d;


namespace Deltaforce_final
{
    public partial class Form1 : Form
    {
        private SerialPort ardSerialPort = new SerialPort();

        int AmountOfNode = 3;                  // 탐색할 노드 갯수 
        uint TIMEOUT = 60;                     // 타임아웃(노드 탐색 시간, ms)

        int nIsSearchedDevice = 0;             // 탐색된 원격 노드 정보
        int NodeCount = 0;
        int NodeID = 1;

        int LmMAP0 = 0, LmMAP1 = 1;            // 델타로봇 리스트 맵 번호
        int IxMAP0 = 0;                        // 델타로봇 보간 모션의 맵 번호
        int IxMAP1 = 1;

        int TotalNode = -1;                    // 탐색된 노드 갯수
        int AxisChannel = -1;                  // 탐색된 축의 갯수

        int CONSTANT = 0, TRAPEZOIDAL = 1, S_CURVE = 2, KEEP = -1;                        // 스피드패턴 세팅에 필요한 값
        int nRetVal = 0;
        int ORG0 = 99, ORG1 = 99, ORG2 = 99, ORG3 = 99, ORG4 = 99, ORG5 = 99, ORG6 = 99;  // Homing을 위한 ORG 센서

        double x = 0, y = 0, z = 0, t = 0;
        double sx1 = 0, stage = 0, sx2 = 0;


        double[] pos = new double[6];

        int IxMASK0 = 0x3F;                     // 보간제어 마스크 0011 1111      (팔레트 단축로봇은 보간제어하지 않음)
        int LmMASK0 = 0x7F;                     // 리스티드모션 마스크 0111 1111  (리스티드모션은 모든 축에 해당됨)

        int CycleCount = 0;                     // 몇 번째 사이클인지 나타내기
        int MotionCount = 0;                    // 사이클 내에서 몇 번째 작업인지 나타내기
        int MotionUntil = 3;

        string errorcode = " ";

        J_CalLambda CAL_POSITION = new J_CalLambda();


        double[] bolt_Xpos = new double[6];      // 나사꽂이에서 각 볼트의 위치
        double[] bolt_Ypos = new double[6];      //  5  6
        double bolt_Zpos = -69.5;                //  3  4
                                                 //  1  2 


        // 축 번호 : 0~3 : 델타로봇     4 : 단축로봇1     5 : 틸팅스테이지     6 : 단축로봇2 (팔레트)

        //=====================================================================================       
        // 델타 사양                                    단축1 사양
        //=====================================================================================
        // 볼스크류 피치 : 5mm                          볼스크류 피치 : 10mm
        // 분해능        : 1회전당 2000펄스             분해능        : 1회전당 2000펄스
        // mm - 펄스     : 1mm = 400펄스                mm - 펄스     : 1mm = 200펄스
        //=====================================================================================

        //=====================================================================================     
        // 틸팅스테이지 사양                            단축2 (팔레트) 사양
        //=====================================================================================
        // 웜기어   피치 : 2도                          볼스크류 피치 : 4mm
        // 분해능        : 1회전당 500펄스              분해능        : 1회전당 800펄스
        // 각도 - 펄스   : 1도 = 250펄스                mm - 펄스     : 1mm = 200펄스
        //=====================================================================================



        public Form1()
        {
            InitializeComponent();

            ardSerialPort.PortName = "COM4";     // 아두이노 포트번호 4번
            ardSerialPort.BaudRate = 9600;       // 보드 레이트
            ardSerialPort.Open();                // 통신실행
        }


        //===========================================================================
        //   폼 로드시 세팅
        //===========================================================================

        private void Form1_Load(object sender, EventArgs e)
        {
            picDeviceLoad.BackColor = Color.Gray;


            nRetVal = CeDll.ceGnLoad();
            SearchNode();

            CeDll.cemCfgSeqMode_Set(1); // 이송 진행중인 축에 새로운 명령 하달시 처리 (오류처리하지 않고 이전 명령 완료 후에 새 명령을 실행하도록 설정함)

            nRetVal = CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 45000, 100000, 100000);
            nRetVal = CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 45000, 100000, 100000);
            nRetVal = CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 45000, 100000, 100000);
            nRetVal = CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 45000, 100000, 100000);  // 델타로봇

            nRetVal = CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 10000, 40000, 40000); // 단축로봇1
            nRetVal = CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 12000, 80000, 80000);  // 틸팅스테이지
            nRetVal = CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 12000, 50000, 50000);  // 단축로봇2 (팔레트)


            CeDll.cemCfgMioProperty_Set(5, 13, 1);   // 틸팅스테이지 내부의 ORG센서 로직 설정 (B접점)   *나머지 원점센서는 모두 기본값인 A접점


            nRetVal = CeDll.cemCfgOutMode_Set(0, 0);
            nRetVal = CeDll.cemCfgOutMode_Set(1, 0);
            nRetVal = CeDll.cemCfgOutMode_Set(2, 0);
            nRetVal = CeDll.cemCfgOutMode_Set(3, 0); // 델타로봇

            nRetVal = CeDll.cemCfgOutMode_Set(4, 0); // 단축로봇1
            nRetVal = CeDll.cemCfgOutMode_Set(5, 0); // 틸팅스테이지
            nRetVal = CeDll.cemCfgOutMode_Set(6, 0); // 단축로봇2 (팔레트)


            SetBoltPos();
        }


        private void SetBoltPos()                   // 6개 볼트의 위치를 설정
        {
            bolt_Xpos[0] = 23;
            bolt_Ypos[0] = 138;

            bolt_Xpos[1] = 20.3;
            bolt_Ypos[1] = 121.6;

            bolt_Xpos[2] = 34.2;
            bolt_Ypos[2] = 136.2;



            bolt_Xpos[3] = 35.2;
            bolt_Ypos[3] = 122.9;

            bolt_Xpos[4] = 48.4;
            bolt_Ypos[4] = 137.9;

            bolt_Xpos[5] = 48.8;
            bolt_Ypos[5] = 123;

        }



        //===========================================================================
        //   노드 탐색
        //===========================================================================

        private void SearchNode()
        {
            nRetVal = CeDll.ceGnIsSearchedDevice(ref nIsSearchedDevice);  // 원격 노드 탐색 여부 확인


            if (nIsSearchedDevice == CeDef.CE_FALSE)                      // 원격 노드 탐색을 수행하지 않았다면 원격 노드 탐색 수행
                nRetVal = CeDll.ceGnSearchDevice(AmountOfNode, TIMEOUT, CeDef.CE_FALSE, ref NodeCount);

            else                                                          // 원격 노드 탐색이 이미 수행되었다면 재탐색
                nRetVal = CeDll.ceGnReSearchDevice(AmountOfNode, TIMEOUT, CeDef.CE_FALSE, ref NodeCount);


            nRetVal = CeDll.ceGnTotalMotionChannel(ref AxisChannel);
            nRetVal = CeDll.ceGnTotalNode(ref TotalNode);

            lblSearchedAxis.Text = Convert.ToString(AxisChannel);
            lblSearchedNode.Text = Convert.ToString(TotalNode);

        }


        //===========================================================================
        //   TIMER
        //===========================================================================

        private void display_error()
        {
            txtError.Text += "errorcode " + errorcode + Environment.NewLine;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            int IsSearched = -99;

            CeDll.cemStReadMioStatuses(0, ref ORG0);
            CeDll.cemStReadMioStatuses(1, ref ORG1);
            CeDll.cemStReadMioStatuses(2, ref ORG2);
            CeDll.cemStReadMioStatuses(3, ref ORG3);
            CeDll.cemStReadMioStatuses(4, ref ORG4);
            CeDll.cemStReadMioStatuses(5, ref ORG5);
            CeDll.cemStReadMioStatuses(6, ref ORG6);    // ORG센서 신호 받아오기


            if (((ORG0 >> 4) & 0x01) == 1) picORG0.BackColor = Color.YellowGreen;
            else picORG0.BackColor = Color.Gray;

            if (((ORG1 >> 4) & 0x01) == 1) picORG1.BackColor = Color.YellowGreen;
            else picORG1.BackColor = Color.Gray;

            if (((ORG2 >> 4) & 0x01) == 1) picORG2.BackColor = Color.YellowGreen;
            else picORG2.BackColor = Color.Gray;

            if (((ORG3 >> 4) & 0x01) == 1) picORG3.BackColor = Color.YellowGreen;
            else picORG3.BackColor = Color.Gray;

            if (((ORG4 >> 4) & 0x01) == 1) picORGSx.BackColor = Color.YellowGreen;
            else picORGSx.BackColor = Color.Gray;

            if (((ORG5 >> 4) & 0x01) == 1) picORGTilt.BackColor = Color.YellowGreen;
            else picORGTilt.BackColor = Color.Gray;

            if (((ORG6 >> 4) & 0x01) == 1) picORGPallet.BackColor = Color.YellowGreen;
            else picORGPallet.BackColor = Color.Gray;


            if (nRetVal != CeDef.ceERR_NONE)       // 수행 실패
            {
                errorcode = " " + Convert.ToString(nRetVal);
                display_error();
            }

            nRetVal = CeDll.ceGnIsSearchedDevice(ref IsSearched);

            if (IsSearched == 1)
            {
                errorcode = " ";
                picDeviceLoad.BackColor = Color.YellowGreen;
            }
            else picDeviceLoad.BackColor = Color.Gray;


            lblXpos.Text = Convert.ToString(x);
            lblYpos.Text = Convert.ToString(y);
            lblZpos.Text = Convert.ToString(z);
            lblTpos.Text = Convert.ToString(t);

        }



        //===========================================================================
        //   장치 로드/언로드
        //===========================================================================

        private void lblLoadUnload_Click(object sender, EventArgs e)
        {
            int IsLoaded = -99;

            nRetVal = CeDll.ceGnIsSearchedDevice(ref IsLoaded);

            if (IsLoaded == 1) nRetVal = CeDll.ceGnUnload();
            else
            {
                nRetVal = CeDll.cemIxUnMap(IxMAP0);
                nRetVal = CeDll.ceGnLoad();
            }
        }



        //===========================================================================
        //   원점세팅
        //===========================================================================

        private void btnAbsoluteHome_Click(object sender, EventArgs e)  // 원점 위치
        {

            CeDll.cemHomeConfig_Set(0, 1, 1, 0, 10 * 400, 0);   // Axis(0번 축), HomeMode(모드 1), Dir(양의 방향), Ezcount(사용안함), EscDist(원점탈출거리), Offset(추가 이송 거리)
            CeDll.cemHomeConfig_Set(1, 1, 1, 0, 10 * 400, 0);
            CeDll.cemHomeConfig_Set(2, 1, 1, 0, 10 * 400, 0);
            CeDll.cemHomeConfig_Set(3, 1, 1, 0, 10 * 400, 0);   // 델타

            CeDll.cemHomeConfig_Set(4, 1, 1, 0, 10 * 200, 0);   // 단축
            CeDll.cemHomeConfig_Set(5, 1, 1, 0, 10 * 250, 0);   // 틸팅
            CeDll.cemHomeConfig_Set(6, 1, 1, 0, 10 * 200, 100); // 팔레트


            CeDll.cemHomePosClrMode_Set(0, 1);
            CeDll.cemHomePosClrMode_Set(1, 1);
            CeDll.cemHomePosClrMode_Set(2, 1);
            CeDll.cemHomePosClrMode_Set(3, 1); // 델타

            CeDll.cemHomePosClrMode_Set(4, 1); // 단축
            CeDll.cemHomePosClrMode_Set(5, 1); // 틸팅
            CeDll.cemHomePosClrMode_Set(6, 1); // 팔레트


            CeDll.cemHomeSpeedPattern_Set(0, S_CURVE, 5000, 20000, 20000, 1500);
            CeDll.cemHomeSpeedPattern_Set(1, S_CURVE, 4500, 20000, 20000, 1500);
            CeDll.cemHomeSpeedPattern_Set(2, S_CURVE, 4500, 20000, 20000, 1500);
            CeDll.cemHomeSpeedPattern_Set(3, S_CURVE, 5000, 20000, 20000, 1500); // 델타

            CeDll.cemHomeSpeedPattern_Set(4, S_CURVE, 5000, 20000, 20000, 1500); // 단축
            CeDll.cemHomeSpeedPattern_Set(5, S_CURVE, 5000, 20000, 20000, 1500); // 틸팅
            CeDll.cemHomeSpeedPattern_Set(6, S_CURVE, 5000, 20000, 20000, 1500); // 팔레트


            CeDll.cemHomeMoveStart(0);
            CeDll.cemHomeMoveStart(1);
            CeDll.cemHomeMoveStart(2);
            CeDll.cemHomeMoveStart(3);

            CeDll.cemHomeMoveStart(4);
            CeDll.cemHomeMoveStart(5);
            CeDll.cemHomeMoveStart(6);


            x = 0; y = 0; z = 0; t = 0;
            sx1 = 0; stage = 0; sx2 = 0;



        }



        //===========================================================================
        //  공정 시작 (작업 3회 1사이클)
        //===========================================================================

        private void btnStart_Click(object sender, EventArgs e)
        {
            CycleCount++;

            for (MotionCount = 0; MotionCount < 3; MotionCount++)
            {
                MoveToFeeder();
                //  Thread.Sleep(200); // 스테이지에 작업물을 올리는 데 필요한 시간

                PickBolt_Down();
                PickBolt_Up();


                MoveToStage();

                MoveToHole();

                AssembleStart();

                ardSerialPort.Write("0"); // 드라이버 off

                SeperatePoint();
                ardSerialPort.Write("2"); // 서보모터 작동

                Thread.Sleep(500);  // 적재되도록 잠시 대기
            }
        }



        //===========================================================================
        //  피더 위치로 이동
        //===========================================================================

        private void btnFeederPoint_Click(object sender, EventArgs e)
        {
            MoveToFeeder();

            PickBolt_Down();

            PickBolt_Up();
        }

        //===========================================================================

        private void MoveToFeeder()
        {
            double[] pos_feed = new double[6];
            double pall = 0;

            nRetVal = CeDll.cemLmBeginList(LmMAP0, NodeID, 0xFF);
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);
            nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 20000, 70000, 70000);

            if (MotionCount < 3) MotionCount++;
            if (MotionCount >= 3) MotionCount = 0;

            x = bolt_Xpos[MotionCount];                 // i번째 모션이면 bolt_Xpos[i] 좌표를 가져온다
            y = bolt_Ypos[MotionCount];
            z = bolt_Zpos + 15;

            t = 0; sx1 = 0; stage = 0;


            CAL_POSITION.Calculate(x, y, z, t);

            pos_feed[0] = 400 * CAL_POSITION.lambda0;
            pos_feed[1] = 400 * CAL_POSITION.lambda1;
            pos_feed[2] = 400 * CAL_POSITION.lambda2;
            pos_feed[3] = 400 * CAL_POSITION.lambda3;
            pos_feed[4] = 200 * sx1;
            pos_feed[5] = 250 * stage;

            if ((MotionCount % 3) == 0) pall = 0;              // 적재 팔레트의 위치
            else if ((MotionCount % 3) == 1) pall = -9000;
            else if ((MotionCount % 3) == 2) pall = -20000;

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos_feed); // 볼트 피딩 지점의 15mm 위로 이동
            CeDll.cemSxMoveToStart(6, pall);

            CeDll.cemLmStart(LmMAP0);

            while (true)                          // 리스티드모션 완료 대기
            {
                int IsDone = 0;
                nRetVal = CeDll.cemLmIsDone(LmMAP0, ref IsDone);
                if (IsDone == 1) { break; }
            }

            CeDll.cemIxUnMap(IxMAP0);
            CeDll.cemLmEndList(LmMAP0);

        }

        //===========================================================================

        private void PickBolt_Down()
        {
            double[] pos_bolt = new double[6];

            CeDll.cemLmBeginList(LmMAP0, NodeID, LmMASK0);
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0x3F, 0);
            // nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 10000, 30000, 30000);

            x = bolt_Xpos[MotionCount];                 // i번째 모션이면 bolt_Xpos[i] 좌표를 가져온다
            y = bolt_Ypos[MotionCount];
            z = bolt_Zpos;

            t = 0; sx1 = 0; stage = 0;            // DOWN


            CAL_POSITION.Calculate(x, y, z, t);

            pos_bolt[0] = 400 * CAL_POSITION.lambda0;
            pos_bolt[1] = 400 * CAL_POSITION.lambda1;
            pos_bolt[2] = 400 * CAL_POSITION.lambda2;
            pos_bolt[3] = 400 * CAL_POSITION.lambda3;

            nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 15, 10000, 10000);  // 저속으로 다이브하기 위해 속도 재설정         
            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos_bolt);


            CeDll.cemLmStart(LmMAP0);

            while (true)                          // 리스티드모션 완료 대기
            {
                int IsDone = 0;
                nRetVal = CeDll.cemLmIsDone(LmMAP0, ref IsDone);
                if (IsDone == 1) { break; }
            }

            CeDll.cemIxUnMap(IxMAP0);
            CeDll.cemLmEndList(LmMAP0);


          //  Thread.Sleep(300);

        }


        //===========================================================================

        private void PickBolt_Up()
        {
            double[] pos_bolt = new double[6];

            ardSerialPort.Write("1");
            Thread.Sleep(300);
            ardSerialPort.Write("0");
            Thread.Sleep(300);

            CeDll.cemLmBeginList(LmMAP0, NodeID, LmMASK0);
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0x3F, 0);
            nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 30, 10000, 10000);

            x = bolt_Xpos[MotionCount];                 // i번째 모션이면 bolt_Xpos[i] 좌표를 가져온다
            y = bolt_Ypos[MotionCount];
            z = bolt_Zpos + 15;

            t = 0; sx1 = 0; stage = 0;            // UP


            CAL_POSITION.Calculate(x, y, z, t);

            pos_bolt[0] = 400 * CAL_POSITION.lambda0;
            pos_bolt[1] = 400 * CAL_POSITION.lambda1;
            pos_bolt[2] = 400 * CAL_POSITION.lambda2;
            pos_bolt[3] = 400 * CAL_POSITION.lambda3;
            pos_bolt[4] = 200 * sx1;
            pos_bolt[5] = 250 * stage;

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos_bolt);


            CeDll.cemLmStart(LmMAP0);

            while (true)                          // 리스티드모션 완료 대기
            {
                int IsDone = 0;
                nRetVal = CeDll.cemLmIsDone(LmMAP0, ref IsDone);
                if (IsDone == 1) { break; }
            }

            CeDll.cemIxUnMap(IxMAP0);
            CeDll.cemLmEndList(LmMAP0);
        }


        //===========================================================================
        //   작업 시작 위치로 이동
        //===========================================================================

        private void btnStartPoint_Click(object sender, EventArgs e)
        {
            MoveToStage();

            MoveToHole();

        }

        private void MoveToStage()
        {
            double[] pos_stage = new double[6];

            CeDll.cemLmBeginList(LmMAP0, NodeID, LmMASK0);

            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0x3F, 0);
            nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 20000, 70000, 70000);  // 원래의 작업속도 복구


            x = -64.7; y = 120; z = -23; t = 0;       // 키팝 구멍의 위로 이동
            sx1 = -102; stage = 0;

            CAL_POSITION.Calculate(x, y, z, t);

            pos_stage[0] = 400 * CAL_POSITION.lambda0;
            pos_stage[1] = 400 * CAL_POSITION.lambda1;
            pos_stage[2] = 400 * CAL_POSITION.lambda2;
            pos_stage[3] = 400 * CAL_POSITION.lambda3;
            pos_stage[4] = 200 * sx1;
            pos_stage[5] = 250 * stage;

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos_stage);



            CeDll.cemLmStart(LmMAP0);

            while (true)                          // 리스티드모션 완료 대기
            {
                int IsDone = 0;
                nRetVal = CeDll.cemLmIsDone(LmMAP0, ref IsDone);
                if (IsDone == 1) { break; }
            }

            CeDll.cemIxUnMap(IxMAP0);
            CeDll.cemLmEndList(LmMAP0);
        }


        private void MoveToHole()
        {
            double[] pos_hole = new double[6];

            CeDll.cemLmBeginList(LmMAP0, NodeID, LmMASK0);
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0x3F, 0);
            nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 20000, 70000, 70000);
 

            x = -65.2; y = 121; z = -39; t = 0;       // 키팝 구멍의 위로 이동
            sx1 = -102; stage = 0;

            CAL_POSITION.Calculate(x, y, z, t);     // 스테이지 위 작업물의 위치로 10mm 하강

            pos_hole[0] = 400 * CAL_POSITION.lambda0;
            pos_hole[1] = 400 * CAL_POSITION.lambda1;
            pos_hole[2] = 400 * CAL_POSITION.lambda2;
            pos_hole[3] = 400 * CAL_POSITION.lambda3;
            pos_hole[4] = 200 * sx1;
            pos_hole[5] = 250 * stage;


            nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 70, 30000, 30000);  // 천천히 하강
            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos_hole);
            nRetVal = CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 20000, 70000, 70000);  // 원래의 작업속도 복구


            CeDll.cemLmStart(LmMAP0);


            while (true)                          // 리스티드모션 완료 대기
            {
                int IsDone = 0;
                nRetVal = CeDll.cemLmIsDone(LmMAP0, ref IsDone);
                if (IsDone == 1) { break; }
            }

            CeDll.cemIxUnMap(IxMAP0);
            CeDll.cemLmEndList(LmMAP0);
        }


        //===========================================================================
        //   작업 수행 (작업 끝점으로 이동)
        //===========================================================================

        private void btnEndPoint_Click(object sender, EventArgs e)
        {
            AssembleStart();
            ardSerialPort.Write("0"); // 드라이버 off

            SeperatePoint();
            ardSerialPort.Write("2"); // 서보모터 작동
        }

        //===========================================================================

        private void AssembleStart()
        {
            double segment = 50;
            int LmMAP1 = 1, IxMAP0 = 0;

            x = -65.2; y = 121; z = -39; t = 0; sx1 = -102; stage = 0;                              // 시작점의 좌표 (이전 위치)
            double x_end = -61, y_end = -17, z_end = -44, t_end = -20, sx1_end = -239, stage_end = -15;  // 끝점의 좌표 (목표 위치)    

            double x_seg, y_seg, z_seg, t_seg;
            double sx1_seg, stage_seg;

            double[] pos_start = new double[6];
            double[] pos_old = new double[6];
            double[] dist = new double[6];


            x_seg = (x_end - x) / segment;      // 한 세그먼트당 이동량
            y_seg = (y_end - y) / segment;
            z_seg = (z_end - z) / segment;
            t_seg = (t_end - t) / segment;
            sx1_seg = ((sx1_end - sx1) / segment);
            stage_seg = (stage_end - stage) / segment;


            CAL_POSITION.Calculate(x, y, z, t);     // 시작점까지 가는 데 필요한 람다값

            pos_start[0] = CAL_POSITION.lambda0;
            pos_start[1] = CAL_POSITION.lambda1;
            pos_start[2] = CAL_POSITION.lambda2;
            pos_start[3] = CAL_POSITION.lambda3;
            pos_start[4] = sx1;
            pos_start[5] = stage;


            pos_old[0] = CAL_POSITION.lambda0;
            pos_old[1] = CAL_POSITION.lambda1;
            pos_old[2] = CAL_POSITION.lambda2;
            pos_old[3] = CAL_POSITION.lambda3;
            pos_old[4] = sx1;
            pos_old[5] = stage;


            CeDll.cemLmBeginList(LmMAP1, NodeID, 0x3F);
            CeDll.cemIxMapAxes(IxMAP0, NodeID, 0x3F, 0);
            CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, CONSTANT, 90, 100, 100);


            for (int i = 0; i < segment; i++)
            {
                x += x_seg;     // 세그먼트의 누적합
                y += y_seg;
                z += z_seg;
                t += t_seg;
                sx1 += sx1_seg;
                stage += stage_seg;


                CAL_POSITION.Calculate(x, y, z, t); // 세그먼트의 누적합으로 이동하기 위한 역기구학 계산

                dist[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
                dist[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
                dist[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
                dist[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
                dist[4] = Convert.ToInt32(200 * sx1);
                dist[5] = Convert.ToInt32(250 * stage);

                nRetVal = CeDll.cemIxLineTo(IxMAP0, dist, 0);
            }


            ardSerialPort.Write("1"); // 드라이버 on
            nRetVal = CeDll.cemLmStart(LmMAP1);

            while (true)                          // 리스티드모션 완료 대기
            {
                int IsDone = 0;
                nRetVal = CeDll.cemLmIsDone(LmMAP1, ref IsDone);
                if (IsDone == 1) { CeDll.cemLmStop(LmMAP1, 1, 90, 1); pictureBox1.BackColor = Color.DarkTurquoise; break; }
            }

            nRetVal = CeDll.cemIxUnMap(IxMAP0);
            nRetVal = CeDll.cemLmEndList(LmMAP1);
            
        }

        //===========================================================================

        private void SeperatePoint()
        {
            double[] dist = new double[6];

            double x_end = -64, y_end = -16, z_end = -49, t_end = -20, sx1_end = -239, stage_end = -15;  // 끝점의 좌표 (목표 위치)    


            CeDll.cemLmBeginList(LmMAP0, NodeID, LmMASK0);
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0x3F, 0);
            CeDll.cemIxSpeedPattern_Set(IxMAP0, 0, S_CURVE, 35000, 15000, 15000);


            x = x_end + 26; z = z_end + 50;         // 스테이지에 수직을 유지한 채로 헤드 들어올리기 

            CAL_POSITION.Calculate(x, y, z, t);

            dist[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            dist[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            dist[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            dist[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            dist[4] = Convert.ToInt32(200 * sx1);
            dist[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineTo(IxMAP0, dist, 0);



            x = 37.5; y = 140.5; z = -58; t = 0; sx1 = -380; stage = -32;  // 헤드는 피더로 귀환, 스테이지는 계속 움직임 

            CAL_POSITION.Calculate(x, y, z, t); 

            dist[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            dist[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            dist[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            dist[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            dist[4] = Convert.ToInt32(200 * sx1);
            dist[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineTo(IxMAP0, dist, 0);


            CeDll.cemLmStart(LmMAP0);

            while (true)                          // 리스티드모션 완료 대기
            {
                int IsDone = 0;
                nRetVal = CeDll.cemLmIsDone(LmMAP0, ref IsDone);
                if (IsDone == 1) { break; }
            }

            CeDll.cemIxUnMap(IxMAP0);
            CeDll.cemLmEndList(LmMAP0);
        }



        //===========================================================================
        //   좌표 입력식 이동
        //===========================================================================
        private void btnMovetoPos_Click(object sender, EventArgs e)
        {
            double[] pos = new double[4];

            CeDll.cemIxMapAxes(IxMAP0, NodeID, 0x0F, 0);

            x = Convert.ToDouble(txtX.Text);
            y = Convert.ToDouble(txtY.Text);
            z = Convert.ToDouble(txtZ.Text);
            t = Convert.ToDouble(txtT.Text);

            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = 400 * CAL_POSITION.lambda0;
            pos[1] = 400 * CAL_POSITION.lambda1;
            pos[2] = 400 * CAL_POSITION.lambda2;
            pos[3] = 400 * CAL_POSITION.lambda3;

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);
        }








        //===========================================================================
        //   단축 이송 (조그)
        //===========================================================================

        //===========================================================================
        // Axis0
        //===========================================================================
        private void btn0Posi_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(0, 1);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        private void btn0Posi_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(0, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        //---------------------------------------------------------------------------
        private void btn0Nega_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 30000, 100000, 100000);

        }
        private void btn0Nega_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(0, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(0, TRAPEZOIDAL, 30000, 100000, 100000);
        }


        //===========================================================================
        // Axis 1
        //===========================================================================
        private void btn1Posi_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(1, 1);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        private void btn1Posi_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(1, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        //---------------------------------------------------------------------------
        private void btn1Nega_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(1, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        private void btn1Nega_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(1, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(1, TRAPEZOIDAL, 30000, 100000, 100000);
        }


        //===========================================================================
        // Axis 2
        //===========================================================================
        private void btn2Posi_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(2, 1);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        private void btn2Posi_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(2, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        //---------------------------------------------------------------------------
        private void btn2Nega_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(2, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        private void btn2Nega_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 5000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(2, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(2, TRAPEZOIDAL, 30000, 100000, 100000);
        }


        //===========================================================================
        // Axis 3
        //===========================================================================
        private void btn3Posi_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(3, 1);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        private void btn3Posi_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(3, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        //---------------------------------------------------------------------------
        private void btn3Nega_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(3, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 30000, 100000, 100000);
        }
        private void btn3Nega_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(3, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(3, TRAPEZOIDAL, 30000, 100000, 100000);
        }


        //===========================================================================
        // 단축로봇
        //===========================================================================
        private void btnSxPosi_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(4, 1);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 20000, 80000, 80000);
        }
        private void btnSxPosi_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(4, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 20000, 80000, 80000);
        }
        //---------------------------------------------------------------------------
        private void btnSxNega_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(4, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 20000, 80000, 80000);
        }
        private void btnSxNega_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(4, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(4, TRAPEZOIDAL, 20000, 80000, 80000);
        }



        //===========================================================================
        // 틸팅스테이지
        //===========================================================================
        private void btnTiltPosi_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(5, 1);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 20000, 80000, 80000);
        }
        private void btnTiltPosi_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(5, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 20000, 80000, 80000);
        }
        //---------------------------------------------------------------------------
        private void btnTiltNega_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(5, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 20000, 80000, 80000);
        }
        private void btnTiltNega_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(5, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(5, TRAPEZOIDAL, 20000, 80000, 80000);
        }



        //===========================================================================
        // 적재 팔레트
        //===========================================================================
        private void btnPalletPosi_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(6, 1);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 10000, 30000, 30000);
        }
        private void btnPalletPosi_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(6, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 10000, 30000, 30000);
        }
        //---------------------------------------------------------------------------
        private void btnPalletNega_MouseDown(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxVMoveStart(6, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 10000, 30000, 30000);
        }
        private void btnPalletNega_MouseUp(object sender, MouseEventArgs e)
        {
            CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 7000, 40000, 40000);
            nRetVal = CeDll.cemSxStop(6, 0, 0);

            nRetVal = CeDll.cemCfgSpeedPattern_Set(6, TRAPEZOIDAL, 10000, 30000, 30000);
        }





        //===========================================================================
        // X축 조그
        //===========================================================================
        private void btnXPosi_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            x += 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }

        //---------------------------------------------------------------------------

        private void btnXNega_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            x -= 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }


        //===========================================================================
        // Y축 조그
        //===========================================================================
        private void btnYPosi_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            y += 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }

        //---------------------------------------------------------------------------

        private void btnYNega_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            y -= 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }


        //===========================================================================
        // Z축 조그
        //===========================================================================
        private void btnZPosi_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            z += 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }

        //---------------------------------------------------------------------------

        private void btnZNega_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            z -= 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }


        //===========================================================================
        // Tilt 조그
        //===========================================================================
        private void btnTPosi_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            t += 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }

        //---------------------------------------------------------------------------

        private void btnTNega_MouseDown(object sender, MouseEventArgs e)
        {
            nRetVal = CeDll.cemIxMapAxes(IxMAP0, NodeID, 0xFF, 0);

            t -= 1;
            CAL_POSITION.Calculate(x, y, z, t);

            pos[0] = Convert.ToInt32(400 * CAL_POSITION.lambda0);
            pos[1] = Convert.ToInt32(400 * CAL_POSITION.lambda1);
            pos[2] = Convert.ToInt32(400 * CAL_POSITION.lambda2);
            pos[3] = Convert.ToInt32(400 * CAL_POSITION.lambda3);
            pos[4] = Convert.ToInt32(200 * sx1);
            pos[5] = Convert.ToInt32(250 * stage);

            nRetVal = CeDll.cemIxLineToStart(IxMAP0, pos);

            Thread.Sleep(30);

            CeDll.cemIxUnMap(IxMAP0);
        }




        //===========================================================================
        // 동작 정지 및 프로그램 종료
        //===========================================================================

        private void btnStop_Click(object sender, EventArgs e)
        {

            nRetVal = CeDll.cemLmStop(LmMAP1, 0, 300, 1);
            nRetVal = CeDll.cemLmStop(LmMAP0, 0, 300, 1);
            nRetVal = CeDll.cemIxStop(IxMAP0);

            nRetVal = CeDll.cemSxStop(0, 0, 0);
            nRetVal = CeDll.cemSxStop(1, 0, 0);
            nRetVal = CeDll.cemSxStop(2, 0, 0);
            nRetVal = CeDll.cemSxStop(3, 0, 0);
            nRetVal = CeDll.cemSxStop(4, 0, 0);
            nRetVal = CeDll.cemSxStop(5, 0, 0);
            nRetVal = CeDll.cemSxStop(6, 0, 0);

        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            CeDll.ceGnUnload();
            nRetVal = CeDll.cemIxUnMap(IxMAP0);

            Application.Exit();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (ardSerialPort.IsOpen == true)
              {
                  ardSerialPort.Close();
              }
        }

        private void label4_Click(object sender, EventArgs e)
        {
            ardSerialPort.Write("0"); // 드라이버 off
        }

        private void lblErrorClear_Click(object sender, EventArgs e)
        {
            txtError.Text = " ";
            ardSerialPort.Write("2");
        }

        private void chkCyc1_CheckedChanged(object sender, EventArgs e)
        {
            if (chkCyc1.Checked == true) MotionUntil = 3;

            if (chkCyc1.Checked == false) MotionUntil = 6;

        }

        private void chkCyc2_CheckedChanged(object sender, EventArgs e)
        {
            if (chk.Checked == true) MotionUntil = 6;

            if (chk.Checked == false) MotionUntil = 3;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            CeDll.cemSxMoveToStart(4, Convert.ToInt32(textBox1.Text)*(200));
        }

        private void button2_Click(object sender, EventArgs e)
        {
            CeDll.cemSxMoveToStart(6, Convert.ToInt32(textBox2.Text));
        }


    }
        
    }

